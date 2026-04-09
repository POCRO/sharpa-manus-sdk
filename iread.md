## 技术路线分析

### 整体数据流

```
Manus 手套
    │  ZMQ PUB  tcp://<host>:2044
    │  Protobuf: MocapKeypoints
    ▼
zmq_mocap_subscriber_process        独立进程，~1000Hz 轮询
    │  mp.Queue(maxsize=10)          满则丢弃最旧帧，保证低延迟
    ▼
DualHandRetargetingSystem.run()     主循环
    │  _protobuf_to_numpy()
    │  left/right keypoints: (25, 7)  [x, y, z, qw, qx, qy, qz]
    ▼
MultiprocessOptimizationManager     hand_retargeting_optimizer.so
    │  CasADi + IPOPT 非线性优化，左右手并行进程
    │  输出: filtered_angles (22,) rad
    ▼
JointSmoother                       后台线程，默认 250Hz
    │  二阶弹簧-阻尼低通滤波
    ▼
WaveController → MockGlove
    │  UDP 广播，HA4 二进制协议，CRC32 校验
    ▼
Sharpa HA4 灵巧手硬件
```

输出双路并行：

- **ZMQ 路**：`HandAction` 消息（关节名 + 角度 rad），供可视化/录制消费，地址 `tcp://*:6668`
- **UDP 路**（Wave 模式）：HA4 二进制协议包，直接驱动硬件

---

## 核心优化器

### 求解框架

`hand_retargeting_optimizer.so` 是编译好的 Cython 扩展，内部使用：

- **CasADi**：符号计算框架，构建正运动学和目标函数计算图
- **IPOPT**：内点法非线性规划求解器
- **hand_kinematic_casadi.so**：基于 HA4 URDF 预生成的 CasADi 符号正运动学函数

### NLP 问题形式

```
minimize    Σ wᵢ · lossᵢ(q, keypoints)
subject to  q_lower ≤ q ≤ q_upper        (关节限位约束)
variable    q ∈ ℝ²²
```

### 目标函数各项 Loss

| Loss 项                | 作用                           |
| ---------------------- | ------------------------------ |
| `tip_pos_loss`       | 指尖位置与手套关键点的距离误差 |
| `tip_ori_loss`       | 指尖朝向与手套姿态的角度误差   |
| `finger_ori_loss`    | 手指整体方向误差               |
| `pinch_loss`         | 拇指与各指捏合距离误差         |
| `pinch_ori_loss`     | 捏合时的朝向误差               |
| `dip_ori_loss`       | DIP 关节朝向误差               |
| `pip_pinch_loss`     | PIP 关节捏合误差               |
| `fist_loss`          | 握拳动作整体误差               |
| `pip_gap_loss`       | 相邻手指 PIP 关节间距约束      |
| `dq_loss`            | 相邻帧关节角变化量（速度平滑） |
| `thumb_ddq_loss`     | 拇指关节角加速度平滑           |
| `thumb_mcp_loss`     | 拇指 MCP 关节专项误差          |
| `lincon_margin_loss` | 线性约束裕量惩罚               |

### 帧间滤波

优化器内部对输出关节角做一阶指数滑动平均（EMA）：

```
q_filtered = (1 - α) · q_filtered_prev + α · q_optimized
```

参数 `filter_alpha` 默认 1.0（不滤波），可通过命令行调节。

---

## 平滑发送模块（JointSmoother）

JointSmoother 将优化器输出频率（受 IPOPT 求解时间限制）与硬件发送频率（固定高频）解耦。

### 二阶弹簧-阻尼系统

```
kp = w²              # 刚度（响应速度）
kd = 2 · z · w       # 阻尼

smooth_target = 0.4 · smooth_target_prev + 0.6 · target   # 输入端平滑
error = smooth_target - q
accel = kp · error - kd · dq
dq   += accel · dt
q    += dq · dt
```

默认参数：`hz=250, w=100, z=0.2`（欠阻尼，响应快）。

---

## HA4 关节定义（22个）

| 编号 | 关节名        | 说明                   |
| ---- | ------------- | ---------------------- |
| 0    | thumb_CMC_FE  | 拇指腕掌关节屈伸       |
| 1    | thumb_CMC_AA  | 拇指腕掌关节内收外展   |
| 2    | thumb_MCP_FE  | 拇指掌指关节屈伸       |
| 3    | thumb_MCP_AA  | 拇指掌指关节内收外展   |
| 4    | thumb_IP      | 拇指指间关节           |
| 5    | index_MCP_FE  | 食指掌指关节屈伸       |
| 6    | index_MCP_AA  | 食指掌指关节内收外展   |
| 7    | index_PIP     | 食指近端指间关节       |
| 8    | index_DIP     | 食指远端指间关节       |
| 9    | middle_MCP_FE | 中指掌指关节屈伸       |
| 10   | middle_MCP_AA | 中指掌指关节内收外展   |
| 11   | middle_PIP    | 中指近端指间关节       |
| 12   | middle_DIP    | 中指远端指间关节       |
| 13   | ring_MCP_FE   | 无名指掌指关节屈伸     |
| 14   | ring_MCP_AA   | 无名指掌指关节内收外展 |
| 15   | ring_PIP      | 无名指近端指间关节     |
| 16   | ring_DIP      | 无名指远端指间关节     |
| 17   | pinky_CMC     | 小指腕掌关节           |
| 18   | pinky_MCP_FE  | 小指掌指关节屈伸       |
| 19   | pinky_MCP_AA  | 小指掌指关节内收外展   |
| 20   | pinky_PIP     | 小指近端指间关节       |
| 21   | pinky_DIP     | 小指远端指间关节       |

---

## HA4 UDP 通信协议

数据包结构：`HA4Header + HA4Payload + HA4Tail`

| 字段      | 说明                                           |
| --------- | ---------------------------------------------- |
| 魔数      | `0xBB 0xEE`                                  |
| 设备类型  | `0x01`                                       |
| type_flag | `0x8001`（右手）/ `0x0001`（左手）         |
| 关节数据  | 22 ×`(angle_rad, velocity, torque)` float32 |
| 手部姿态  | 角速度(3) + 四元数(4) float32                  |
| 校验      | CRC32，覆盖除最后 4 字节外的全部数据           |

发送方式：UDP 广播到 `/24` 子网（如 `192.168.x.255`），右手端口 `50030`，左手端口 `50020`。心跳包由独立线程维持设备连接。

---

## 性能优化设计

### 1. 多进程 + 队列隔离（延迟解耦）

ZMQ 订阅运行在独立子进程中，与主进程通过 `mp.Queue(maxsize=10)` 通信。队列满时主动丢弃最旧帧而非阻塞，确保主循环始终处理最新数据：

```python
try:
    mocap_queue.put_nowait(msg)
except queue.Full:
    mocap_queue.get_nowait()  # 丢弃最旧帧，重试写入
```

左右手优化器也各自运行在独立进程中并行求解，互不阻塞。

### 2. ZMQ 高水位标记（RCVHWM=1）

ZMQ socket 设置 `RCVHWM=1`，内核缓冲区最多保留 1 条消息，从源头防止数据积压：

```python
socket.setsockopt(zmq.RCVHWM, 1)
socket.recv(flags=zmq.NOBLOCK)  # 非阻塞轮询，无消息立即返回
```

### 3. 优化频率与发送频率解耦

IPOPT 求解时间不固定（数毫秒到数十毫秒），直接发送会导致硬件收到不均匀的指令流。JointSmoother 在独立线程中以固定 250Hz 运行二阶动力学插值，将优化器的离散输出转为平滑连续轨迹：

```
优化器 (变频)  →  JointSmoother (固定 250Hz)  →  硬件
```

### 4. 首帧对齐（防启动抖动）

JointSmoother 首次收到目标时直接 snap 到目标位置，避免从零位弹射：

```python
if np.all(self.current_angles == 0):
    self.current_angles = self.target_angles.copy()
    self.current_velocity[:] = 0
```

### 5. 超时自停（防数据中断时漂移）

超过 0.5s 未收到新目标时，JointSmoother 清零速度并挂起发送线程，防止惯性积累导致关节漂移：

```python
if time.time() - self.last_update_time > self.data_timeout:  # 0.5s
    self.data_event.clear()
    self.current_velocity[:] = 0
    self.smooth_target = None
```

### 6. Event 驱动的线程唤醒

发送线程使用 `threading.Event` 等待而非忙等，主线程有新数据时才唤醒，避免空转消耗 CPU：

```python
self.data_event.set()          # 主线程：有新数据时触发
self.data_event.wait(timeout=0.1)  # 发送线程：等待唤醒
```

### 7. 严格频率控制

发送循环扣除实际执行时间后再 sleep，维持稳定的发送频率：

```python
elapsed = time.time() - loop_start
time.sleep(max(0, dt - elapsed))
```

### 8. 安全关闭顺序

停止时先停 Smoother 线程，再关 WaveController socket，避免线程向已关闭的 socket 发送数据：

```python
self.left_smoother.stop()        # 先停线程
self.right_smoother.stop()
self.left_wave_controller.stop() # 再关 socket
self.right_wave_controller.stop()
```

---

## 可移植性说明

`hand_retargeting_optimizer.so` 与 `hand_kinematic_casadi.so` 均为针对 **HA4 构型硬编码**的二进制库，不可直接用于其他灵巧手。若需适配其他手：

1. 从目标手的 URDF 重建 CasADi 符号正运动学
2. 按相同 loss 结构重新构建 NLP 问题
3. 对齐 Manus 25 个关键点与目标手运动学链的语义映射
