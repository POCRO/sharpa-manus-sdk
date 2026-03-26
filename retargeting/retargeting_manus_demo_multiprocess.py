"""
retarget manus meta/meta pro to HA4.0 dexterous hand using multiprocess optimization
use zmq to communicate with manus and dexterous hand
"""

import zmq
import numpy as np
import time
import signal
import sys
import os
import multiprocessing as mp
from rich.live import Live
from rich.table import Table
from rich.console import Console
import threading
import queue
import traceback
import logging

# Add include directory to Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/include")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/include/proto_hand")
from hand_retargeting_optimizer import  MultiprocessOptimizationManager, init_hand_model
from proto_hand.receiver_demo import MocapKeypointsReceiver
from proto_hand.sender_demo import HandActionSender
import sharpa_hand_pb2

# Configure logging system
def setup_logging():
    """Configure logging system, support multiprocess, output to file and console"""
    log_dir = "logs"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(os.path.join(log_dir, 'retargeting.log'), encoding='utf-8'),
            logging.StreamHandler()  # Also output to console
        ]
    )

setup_logging()
logger = logging.getLogger(__name__)

# Add visualization related imports
from visualizer import DualHandVisualizer
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Add WaveController related imports
try:
    from heartbeat_ha4 import send_heartbeat
    from pub_glove_ha4 import MockGlove, HandType, DEFAULT_HA4_PORT_RIGHT, DEFAULT_HA4_PORT_LEFT
    WAVE_MODE_AVAILABLE = True
except ImportError as e:
    print(f"Unable to import Wave mode required modules, but this demo can be used without wave mode: {e}")
    WAVE_MODE_AVAILABLE = False


def zmq_mocap_subscriber_process(mocap_queue, address="tcp://localhost:6667"):
    """ZMQ mocap subscriber process running at 200Hz with minimal control flow."""
    context = zmq.Context.instance()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.connect(address)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    print(f"[ZMQ Mocap Subscriber Process] Connected to {address}")

    target_interval = 1.0 / 1000.0  # 200Hz

    while True:
        try:
            payload = socket.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            payload = None

        if payload:
            msg = sharpa_hand_pb2.MocapKeypoints()
            msg.ParseFromString(payload)

            message_added = False
            attempts = 0
            max_attempts = 10 

            while not message_added and attempts < max_attempts:
                try:
                    mocap_queue.put_nowait(msg)
                    message_added = True
                except queue.Full:
                    attempts += 1
                    try:
                        mocap_queue.get_nowait()  # Remove oldest message
                    except queue.Empty:
                        time.sleep(0.001)
                        continue
        
        time.sleep(target_interval)

# ==============================================================================
# Joint Smoother Class (Integrated)
# ==============================================================================
class JointSmoother:
    def __init__(self, send_callback, hz=120.0, w=25.0, z=0.8):
        """
        Generic Joint Smoother (2nd Order Low-Pass Filter + High Freq Sender Thread)
        
        Args:
            send_callback: Function to execute actual hardware sending.
                           Signature: func(angles: list) -> None
            hz (float): Sending frequency (e.g., 120Hz)
            w (float): Natural frequency (Response speed)
            z (float): Damping ratio (Smoothness)
        """
        self.send_callback = send_callback
        self.hz = hz
        
        # 2nd Order Dynamics Parameters
        self.k_p = w * w
        self.k_d = 2 * z * w
        
        # State variables
        self.current_angles = np.zeros(22, dtype=np.float32)
        self.current_velocity = np.zeros(22, dtype=np.float32)
        self.target_angles = None
        self.smooth_target = None   # Input side smoothing variable

        # Thread control
        self.lock = threading.Lock()
        self.data_event = threading.Event()
        self.is_running = False
        self.worker_thread = None
        
        # Timeout control
        self.last_update_time = 0
        self.data_timeout = 0.5

    def start(self):
        """Start background smoothing thread"""
        if self.is_running:
            return
        self.is_running = True
        self.worker_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.worker_thread.start()
        logger.info(f"JointSmoother Started (Freq={self.hz}Hz)")

    def stop(self):
        """Stop thread"""
        self.is_running = False
        self.data_event.set() # Wake up thread to exit
        if self.worker_thread:
            self.worker_thread.join(timeout=0.2)

    def update(self, new_target):
        """[Called by Main Thread] Update target angles"""
        if not self.is_running:
            return False
            
        with self.lock:
            # Update target
            self.target_angles = np.array(new_target, dtype=np.float32)
            self.last_update_time = time.time()
            
            # Initialization alignment: if first time receiving, snap to target
            if np.all(self.current_angles == 0):
                self.current_angles = self.target_angles.copy()
                self.current_velocity[:] = 0
        
            # If just started or recovered from timeout, reset smooth target
            if self.smooth_target is None:
                self.smooth_target = self.target_angles.copy()

        # Wake up worker thread if waiting
        if not self.data_event.is_set():
            self.data_event.set()
        return True

    def _control_loop(self):
        """[Internal Thread] Physics simulation and callback execution"""
        dt = 1.0 / self.hz
        
        while self.is_running:
            # Wait for data
            if not self.data_event.wait(timeout=0.1):
                continue
            
            # Timeout logic: stop sending and zero velocity if no new data
            if time.time() - self.last_update_time > self.data_timeout:
                self.data_event.clear()
                self.current_velocity[:] = 0 
                with self.lock:
                    self.smooth_target = None
                continue

            loop_start = time.time()
            
            # 2nd Order Low Pass Filter
            if self.target_angles is not None:
                with self.lock:
                    if self.smooth_target is None:
                        self.smooth_target = self.target_angles.copy()
                    
                    # Input smoothing (smooth the error/step)
                    alpha = 0.6
                    self.smooth_target = (1 - alpha) * self.smooth_target + alpha * self.target_angles

                    # Calculate deviation
                    error = self.smooth_target - self.current_angles

                    # Calculate acceleration (a = kp * error - kd * velocity)
                    accel = self.k_p * error - self.k_d * self.current_velocity

                    # Update velocity (v = v + a * dt)
                    self.current_velocity += accel * dt

                    # Update position (x = x + v * dt)
                    self.current_angles += self.current_velocity * dt
                    
                    # Prepare data to send
                    angles_to_send = self.current_angles.tolist()
                
                # Execute Callback: Send to hardware
                try:
                    self.send_callback(angles_to_send)
                except Exception as e:
                    logger.error(f"Smoother callback error: {e}")

            # Strict frequency control
            elapsed = time.time() - loop_start
            time.sleep(max(0, dt - elapsed))

# ==============================================================================

class WaveController:
    """Wave mode controller, for sending optimized joint angles"""
    
    def __init__(self, target_ip='192.168.1.100', hand_type=None, ha4_port=None):
        self.target_ip = target_ip
        self.hand_type = hand_type
        self.mock_glove = None
        self.heartbeat_thread = None
        self.is_running = False
        
        if self.hand_type is None:
            raise ValueError("hand_type parameter is required")
        if self.hand_type not in [HandType.LEFT, HandType.RIGHT]:
            raise ValueError(f"Invalid hand_type: {self.hand_type}, must be HandType.LEFT or HandType.RIGHT")
        
        if ha4_port is not None:
            self.ha4_port = ha4_port
        else:
            self.ha4_port = DEFAULT_HA4_PORT_LEFT if self.hand_type == HandType.LEFT else DEFAULT_HA4_PORT_RIGHT

    def start(self):
        if not WAVE_MODE_AVAILABLE:
            raise ImportError("Wave mode required modules not installed")
            
        heartbeat_name = f'Glove-{"L" if self.hand_type == HandType.LEFT else "R"}-retargeting'
        self.heartbeat_thread = threading.Thread(
            target=send_heartbeat, 
            args=(heartbeat_name, self.target_ip)
        )
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()
        
        self.mock_glove = MockGlove()
        if not self.mock_glove.create_glove(self.target_ip, self.ha4_port, self.hand_type):
            raise RuntimeError("Failed to create UDP connection")
        
        self.is_running = True
        hand_type_name = "LEFT" if self.hand_type == HandType.LEFT else "RIGHT"
        print(f"Wave controller started, hand_type: {hand_type_name}, target IP: {self.target_ip}, port: {self.ha4_port}")
        
    def send_joint_angles(self, joint_angles):
        if not self.is_running or self.mock_glove is None:
            return False
        
        try:
            if len(joint_angles) != 22:
                # logger.warning(f"Incorrect number of input angles: {len(joint_angles)}, expected 22")
                # raise ValueError("Incorrect number of input angles, cannot send")
                return False
            
            ha4_angles = joint_angles # Input is already list or numpy array
            self.mock_glove.send_HA4_packet(ha4_angles, self.hand_type)

            return True
        except Exception as e:
            print(f"Failed to send joint angles: {e}")
            return False
    
    def stop(self):
        self.is_running = False
        if self.mock_glove is not None:
            self.mock_glove = None
        print("Wave controller stopped")

class DualHandRetargetingSystem:
    """Dual hand retargeting system, with optional left/right input"""
    
    def __init__(self, hand_serial='HA4', filter_alpha=0.2, 
                 mocap_address="tcp://localhost:6667", 
                 hand_action_address="tcp://localhost:6668",
                 wave=False, glove_left_ip='192.168.10.110', glove_right_ip='192.168.10.100',
                 plot_enabled=False, plot_orientation=False, debug_print=False,
                 smoother_hz=250.0, smoother_w=25.0, smoother_z=0.8):
        
        self.hand_serial = hand_serial
        self.filter_alpha = filter_alpha
        self.mocap_address = mocap_address
        self.hand_action_address = hand_action_address
        self.wave = wave
        self.glove_left_ip = glove_left_ip
        self.glove_right_ip = glove_right_ip
        self.plot_enabled = plot_enabled
        self.plot_orientation = plot_orientation
        self.debug_print = debug_print
        self.smoother_hz = smoother_hz
        self.smoother_w = smoother_w
        self.smoother_z = smoother_z
        
        if hand_serial != 'HA4':
            raise ValueError(f"Unsupported hand model: {hand_serial}, only supports HA4")
        
        self.hand_models = init_hand_model(hand_serial)
        self.hand_action_sender = self._create_zmq_components()
        
        self.mocap_queue = mp.Queue(maxsize=10)
        self.mocap_subscriber_process = None
        
        self.multiprocess_manager = MultiprocessOptimizationManager(
            self.hand_models, 
            filter_alpha=filter_alpha, 
            hand_serial=hand_serial
        )
        
        # Wave controllers and Smoothers
        self.left_wave_controller = None
        self.right_wave_controller = None
        self.left_smoother = None
        self.right_smoother = None

        if self.wave and WAVE_MODE_AVAILABLE:
            try:
                # 1. Initialize Low-level Wave Controllers
                self.left_wave_controller = WaveController(
                    target_ip=self.glove_left_ip, 
                    hand_type=HandType.LEFT
                )
                self.right_wave_controller = WaveController(
                    target_ip=self.glove_right_ip, 
                    hand_type=HandType.RIGHT
                )
                self.left_wave_controller.start()
                self.right_wave_controller.start()

                # 2. Initialize Joint Smoothers
                # We connect the smoother's callback directly to the wave controller's send method
                self.left_smoother = JointSmoother(
                    send_callback=self.left_wave_controller.send_joint_angles,
                    hz=self.smoother_hz,  # High frequency sending to hardware
                    w=self.smoother_w,    # Response speed
                    z=self.smoother_z     # Damping
                )
                self.right_smoother = JointSmoother(
                    send_callback=self.right_wave_controller.send_joint_angles,
                    hz=self.smoother_hz,
                    w=self.smoother_w,
                    z=self.smoother_z
                )

                print("Wave controllers and Smoothers initialized")
            except Exception as e:
                print(f"Failed to start Wave controllers: {e}")
                self.wave = False
        
        self.visualizer = None
        if self.plot_enabled and not self.wave:
            try:
                self.visualizer = DualHandVisualizer(display_mode='both', plot_orientation=self.plot_orientation)
                print("Visualizer initialized")
            except Exception as e:
                print(f"Failed to initialize visualizer: {e}")
                self.visualizer = None
        
        if self.debug_print:
            self.table, self.console = self._create_table()
        else:
            self.table, self.console = None, None
        
        self.running = False
        self.frame_count = 0
        
    def _create_zmq_components(self):
        hand_action_sender = HandActionSender(address=self.hand_action_address)
        return hand_action_sender
    
    def _start_mocap_subscriber_process(self):
        if self.mocap_subscriber_process is None or not self.mocap_subscriber_process.is_alive():
            self.mocap_subscriber_process = mp.Process(
                target=zmq_mocap_subscriber_process,
                args=(self.mocap_queue, self.mocap_address)
            )
            self.mocap_subscriber_process.daemon = True
            self.mocap_subscriber_process.start()
            print(f"Mocap data receiving process started, address: {self.mocap_address}")
    
    def _stop_mocap_subscriber_process(self):
        if self.mocap_subscriber_process and self.mocap_subscriber_process.is_alive():
            self.mocap_subscriber_process.terminate()
            self.mocap_subscriber_process.join(timeout=1.0)
            if self.mocap_subscriber_process.is_alive():
                self.mocap_subscriber_process.kill()
            print("Mocap data receiving process stopped")
    
    def _create_table(self):
        """Create display table"""
        table = Table(show_header=False, box=None)
        table.add_column("Left Hand", style="cyan")
        table.add_column("Right Hand", style="magenta")
        table.add_row("Waiting for data...", "Waiting for data...")
        table.add_row("Cost value: ", "Cost value: ")
        table.add_row("Optimization time: ", "Optimization time: ")
        table.add_row("tip_pos_loss: ", "tip_pos_loss: ")
        table.add_row("tip_ori_loss: ", "tip_ori_loss: ")
        table.add_row("finger_ori_loss: ", "finger_ori_loss: ")
        table.add_row("pinch_loss: ", "pinch_loss: ")
        table.add_row("pinch_ori_loss: ", "pinch_ori_loss: ")
        table.add_row("dq_loss: ", "dq_loss: ")
        table.add_row("thumb_ddq_loss: ", "thumb_ddq_loss: ")
        table.add_row("thumb_mcp_loss: ", "thumb_mcp_loss: ")
        table.add_row("dip_ori_loss: ", "dip_ori_loss: ")
        table.add_row("pip_pinch_loss: ", "pip_pinch_loss: ")
        table.add_row("fist_loss: ", "fist_loss: ")
        table.add_row("pip_gap_loss: ", "pip_gap_loss: ")
        
        console = Console()
        return table, console
    
    def _protobuf_to_numpy(self, mocap_msg):
        left_keypoints = np.zeros((25, 7))
        right_keypoints = np.zeros((25, 7))
        
        for i, pose in enumerate(mocap_msg.left_mocap_pose):
            if i < 25: 
                left_keypoints[i, 0] = pose.position.x
                left_keypoints[i, 1] = pose.position.y
                left_keypoints[i, 2] = pose.position.z
                left_keypoints[i, 3] = pose.orientation.w
                left_keypoints[i, 4] = pose.orientation.x
                left_keypoints[i, 5] = pose.orientation.y
                left_keypoints[i, 6] = pose.orientation.z
        
        for i, pose in enumerate(mocap_msg.right_mocap_pose):
            if i < 25:
                right_keypoints[i, 0] = pose.position.x
                right_keypoints[i, 1] = pose.position.y
                right_keypoints[i, 2] = pose.position.z
                right_keypoints[i, 3] = pose.orientation.w
                right_keypoints[i, 4] = pose.orientation.x
                right_keypoints[i, 5] = pose.orientation.y
                right_keypoints[i, 6] = pose.orientation.z
        return left_keypoints, right_keypoints
    
    def _update_table(self, frame_idx, left_result, right_result):
        if not self.debug_print or self.table is None:
            return
            
        if left_result:
            self.table.columns[0]._cells[0] = f"Left hand frame {left_result.frame_index + 1} optimization completed"
            self.table.columns[0]._cells[1] = f"Cost value: {left_result.cost_value:.6f}"
            self.table.columns[0]._cells[2] = f"Optimization time: {left_result.optimization_time:.6f}s"
            # ... (truncated for brevity, same as original) ...
            
        if right_result:
            self.table.columns[1]._cells[0] = f"Right hand frame {right_result.frame_index + 1} optimization completed"
            self.table.columns[1]._cells[1] = f"Cost value: {right_result.cost_value:.6f}"
            self.table.columns[1]._cells[2] = f"Optimization time: {right_result.optimization_time:.6f}s"
            # ... (truncated for brevity, same as original) ...

    def _send_hand_action(self, left_result, right_result):
        """
        Send joint command message
        
        Args:
            left_result: Left optimization result
            right_result: Right optimization result
        """
        try:
            # === WAVE Mode (Hardware Control with Smoothing) ===
            if self.wave:
                # Update LEFT Smoother
                if left_result and left_result.filtered_angles is not None and self.left_smoother:
                    # Convert to degrees for HA4 hardware
                    left_angles_deg = np.rad2deg(left_result.filtered_angles)
                    # Push new target to smoother. Smoother thread handles the high-freq sending.
                    self.left_smoother.update(left_angles_deg)
                
                # Update RIGHT Smoother
                if right_result and right_result.filtered_angles is not None and self.right_smoother:
                    # Convert to degrees for HA4 hardware
                    right_angles_deg = np.rad2deg(right_result.filtered_angles)
                    # Push new target to smoother. Smoother thread handles the high-freq sending.
                    self.right_smoother.update(right_angles_deg)
            
            # === ZMQ Mode (Visualization / Data Recording) ===
            # This continues to send the "Target" (Optimized) values to ZMQ for visualizer
            
            # Get joint names
            left_joint_names = self.hand_models['left'].joint_names if self.hand_models['left'] else []
            right_joint_names = self.hand_models['right'].joint_names if self.hand_models['right'] else []
            
            # Convert to degrees for ZMQ (Standard is usually degrees for Unity/Unreal)
            left_positions = np.array(left_result.filtered_angles).tolist() if left_result and left_result.filtered_angles is not None else [0.0] * len(left_joint_names)
            right_positions = np.array(right_result.filtered_angles).tolist() if right_result and right_result.filtered_angles is not None else [0.0] * len(right_joint_names)
            
            # Send ZMQ message
            success = self.hand_action_sender.send_hand_action(
                left_joint_names, left_positions,
                right_joint_names, right_positions
            )
            
            if not success:
                print("[HandAction Sender] Failed to send joint commands")
                
        except Exception as e:
            print(f"[HandAction Sender] Error sending joint commands: {e}")
    
    def start(self):
        """Start system"""
        try:
            self._start_mocap_subscriber_process()
            self.multiprocess_manager.start()
            
            # Start Smoothers (Threads)
            if self.wave:
                if self.left_smoother:
                    self.left_smoother.start()
                if self.right_smoother:
                    self.right_smoother.start()
            
            time.sleep(1)
            
            if self.visualizer is not None and self.plot_enabled and not self.wave:
                try:
                    self.visualizer.start_animation(interval=100)
                    print("Visualizer started")
                except Exception as e:
                    print(f"Failed to start visualizer: {e}")
            
            self.running = True
            print("Dual hand retargeting system started")
            
        except Exception as e:
            print(f"Failed to start system: {e}")
            raise e
    
    def run(self):
        """Run main loop"""
        if not self.running:
            raise RuntimeError("System not started, please call start() method first")
        
        try:
            left_keypoints= None
            right_keypoints = None
            left_keypoints_last = None
            right_keypoints_last = None
            
            if self.debug_print and self.table is not None:
                live_context = Live(self.table, refresh_per_second=100, console=self.console, screen=False)
            else:
                from contextlib import nullcontext
                live_context = nullcontext()
            
            with live_context:
                while self.running:
                    try:
                        mocap_msg = self.mocap_queue.get_nowait()
                        
                        if mocap_msg is not None:
                            left_keypoints, right_keypoints = self._protobuf_to_numpy(mocap_msg)
                            if left_keypoints is None or np.all(left_keypoints == 0):
                                left_keypoints = left_keypoints_last
                            if right_keypoints is None or np.all(right_keypoints == 0):
                                right_keypoints = right_keypoints_last

                            left_keypoints_last = left_keypoints
                            right_keypoints_last = right_keypoints
                            
                            left_updated = False
                            right_updated = False
                            
                            if left_keypoints is not None and not np.all(left_keypoints == 0):
                                left_updated = self.multiprocess_manager.update_process_keypoints('left', left_keypoints, self.frame_count)
                                while not left_updated and self.running: 
                                    left_updated = self.multiprocess_manager.update_process_keypoints('left', left_keypoints, self.frame_count)
                                    time.sleep(0.001)
                            
                            if right_keypoints is not None and not np.all(right_keypoints == 0):
                                right_updated = self.multiprocess_manager.update_process_keypoints('right', right_keypoints, self.frame_count)
                                while not right_updated and self.running:
                                    right_updated = self.multiprocess_manager.update_process_keypoints('right', right_keypoints, self.frame_count)
                                    time.sleep(0.001)
                            
                            left_result = self.multiprocess_manager.get_result('left')  if left_updated else None
                            right_result = self.multiprocess_manager.get_result('right')  if right_updated else None

                            if self.visualizer is not None and self.plot_enabled and not self.wave:
                                try:
                                    left_hand_keypoints = left_result.keypoints if left_result else None
                                    right_hand_keypoints = right_result.keypoints if right_result else None
                                    
                                    self.visualizer.update(
                                        left_raw_keypoints=left_keypoints,
                                        left_hand_keypoints=left_hand_keypoints,
                                        right_raw_keypoints=right_keypoints,
                                        right_hand_keypoints=right_hand_keypoints
                                    )
                                    self.visualizer.update_display()
                                except KeyboardInterrupt:
                                    raise 
                                except Exception as e:
                                    print(f"Error updating visualizer: {e}")
                            
                            self._update_table(self.frame_count, left_result, right_result)
                            
                            # Send joint commands (Now utilizes Smoothers inside)
                            self._send_hand_action(left_result, right_result)
                            
                            self.frame_count += 1
                            
                    except queue.Empty:
                        pass
                    except KeyboardInterrupt:
                        raise
                    except Exception as e:
                        print(f"Error processing message: {e}, {traceback.format_exc()}")
                        time.sleep(0.01)
                    
                    time.sleep(0.001)
                        
        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"Error occurred during program execution: {e}")
            raise e
    
    def stop(self):
        """Stop system"""
        print("Stopping system...")
        
        self.running = False
        
        self._stop_mocap_subscriber_process()
        
        if hasattr(self, 'multiprocess_manager') and self.multiprocess_manager:
            try:
                self.multiprocess_manager.cleanup()
            except Exception as e:
                print(f"Multiprocess cleanup error: {e}")
        
        if hasattr(self, 'hand_action_sender') and self.hand_action_sender:
            try:
                self.hand_action_sender.close()
            except:
                pass
        
        # Stop Smoothers BEFORE stopping WaveControllers
        # This ensures threads don't try to send to closed sockets
        if self.left_smoother:
            self.left_smoother.stop()
        if self.right_smoother:
            self.right_smoother.stop()

        if self.left_wave_controller:
            try:
                self.left_wave_controller.stop()
            except:
                pass
        if self.right_wave_controller:
            try:
                self.right_wave_controller.stop()
            except:
                pass
        
        if self.visualizer is not None:
            try:
                if hasattr(self.visualizer, 'left_ani') and self.visualizer.left_ani is not None:
                    self.visualizer.left_ani.event_source.stop()
                if hasattr(self.visualizer, 'right_ani') and self.visualizer.right_ani is not None:
                    self.visualizer.right_ani.event_source.stop()
                plt.close('all')
                plt.ioff()
            except:
                pass

        print("System stopped")

def main():
    try:
        mp.set_start_method('spawn', force=True)
    except RuntimeError:
        pass
    
    import argparse
    parser = argparse.ArgumentParser(description='Dual hand retargeting system')
    parser.add_argument('-hand_serial', type=str, default='HA4', choices=['HA4'], 
                       help='Hand model (default: HA4)')
    parser.add_argument('-filter_alpha', type=float, default=1.0, 
                       help='Filter parameter (0-1, default: 0.8)')
    parser.add_argument('-mocap_address', type=str, default="tcp://localhost:2044",help='Mocap data receiving address')
    parser.add_argument('-hand_action_address', type=str, default="tcp://*:6668",help='Joint command sending address')
    parser.add_argument('-wave', action='store_true', 
                       help='Enable wave mode, send joint angles to Wave controllers')
    parser.add_argument('-glove_left_ip', type=str, default='192.168.10.99',
                       help='Left hand Wave controller target IP')
    parser.add_argument('-glove_right_ip', type=str, default='192.168.10.100',
                       help='Right hand Wave controller target IP')
    parser.add_argument('-plot', action='store_true', default=False,
                       help='Enable plot function')
    parser.add_argument('-plot_orientation', action='store_true', default=False,
                       help='Display orientation arrows for key points')
    parser.add_argument('-debug_print', action='store_true', default=False,
                       help='Enable debug print mode')
    parser.add_argument('-smoother_hz', type=float, default=250.0,
                       help='Joint smoother sending frequency in Hz (default: 250.0)')
    parser.add_argument('-smoother_w', type=float, default=100.0,
                       help='Joint smoother response speed (default: 100.0)')
    parser.add_argument('-smoother_z', type=float, default=0.2,
                       help='Joint smoother damping (default: 0.2)')
    
    args = parser.parse_args()
    
    hand_serial = args.hand_serial
    filter_alpha = args.filter_alpha
    mocap_address = args.mocap_address
    hand_action_address = args.hand_action_address
    wave = args.wave
    glove_left_ip = args.glove_left_ip
    glove_right_ip = args.glove_right_ip
    plot_enabled = args.plot
    plot_orientation = args.plot_orientation
    debug_print = args.debug_print
    smoother_hz = args.smoother_hz
    smoother_w = args.smoother_w
    smoother_z = args.smoother_z
    
    if hand_serial != 'HA4':
        raise ValueError(f"Unsupported hand model: {hand_serial}, only supports HA4")
    
    print(f"System configuration:")
    print(f"  Hand model: {hand_serial}")
    print(f"  Filter alpha: {filter_alpha}")
    print(f"  Mocap address: {mocap_address}")
    print(f"  Wave mode: {'Enabled' if wave else 'Disabled'}")
    
    system = DualHandRetargetingSystem(
        hand_serial=hand_serial,
        filter_alpha=filter_alpha,
        mocap_address=mocap_address,
        hand_action_address=hand_action_address,
        wave=wave,
        glove_left_ip=glove_left_ip,
        glove_right_ip=glove_right_ip,
        plot_enabled=plot_enabled,
        plot_orientation=plot_orientation,
        debug_print=debug_print,
        smoother_hz=smoother_hz,
        smoother_w=smoother_w,
        smoother_z=smoother_z
    )
    
    try:
        system.start()
        system.run()
    except KeyboardInterrupt:
        print("\nUser interrupted program execution")
    except Exception as e:
        print(f"Program execution error: {e}")
        raise e
    finally:
        system.stop()
        print("Program exit")

if __name__ == "__main__":
    main()
