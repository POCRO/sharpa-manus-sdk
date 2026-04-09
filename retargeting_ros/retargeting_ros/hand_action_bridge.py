"""
HandAction → JointState bridge.
Subscribes to the retargeting pipeline output (ZMQ, Protobuf HandAction)
and republishes as sensor_msgs/JointState for robot_state_publisher / RViz.

Also logs end-to-end latency using the timestamp embedded in HandAction.header.
"""

import sys
import os
import time

# Must be set before importing protobuf — sharpa_hand_pb2.py was generated with old protoc
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

sys.path.insert(0, os.path.expanduser(
    '~/leo_ws/src/sharpa-manus-sdk/retargeting/include/proto_hand'))

import zmq
import sharpa_hand_pb2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HandActionBridge(Node):
    def __init__(self):
        super().__init__('hand_action_bridge')

        self.declare_parameter('zmq_address', 'tcp://localhost:6668')
        self.declare_parameter('frequency', 250.0)   # poll Hz
        self.declare_parameter('hand', 'right')      # 'right' | 'left' | 'both'
        self.declare_parameter('log_latency', True)

        address     = self.get_parameter('zmq_address').get_parameter_value().string_value
        freq        = self.get_parameter('frequency').get_parameter_value().double_value
        self.hand   = self.get_parameter('hand').get_parameter_value().string_value
        self.log_latency = self.get_parameter('log_latency').get_parameter_value().bool_value

        ctx = zmq.Context()
        self.sock = ctx.socket(zmq.SUB)
        self.sock.setsockopt(zmq.RCVHWM, 1)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.setsockopt(zmq.RCVTIMEO, 0)   # non-blocking
        self.sock.connect(address)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, '')

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(1.0 / freq, self.poll)

        # Latency stats
        self._latency_samples = []
        self._stat_timer = self.create_timer(5.0, self._log_latency_stats)

        self.get_logger().info(
            f'HandActionBridge started: zmq={address}, freq={freq}Hz, hand={self.hand}')

    def poll(self):
        try:
            raw = self.sock.recv(flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        msg = sharpa_hand_pb2.HandAction()
        msg.ParseFromString(raw)

        now = time.time()
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp_sec > 0 and self.log_latency:
            latency_ms = (now - stamp_sec) * 1000.0
            self._latency_samples.append(latency_ms)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        if self.hand in ('right', 'both') and len(msg.joint_right.name) > 0:
            js.name.extend(list(msg.joint_right.name))
            js.position.extend(list(msg.joint_right.position))

        if self.hand in ('left', 'both') and len(msg.joint_left.name) > 0:
            js.name.extend(list(msg.joint_left.name))
            js.position.extend(list(msg.joint_left.position))

        if js.name:
            self.pub.publish(js)

    def _log_latency_stats(self):
        if not self._latency_samples:
            self.get_logger().info('Latency: no data yet')
            return
        import statistics
        s = self._latency_samples
        self.get_logger().info(
            f'Latency (last {len(s)} frames): '
            f'mean={statistics.mean(s):.1f}ms  '
            f'median={statistics.median(s):.1f}ms  '
            f'max={max(s):.1f}ms'
        )
        self._latency_samples.clear()


def main(args=None):
    rclpy.init(args=args)
    node = HandActionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
