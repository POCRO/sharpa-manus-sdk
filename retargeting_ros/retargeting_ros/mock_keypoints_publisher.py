"""
Mock Manus keypoints publisher for testing retargeting pipeline without real gloves.
Generates synthetic hand keypoints and publishes via ZMQ (same interface as real Manus client).

Keypoint layout (25 points, matches Manus SDK output):
  0: wrist
  1-4:   thumb  (CMC, MCP, IP, TIP)
  5-8:   index  (MCP, PIP, DIP, TIP)
  9-12:  middle (MCP, PIP, DIP, TIP)
  13-16: ring   (MCP, PIP, DIP, TIP)
  17-20: pinky  (MCP, PIP, DIP, TIP)
  21-24: metacarpal bases (index, middle, ring, pinky)
"""

import sys
import os
import time
import math

# Must be set before importing protobuf — sharpa_hand_pb2.py was generated with old protoc
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

import numpy as np

sys.path.insert(0, os.path.expanduser(
    '~/leo_ws/src/sharpa-manus-sdk/retargeting/include/proto_hand'))

import rclpy
from rclpy.node import Node

import zmq
import sharpa_hand_pb2


# ---------------------------------------------------------------------------
# Minimal hand skeleton in rest pose (right hand, metre units)
# Origin = wrist. Finger chains grow along +X, spread along +Y.
# ---------------------------------------------------------------------------
REST_KEYPOINTS = np.array([
    # wrist (palm root)
    [0.0000,  0.0000, 0.0000],
    # thumb: CMC, MCP, IP, TIP  — from URDF cumulative positions
    [0.0100,  0.0260, 0.0212],   # CMC  (right_thumb_CMC_VL)
    [0.0750,  0.0150, 0.0316],   # MCP  (right_thumb_MCP_VL)
    [0.1140,  0.0150, 0.0316],   # IP   (right_thumb_DP)
    [0.1140, -0.0026, 0.0366],   # TIP  (right_thumb_fingertip)
    # index: MCP, PIP, DIP, TIP
    [0.0010,  0.0303, 0.0957],   # MCP  (right_index_MCP_VL)
    [0.0480,  0.0303, 0.0957],   # PIP  (right_index_MP)
    [0.0795,  0.0303, 0.0957],   # DIP  (right_index_DP)
    [0.0795,  0.0043, 0.0957],   # TIP  (right_index_fingertip)
    # middle: MCP, PIP, DIP, TIP
    [0.0000,  0.0100, 0.0987],   # MCP  (right_middle_MCP_VL)
    [0.0470,  0.0100, 0.0987],   # PIP  (right_middle_MP)
    [0.0785,  0.0100, 0.0987],   # DIP  (right_middle_DP)
    [0.0785, -0.0160, 0.0987],   # TIP  (right_middle_fingertip)
    # ring: MCP, PIP, DIP, TIP
    [0.0015, -0.0103, 0.0927],   # MCP  (right_ring_MCP_VL)
    [0.0485, -0.0103, 0.0927],   # PIP  (right_ring_MP)
    [0.0800, -0.0103, 0.0927],   # DIP  (right_ring_DP)
    [0.0800, -0.0363, 0.0927],   # TIP  (right_ring_fingertip)
    # pinky: MCP, PIP, DIP, TIP
    [0.0197, -0.0215, 0.0867],   # MCP  (right_pinky_MCP_VL)
    [0.0667, -0.0215, 0.0867],   # PIP  (right_pinky_MP)
    [0.0982, -0.0215, 0.0867],   # DIP  (right_pinky_DP)
    [0.0982, -0.0475, 0.0867],   # TIP  (right_pinky_fingertip)
    # metacarpal bases (index, middle, ring, pinky)
    [0.0010,  0.0303, 0.0480],
    [0.0000,  0.0100, 0.0494],
    [0.0015, -0.0103, 0.0464],
    [0.0114, -0.0263, 0.0434],
], dtype=np.float64)  # shape (25, 3), from URDF right_sharpa_ha4 cumulative positions

# Finger tip indices (used for curl animation)
FINGER_TIP_INDICES = [4, 8, 12, 16, 20]
# Corresponding MCP indices
FINGER_MCP_INDICES = [2, 5, 9, 13, 17]


def curl_keypoints(rest: np.ndarray, curl: float) -> np.ndarray:
    """
    Apply a simple curl deformation to all fingers.
    curl: 0.0 = open hand, 1.0 = fist
    Returns (25, 3) array.

    Finger layout (indices):
      thumb:  CMC=1, MCP=2, IP=3,  TIP=4
      index:  MCP=5, PIP=6, DIP=7, TIP=8
      middle: MCP=9, PIP=10,DIP=11,TIP=12
      ring:   MCP=13,PIP=14,DIP=15,TIP=16
      pinky:  MCP=17,PIP=18,DIP=19,TIP=20
    """
    kp = rest.copy()
    # (pivot_idx, [joints to rotate around pivot])
    finger_defs = [
        (2, [3, 4]),       # thumb:  pivot=MCP(2), rotate IP+TIP
        (5, [6, 7, 8]),    # index:  pivot=MCP(5), rotate PIP+DIP+TIP
        (9, [10, 11, 12]), # middle: pivot=MCP(9)
        (13, [14, 15, 16]),# ring:   pivot=MCP(13)
        (17, [18, 19, 20]),# pinky:  pivot=MCP(17)
    ]

    angle = curl * math.pi * 0.7  # max ~126 deg curl
    c, s = math.cos(angle), math.sin(angle)

    for pivot_idx, chain in finger_defs:
        pivot = rest[pivot_idx]
        for ci in chain:
            rel = rest[ci] - pivot
            # rotate around Y axis (curl fingers toward -Z)
            x_new = rel[0] * c + rel[2] * s
            z_new = -rel[0] * s + rel[2] * c
            kp[ci] = pivot + np.array([x_new, rel[1], z_new])
    return kp


def make_pose(pos: np.ndarray):
    return {'x': float(pos[0]), 'y': float(pos[1]), 'z': float(pos[2]),
            'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0}


class MockKeypointsPublisher(Node):
    def __init__(self):
        super().__init__('mock_keypoints_publisher')

        self.declare_parameter('zmq_address', 'tcp://*:2044')
        self.declare_parameter('frequency', 50.0)   # Hz
        self.declare_parameter('motion', 'wave')    # 'wave' | 'fist' | 'static'

        address = self.get_parameter('zmq_address').get_parameter_value().string_value
        freq    = self.get_parameter('frequency').get_parameter_value().double_value
        self.motion = self.get_parameter('motion').get_parameter_value().string_value

        ctx = zmq.Context()
        self.sock = ctx.socket(zmq.PUB)
        self.sock.setsockopt(zmq.SNDHWM, 1)
        self.sock.bind(address)
        time.sleep(0.2)  # allow subscribers to connect

        self.t0 = time.time()
        self.create_timer(1.0 / freq, self.publish)
        self.get_logger().info(
            f'MockKeypointsPublisher started: address={address}, '
            f'freq={freq}Hz, motion={self.motion}')

    def publish(self):
        t = time.time() - self.t0

        if self.motion == 'wave':
            # Sinusoidal open/close cycle at 0.5 Hz
            curl = 0.5 * (1.0 - math.cos(2 * math.pi * 0.5 * t))
        elif self.motion == 'fist':
            curl = 1.0
        else:  # static open
            curl = 0.0

        kp = curl_keypoints(REST_KEYPOINTS, curl)
        poses = [make_pose(kp[i]) for i in range(25)]

        msg = sharpa_hand_pb2.MocapKeypoints()
        msg.header.stamp.sec = int(time.time())
        msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
        msg.header.frame_id = 'mock'

        # REST_KEYPOINTS matches left hand URDF (Y negative for index side)
        # Right hand is mirrored (flip Y)
        for p in poses:
            pose = msg.left_mocap_pose.add()
            pose.position.x = p['x']
            pose.position.y = p['y']
            pose.position.z = p['z']
            pose.orientation.w = p['qw']
            pose.orientation.x = p['qx']
            pose.orientation.y = p['qy']
            pose.orientation.z = p['qz']

        # Mirror for right hand (flip Y)
        for p in poses:
            pose = msg.right_mocap_pose.add()
            pose.position.x = p['x']
            pose.position.y = -p['y']
            pose.position.z = p['z']
            pose.orientation.w = p['qw']
            pose.orientation.x = p['qx']
            pose.orientation.y = -p['qy']
            pose.orientation.z = p['qz']

        try:
            self.sock.send(msg.SerializeToString(), flags=zmq.NOBLOCK)
        except zmq.Again:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MockKeypointsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
