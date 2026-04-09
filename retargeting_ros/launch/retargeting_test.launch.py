"""
Launch file for retargeting pipeline test in RViz (no hardware required).

Starts:
  1. robot_state_publisher  — loads HA4 URDF
  2. rviz2                  — loads sharpa.rviz config
  3. mock_keypoints_publisher — generates synthetic hand keypoints via ZMQ
  4. hand_action_bridge     — converts HandAction (ZMQ) to /joint_states (ROS2)

Usage:
  ros2 launch retargeting_ros retargeting_test.launch.py

Then manually start the retargeting optimizer in a separate terminal:
  cd ~/leo_ws/src/sharpa-manus-sdk/retargeting
  python retargeting_manus_demo_multiprocess.py -mocap_address tcp://localhost:2044
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_HA4_DIR = os.path.expanduser(
    '~/leo_ws/src/sharpa-manus-sdk/retargeting/urdf/right_sharpa_ha4')


def generate_launch_description():
    urdf_file = os.path.join(_HA4_DIR, 'right_sharpa_ha4.urdf')
    rviz_file = os.path.join(_HA4_DIR, 'config', 'sharpa.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Fix protobuf version mismatch: sharpa_hand_pb2.py was generated with old protoc
    # Using pure-Python implementation avoids the "Descriptors cannot be created directly" error
    pb_env = SetEnvironmentVariable(
        'PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')

    motion_arg = DeclareLaunchArgument(
        'motion', default_value='wave',
        description='Mock motion mode: wave | fist | static')

    zmq_mocap_arg = DeclareLaunchArgument(
        'zmq_mocap', default_value='tcp://*:2044',
        description='ZMQ bind address for mock keypoints publisher')

    zmq_action_arg = DeclareLaunchArgument(
        'zmq_action', default_value='tcp://localhost:6668',
        description='ZMQ address to receive HandAction from retargeting optimizer')

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
    )

    mock_pub_node = Node(
        package='retargeting_ros',
        executable='mock_keypoints_publisher',
        name='mock_keypoints_publisher',
        parameters=[{
            'zmq_address': LaunchConfiguration('zmq_mocap'),
            'frequency': 50.0,
            'motion': LaunchConfiguration('motion'),
        }],
        output='screen',
    )

    bridge_node = Node(
        package='retargeting_ros',
        executable='hand_action_bridge',
        name='hand_action_bridge',
        parameters=[{
            'zmq_address': LaunchConfiguration('zmq_action'),
            'frequency': 250.0,
            'hand': 'right',
            'log_latency': True,
        }],
        output='screen',
    )

    return LaunchDescription([
        pb_env,
        motion_arg,
        zmq_mocap_arg,
        zmq_action_arg,
        rsp_node,
        rviz_node,
        mock_pub_node,
        bridge_node,
    ])
