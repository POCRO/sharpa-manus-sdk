# Manus SDK Integration for SharpaWave

This repository is used for integration and demonstration of SharpaWave with Manus MetaGloves Pro, including a Manus data acquisition client and hand retargeting examples.

## Overview

The project is organized into two main components:

- `client/`: Linux client based on Manus SDK (C++), responsible for glove data acquisition and publishing.
- `retargeting/`: Retargeting and visualization modules (Python), responsible for mapping keypoints to SharpaWave joint space.
- `retargeting_ros/`: ROS2 package for pipeline testing without hardware (mock publisher + RViz bridge).

## Repository Layout

```text
manus-sdk/
├── client/
│   ├── ManusSDK/
│   ├── Makefile
│   └── *.cpp / *.cc
├── retargeting/
│   ├── retargeting_manus_demo_multiprocess.py
│   ├── environment.yml
│   └── requirements.txt
├── retargeting_ros/
│   ├── retargeting_ros/
│   │   ├── mock_keypoints_publisher.py
│   │   └── hand_action_bridge.py
│   └── launch/
│       └── retargeting_test.launch.py
├── SharpaWave with Manus User Manual.pdf
└── README.md
```

## Documentation

For complete usage instructions, installation steps, network configuration, calibration workflow, developer interfaces, and troubleshooting, refer to:

- `SharpaWave with Manus User Manual.pdf`

Recommended reading order:

1. Setup Guide
2. Calibration
3. Developer Guide

## Quick Entry

For quick navigation, start with:

- Client build entry: `client/Makefile`
- Retargeting demo entry: `retargeting/retargeting_manus_demo_multiprocess.py`
- Python environment: `retargeting/environment.yml`

## Hardware-Free Pipeline Test (ROS2)

A ROS2-based test pipeline is available in `retargeting_ros/` for validating the full retargeting stack without Manus gloves or SharpaWave hardware.

### Pipeline

```
mock_keypoints_publisher
    → ZMQ tcp://*:2044
        → retargeting_manus_demo_multiprocess (CasADi optimizer)
            → ZMQ tcp://localhost:6668
                → hand_action_bridge
                    → /joint_states (ROS2)
                        → robot_state_publisher
                            → RViz2
```

### Usage

Terminal 1 — launch ROS2 nodes:
```bash
cd ~/leo_ws
source install/setup.bash
ros2 launch retargeting_ros retargeting_test.launch.py motion:=wave
# motion options: wave | fist | static
```

Terminal 2 — launch CasADi optimizer:
```bash
cd ~/leo_ws/src/sharpa-manus-sdk/retargeting
~/retargeting_venv/bin/python retargeting_manus_demo_multiprocess.py \
    -mocap_address tcp://localhost:2044
```

Add `-debug_print` to see per-frame optimization cost and timing.

### Benchmark Results (no hardware, ROS2 Jazzy, Ubuntu 24.04)

| Metric | Value |
|---|---|
| Optimizer (CasADi/IPOPT) per frame | ~8–15 ms |
| bridge→/joint_states latency (mean) | ~3 ms |
| Optimization cost (open hand) | ~18 |
| Optimization cost (fist) | ~25 |
| Mock publisher frequency | 50 Hz |
| bridge frequency | 250 Hz |

### Known Issues & Fixes Applied

- `mock_keypoints_publisher`: REST_KEYPOINTS were updated to match URDF cumulative joint positions (extracted from `right_sharpa_ha4.urdf`). Left/right hand assignment was corrected (REST_KEYPOINTS matches left hand URDF; right hand is Y-mirrored).
- `curl_keypoints`: Fixed incorrect pivot index for thumb (was CMC index 1, corrected to MCP index 2).
- Protobuf version mismatch: resolved by setting `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` in launch file.
- `rcl_shutdown already called` on Ctrl+C: cosmetic error from rclpy, does not affect runtime.

## License

Repository license terms are provided in the `License` file.
Manus SDK components are subject to their official license terms and must be used accordingly.
