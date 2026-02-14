# catalyst_gripper

ROS 2 driver for the Dynamixel AX-18A bio gripper on the Catalyst Manipulator.

## Overview

This package controls the gripper independently from the xArm controller. The gripper is a Dynamixel AX-18A servo that is **not** part of the xArm's ros2_control chain — it uses the Dynamixel SDK directly.

MoveIt still sees the gripper geometry (links, joints, collision meshes) from the URDF for collision avoidance, but does not execute gripper trajectories on real hardware.

## Joint State Publishing

The gripper node publishes `right_finger_joint` to `/joint_states` at a configurable rate (default 30 Hz) so that:
- `robot_state_publisher` computes the gripper TF tree
- MoveIt sees the correct collision state of the gripper

Position is read from the real servo and converted from ticks to meters. `left_finger_joint` is handled automatically via the URDF mimic relationship (mirrors `right_finger_joint` with multiplier `-1.0`).

## Installation

Install the Dynamixel SDK:

```bash
pip install dynamixel-sdk
```

Build the interfaces package first, then the gripper package:

```bash
cd ~/catalyst-manipulation/ros2_ws
colcon build --packages-select catalyst_interfaces
colcon build --packages-select catalyst_gripper
source install/setup.bash
```

## Usage

### Standalone

```bash
ros2 run catalyst_gripper gripper_node
```

To load gripper config from a YAML file:

```bash
ros2 run catalyst_gripper gripper_node --ros-args \
  -p config_path:=$(ros2 pkg prefix catalyst_gripper)/share/catalyst_gripper/config/gripper_params.yaml
```

### With the unified launch file

In real-robot mode (`sim:=false`), the gripper node should be launched separately or added to a workspace-level launch file:

```bash
# Terminal 1: Robot + MoveIt
ros2 launch catalyst_bringup demo.launch.py robot_ip:=192.168.1.212

# Terminal 2: Gripper
ros2 run catalyst_gripper gripper_node
```

## Configuration

The node reads gripper hardware config from `gripper_params.yaml` (via the `config_path` parameter). Key settings:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `devicename` | `/dev/ttyUSB0` | Serial port for Dynamixel |
| `baudrate` | `1000000` | Baud rate (AX-18A default) |
| `dxl_id` | `7` | Dynamixel servo ID |
| `pos_min` | `30` | Minimum position limit (ticks, fully open) |
| `pos_max` | `720` | Maximum position limit (ticks, fully closed) |
| `close_speed` | `150` | Motor speed during closing (0-1023) |
| `load_threshold` | `80` | Opposing load threshold for contact detection |
| `hold_torque_limit` | `900` | Torque limit while holding an object |

ROS parameters on the node:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `config_path` | `''` | Path to `gripper_params.yaml` (uses defaults if empty) |
| `max_position` | `0.037` | Maximum joint position in meters |
| `publish_rate` | `30.0` | Joint state publish rate (Hz) |

## Service Interface

The gripper exposes a `~/gripper_command` service (`catalyst_interfaces/srv/GripperCommand`) that accepts JSON commands:

### Close the gripper

```bash
ros2 service call /gripper_node/gripper_command catalyst_interfaces/srv/GripperCommand \
  "{command: '{\"action\": \"close\"}'}"
```

Returns `{"success": true, "message": "contact detected", "position": 450}` if an object is grasped, or `{"success": false, "message": "no object detected", "position": 705}` if no object is found.

### Open the gripper

```bash
# Open to default position
ros2 service call /gripper_node/gripper_command catalyst_interfaces/srv/GripperCommand \
  "{command: '{\"action\": \"open\"}'}"

# Open to a specific servo position (ticks)
ros2 service call /gripper_node/gripper_command catalyst_interfaces/srv/GripperCommand \
  "{command: '{\"action\": \"open\", \"position\": 200}'}"
```

### Release (disable torque)

```bash
ros2 service call /gripper_node/gripper_command catalyst_interfaces/srv/GripperCommand \
  "{command: '{\"action\": \"release\"}'}"
```

## Monitoring

```bash
# Check joint states are publishing
ros2 topic echo /joint_states

# List available services
ros2 service list | grep gripper
```
