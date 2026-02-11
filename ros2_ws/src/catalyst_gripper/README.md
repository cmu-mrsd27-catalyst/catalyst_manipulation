# catalyst_gripper

ROS 2 driver for the Dynamixel AX-18A bio gripper on the Catalyst Manipulator.

## Overview

This package controls the gripper independently from the xArm controller. The gripper is a Dynamixel AX-18A servo that is **not** part of the xArm's ros2_control chain — it uses the Dynamixel SDK directly.

MoveIt still sees the gripper geometry (links, joints, collision meshes) from the URDF for collision avoidance, but does not execute gripper trajectories on real hardware.

## Joint State Publishing

The gripper node publishes `right_finger_joint` to `/joint_states` so that:
- `robot_state_publisher` computes the gripper TF tree
- MoveIt sees the correct collision state of the gripper

`left_finger_joint` is handled automatically via the URDF mimic relationship (mirrors `right_finger_joint` with multiplier `-1.0`).

## Installation

Install the Dynamixel SDK:

```bash
pip install dynamixel-sdk
```

Build the package:

```bash
cd ~/catalyst-manipulation/ros2_ws
colcon build --packages-select catalyst_gripper
```

## Usage

### Standalone

```bash
ros2 run catalyst_gripper gripper_node --ros-args --params-file \
  $(ros2 pkg prefix catalyst_gripper)/share/catalyst_gripper/config/gripper_params.yaml
```

### With the unified launch file

In real-robot mode (`sim:=false`), the gripper node should be launched separately or added to a workspace-level launch file:

```bash
# Terminal 1: Robot + MoveIt
ros2 launch catalyst_bringup demo.launch.py robot_ip:=192.168.1.212

# Terminal 2: Gripper
ros2 run catalyst_gripper gripper_node --ros-args --params-file \
  $(ros2 pkg prefix catalyst_gripper)/share/catalyst_gripper/config/gripper_params.yaml
```

## Configuration

Edit `config/gripper_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `port` | `/dev/ttyUSB0` | Serial port for Dynamixel |
| `baud_rate` | `1000000` | Baud rate (AX-18A default) |
| `servo_id` | `1` | Dynamixel servo ID |
| `min_position` | `0.0` | Closed position (meters) |
| `max_position` | `0.037` | Open position (meters) |
| `publish_rate` | `30.0` | Joint state publish rate (Hz) |

## Command Interface

Send gripper commands (position in meters):

```bash
ros2 topic pub /gripper_node/command std_msgs/msg/Float64 "{data: 0.02}"
```
