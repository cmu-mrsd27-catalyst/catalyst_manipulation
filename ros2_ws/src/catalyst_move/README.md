# catalyst_move

Motion control package for the xArm6 on the Catalyst Manipulator. Provides both a CLI tool for manual testing and ROS2 services for programmatic arm control.

## Overview

This package provides:

- **`test_move`** — Interactive CLI menu for quick manual testing of arm poses and gripper actions.
- **`arm_control`** — ROS2 service node exposing cartesian and joint position control via JSON over string services.

Both use MoveIt2's `MoveGroup` action for motion planning and execution.

## Installation

Build the package (after `catalyst_interfaces` and `moveit_config` are already built):

```bash
cd ~/catalyst-manipulation/ros2_ws
colcon build --packages-select catalyst_move
source install/setup.bash
```

## Arm Control Service Node

The `arm_control` node exposes two services for programmatic control. Both use the `catalyst_interfaces/srv/GripperCommand` type (string request / string response) with JSON payloads.

```bash
# Terminal 1: Launch MoveIt2 (fake/sim/real)
ros2 launch catalyst_bringup demo.launch.py sim:=fake

# Terminal 2: Start the arm control service node
ros2 run catalyst_move arm_control
```

### Cartesian Service — `/arm_control/cartesian`

Move the end-effector to a cartesian pose (position in meters, orientation as quaternion). The format matches TF2 transform output directly — no conversion needed.

**Request:**
```json
{"x": 0.3, "y": 0.0, "z": 0.5, "qx": 0.0, "qy": 0.707, "qz": 0.0, "qw": 0.707}
```

**Response:**
```json
{"success": true, "message": "Motion succeeded"}
```

**CLI example:**
```bash
ros2 service call /arm_control/cartesian catalyst_interfaces/srv/GripperCommand \
  "{command: '{\"x\": 0.3, \"y\": 0.0, \"z\": 0.5, \"qx\": 0.0, \"qy\": 0.707, \"qz\": 0.0, \"qw\": 0.707}'}"
```

### Joint Service — `/arm_control/joint`

Move to a set of joint angles (in degrees) or a named pose.

**Request (joint angles):**
```json
{"joints": [0, 0, 0, 0, -90, 0]}
```

**Request (named pose):**
```json
{"pose": "home"}
```

**Response:**
```json
{"success": true, "message": "Motion succeeded"}
```

**CLI examples:**
```bash
# Joint angles
ros2 service call /arm_control/joint catalyst_interfaces/srv/GripperCommand \
  "{command: '{\"joints\": [0, -30, 0, 0, -90, 0]}'}"

# Named pose
ros2 service call /arm_control/joint catalyst_interfaces/srv/GripperCommand \
  "{command: '{\"pose\": \"home\"}'}"
```

### Named Poses

| Pose | Joint values |
|------|-------------|
| `home` | All joints at 0° |
| `hold_up` | joint5 at -90°, all others at 0° |

## Test Move CLI

Interactive menu for manual testing:

```bash
ros2 run catalyst_move test_move
```

```
=== Catalyst Move Test ===
1. Move to joint angles
2. Move to cartesian pose
3. Open gripper
4. Close gripper
5. Release gripper
6. Go to home pose
7. Go to hold_up pose
0. Exit
```

- **Joint angles**: Enter 6 values in degrees, comma-separated.
- **Cartesian pose**: Enter x, y, z (meters) and roll, pitch, yaw (degrees), comma-separated. Pose is in the `link_base` frame targeting `link_eef`.
- **Gripper**: Commands are sent to `/gripper_node/gripper_command`. The gripper node must be running.

## Configuration

| Setting | Value |
|---------|-------|
| Planning group | `xarm6` |
| Joints | `joint1` through `joint6` |
| Reference frame | `link_base` |
| End-effector link | `link_eef` |
| MoveGroup action | `move_action` |

## Dependencies

- `rclpy`
- `moveit_msgs`
- `geometry_msgs`
- `shape_msgs`
- `catalyst_interfaces`
