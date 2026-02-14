# catalyst_move

Interactive test script for moving the xArm6 arm and controlling the gripper on the Catalyst Manipulator.

## Overview

This package provides a CLI menu for sending motion commands to the xArm6 via MoveIt2's `MoveGroup` action, and controlling the gripper via the `GripperCommand` service. It is intended for quick manual testing of arm poses and gripper actions.

## Installation

Build the package (after `catalyst_interfaces` and `moveit_config` are already built):

```bash
cd ~/catalyst-manipulation/ros2_ws
colcon build --packages-select catalyst_move
source install/setup.bash
```

## Usage

First launch the robot + MoveIt2 stack, then run the test script:

```bash
# Terminal 1: Launch MoveIt2 (fake/sim/real)
ros2 launch catalyst_bringup demo.launch.py sim:=fake

# Terminal 2: Run test script
ros2 run catalyst_move test_move
```

The interactive menu:

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

### Move to joint angles

Enter 6 joint values in degrees (comma-separated). They are converted to radians internally and sent as a joint-space goal to MoveIt2.

```
Enter 6 joint angles in degrees (comma-separated): 0, -30, 0, 0, -90, 0
```

### Move to cartesian pose

Enter x, y, z in meters and roll, pitch, yaw in degrees (comma-separated). The pose is specified in the `link_base` frame, targeting the `link_eef` end-effector link.

```
Enter x,y,z (meters) and roll,pitch,yaw (degrees) comma-separated: 0.3, 0.0, 0.4, 180, 0, 0
```

### Named poses

| Pose | Joint values |
|------|-------------|
| `home` | All joints at 0° |
| `hold_up` | joint5 at -90°, all others at 0° |

### Gripper control

Open, close, and release commands are sent to `/gripper_node/gripper_command` (`catalyst_interfaces/srv/GripperCommand`). The gripper node must be running for these to work.

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
- `catalyst_interfaces`
