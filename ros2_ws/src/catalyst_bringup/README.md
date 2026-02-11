# Catalyst Bringup Package

Bringup package for the Catalyst Manipulator (xArm-6 + Bio Gripper). Provides launch files to start the robot with ros2_control and MoveIt.

## Package Structure

```
catalyst_bringup/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── ros2_controllers.yaml       # Full config (fake + Gazebo modes)
│   └── ros2_controllers_real.yaml  # Arm-only config (real hardware)
└── launch/
    ├── robot_bringup.launch.py
    └── demo.launch.py              # Unified 3-mode launch file
```

---

## Quick Start

```bash
# Build the workspace
cd ~/catalyst-manipulation/ros2_ws
colcon build
source install/setup.bash

# Fake hardware (quick MoveIt testing)
ros2 launch catalyst_bringup demo.launch.py sim:=fake

# Gazebo simulation
ros2 launch catalyst_bringup demo.launch.py sim:=gazebo

# Real robot (default)
ros2 launch catalyst_bringup demo.launch.py robot_ip:=192.168.1.212
```

---

## Launch Files

### demo.launch.py (Recommended)

Unified launch file supporting three modes: real hardware, fake hardware, and Gazebo simulation.

**Arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `sim` | `false` | Simulation mode: `false` (real hardware), `fake`, or `gazebo` |
| `robot_ip` | `192.168.1.212` | xArm IP address (only used when `sim:=false`) |
| `rviz` | `true` | Launch RViz |

**Per-mode behavior:**

| Aspect | `sim:=false` (real) | `sim:=fake` | `sim:=gazebo` |
|--------|---------------------|-------------|---------------|
| Hardware plugin | `UFRobotSystemHardware` | `UFRobotFakeSystemHardware` | `GazeboSystem` |
| Controller config | `ros2_controllers_real.yaml` | `ros2_controllers.yaml` | `ros2_controllers.yaml` |
| `ros2_control_node` | yes | yes | no (Gazebo plugin) |
| Gazebo server/client | no | no | yes |
| `bio_gripper_controller` | no | yes | yes |
| MoveIt controllers | arm only | arm + gripper | arm + gripper |
| `use_sim_time` | `false` | `false` | `true` |
| Gripper ros2_control in URDF | no | yes | yes |

**Real mode notes:**
- The bio gripper is controlled independently via the Dynamixel SDK (see `catalyst_gripper` package)
- Gripper URDF links/joints remain in the model for MoveIt collision checking
- A `joint_state_publisher` node merges joint states from multiple sources

---

### robot_bringup.launch.py

Starts only the robot hardware interface and controllers (no MoveIt/RViz).

```bash
ros2 launch catalyst_bringup robot_bringup.launch.py
```

**What it starts:**
- `robot_state_publisher`
- `ros2_control_node`
- `joint_state_broadcaster`
- `xarm6_traj_controller`
- `bio_gripper_controller`

**Arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation clock |
| `ros2_control_plugin` | `UFRobotFakeSystemHardware` | Hardware plugin |

---

## ros2_control Controllers

### Controller Configuration

- `config/ros2_controllers.yaml` — Full config with arm + gripper (fake and Gazebo modes)
- `config/ros2_controllers_real.yaml` — Arm-only config (real hardware, gripper handled by `catalyst_gripper`)

### Controllers

| Controller | Type | Joints |
|------------|------|--------|
| `joint_state_broadcaster` | JointStateBroadcaster | All joints (publishes to `/joint_states`) |
| `xarm6_traj_controller` | JointTrajectoryController | joint1-6 |
| `bio_gripper_controller` | JointTrajectoryController | right_finger_joint |

### Controller Topics

| Controller | Action Server |
|------------|--------------|
| `xarm6_traj_controller` | `/xarm6_traj_controller/follow_joint_trajectory` |
| `bio_gripper_controller` | `/bio_gripper_controller/follow_joint_trajectory` |

---

## Hardware Plugins

| Plugin | Package | Use Case |
|--------|---------|----------|
| `uf_robot_hardware/UFRobotFakeSystemHardware` | xarm_controller | Fake hardware (`sim:=fake`) |
| `uf_robot_hardware/UFRobotSystemHardware` | xarm_controller | Real xArm hardware (`sim:=false`) |
| `gazebo_ros2_control/GazeboSystem` | gazebo_ros2_control | Gazebo simulation (`sim:=gazebo`) |

### Using Real Hardware

```bash
ros2 launch catalyst_bringup demo.launch.py robot_ip:=192.168.1.212
```

The gripper on real hardware is a Dynamixel AX-18A controlled separately — see `catalyst_gripper` package.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        User Interface                        │
│                     (RViz / MoveIt GUI)                     │
└─────────────────────────┬───────────────────────────────────┘
                          │ Plan & Execute
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                        MoveIt                                │
│                     (move_group)                            │
└─────────────────────────┬───────────────────────────────────┘
                          │ FollowJointTrajectory
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    ros2_control                              │
│              (controller_manager)                           │
│  ┌──────────────────┐  ┌──────────────────────────────┐    │
│  │ joint_state_     │  │ xarm6_traj_controller        │    │
│  │ broadcaster      │  │ bio_gripper_controller       │    │
│  └──────────────────┘  └──────────────────────────────┘    │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                   Hardware Interface                         │
│            (UFRobotFakeSystemHardware)                      │
└─────────────────────────────────────────────────────────────┘
```

---

## Troubleshooting

### Controller not found

```
[ERROR] Could not find controller 'xarm6_traj_controller'
```

**Solution:** Wait for controller_manager to fully start, or check `ros2_controllers.yaml`.

### Action server not available

```
[ERROR] Action client not connected: xarm6_traj_controller/follow_joint_trajectory
```

**Solution:** Ensure controllers are spawned. Check with:
```bash
ros2 control list_controllers
```

### Hardware interface not loaded

```
[ERROR] Could not load hardware 'uf_robot_hardware/UFRobotFakeSystemHardware'
```

**Solution:** Ensure xarm_ros2 packages are built:
```bash
colcon build --packages-select xarm_controller
```

---

## Dependencies

- `description` - Robot URDF
- `moveit_config` - MoveIt configuration
- `xarm_controller` - xArm hardware interface
- `controller_manager` - ros2_control
- `joint_state_broadcaster` - Joint state publisher
- `joint_trajectory_controller` - Trajectory execution
- `joint_state_publisher` - Joint state merging (real mode)
- `gazebo_ros` - Gazebo integration (Gazebo mode)
- `catalyst_gazebo` - World/model assets (Gazebo mode)

## See Also

- [catalyst_gripper README](../catalyst_gripper/README.md) - Dynamixel gripper driver
- [catalyst_gazebo README](../catalyst_gazebo/README.md) - Gazebo simulation environment
- [description README](../description/README.md) - Robot URDF
