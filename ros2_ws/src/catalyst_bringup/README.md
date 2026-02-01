# Catalyst Bringup Package

Bringup package for the Catalyst Manipulator (xArm-6 + Bio Gripper). Provides launch files to start the robot with ros2_control and MoveIt.

## Package Structure

```
catalyst_bringup/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── ros2_controllers.yaml
└── launch/
    ├── robot_bringup.launch.py
    └── demo.launch.py
```

---

## Quick Start

```bash
# Build the workspace
cd ~/catalyst-manipulation/ros2_ws
colcon build
source install/setup.bash

# Launch the full demo (fake hardware + MoveIt + RViz)
ros2 launch catalyst_bringup demo.launch.py
```

---

## Launch Files

### demo.launch.py (Recommended)

Full demo with fake hardware, MoveIt, and RViz. **Use this for testing motion planning and execution.**

```bash
ros2 launch catalyst_bringup demo.launch.py
```

**What it starts:**
- `robot_state_publisher` - Publishes TF transforms
- `ros2_control_node` - Controller manager with fake hardware
- `joint_state_broadcaster` - Publishes joint states
- `xarm6_traj_controller` - Arm trajectory controller
- `bio_gripper_controller` - Gripper trajectory controller
- `move_group` - MoveIt planning server
- `rviz2` - Visualization with MoveIt plugin

**Arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation clock |
| `rviz_config` | `moveit.rviz` | Path to RViz config |

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

Location: `config/ros2_controllers.yaml`

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
| `uf_robot_hardware/UFRobotFakeSystemHardware` | xarm_controller | Fake hardware (default) |
| `uf_robot_hardware/UFRobotSystemHardware` | xarm_controller | Real xArm hardware |
| `mock_components/GenericSystem` | ros2_control | Generic mock (no xarm dep) |

### Using Real Hardware

```bash
ros2 launch catalyst_bringup robot_bringup.launch.py \
  ros2_control_plugin:=uf_robot_hardware/UFRobotSystemHardware
```

**Note:** Real hardware requires additional parameters (robot_ip, etc.) in the URDF.

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
