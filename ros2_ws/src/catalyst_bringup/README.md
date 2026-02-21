# Catalyst Bringup Package

Bringup package for the Catalyst Manipulator (xArm-6 + Bio Gripper). Provides launch files to start the robot with ros2_control and MoveIt in three modes: real hardware, fake hardware, and Gz Harmonic simulation.

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
    ├── demo.launch.py              # Unified 3-mode launch file
    └── robot_bringup.launch.py     # Hardware + controllers only
```

---

## Quick Start

```bash
cd ~/catalyst_repo_jazzy/catalyst-manipulation/ros2_ws
colcon build
source install/setup.bash

# Fake hardware (quick MoveIt testing)
ros2 launch catalyst_bringup demo.launch.py sim:=fake

# Gz Harmonic simulation
ros2 launch catalyst_bringup demo.launch.py sim:=gazebo

# Real robot
ros2 launch catalyst_bringup demo.launch.py robot_ip:=192.168.1.212
```

---

## Launch Files

### demo.launch.py (Recommended)

Unified launch file supporting three modes.

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `sim` | `false` | Simulation mode: `false` (real), `fake`, or `gazebo` |
| `robot_ip` | `192.168.1.212` | xArm IP address (only used when `sim:=false`) |
| `rviz` | `true` | Launch RViz |

**Per-mode behavior:**

| Aspect | `sim:=false` (real) | `sim:=fake` | `sim:=gazebo` |
|--------|---------------------|-------------|---------------|
| Hardware plugin | `UFRobotSystemHardware` | `UFRobotFakeSystemHardware` | `GazeboSimSystem` |
| Controller config | `ros2_controllers_real.yaml` | `ros2_controllers.yaml` | `ros2_controllers.yaml` |
| `ros2_control_node` | yes | yes | no (Gz plugin) |
| Gz Harmonic | no | no | yes |
| `bio_gripper_controller` | no | yes | yes |
| MoveIt controllers | arm only | arm + gripper | arm + gripper |
| `use_sim_time` | `false` | `false` | `true` |
| Clock bridge | no | no | yes (ros_gz_bridge) |

### robot_bringup.launch.py

Starts only the robot hardware interface and controllers (no MoveIt/RViz).

```bash
ros2 launch catalyst_bringup robot_bringup.launch.py
```

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation clock |
| `ros2_control_plugin` | `UFRobotFakeSystemHardware` | Hardware plugin |

---

## ros2_control Controllers

### Controller Configuration

- `config/ros2_controllers.yaml` — Full config with arm + gripper (fake and Gazebo modes)
- `config/ros2_controllers_real.yaml` — Arm-only config (real hardware)

### Controllers

| Controller | Type | Joints |
|------------|------|--------|
| `joint_state_broadcaster` | JointStateBroadcaster | All joints |
| `xarm6_traj_controller` | JointTrajectoryController | joint1–joint6 |
| `bio_gripper_controller` | JointTrajectoryController | right_finger_joint |

---

## Hardware Plugins

| Plugin | Package | Use Case |
|--------|---------|----------|
| `uf_robot_hardware/UFRobotFakeSystemHardware` | xarm_controller | Fake hardware (`sim:=fake`) |
| `uf_robot_hardware/UFRobotSystemHardware` | xarm_controller | Real xArm (`sim:=false`) |
| `gz_ros2_control/GazeboSimSystem` | gz_ros2_control | Gz Harmonic (`sim:=gazebo`) |

---

## Event Sequencing

### Gazebo mode
```
Gz Harmonic launch → spawn_robot → joint_state_broadcaster → xarm6_traj_controller ─┬→ move_group
                                                            → bio_gripper_controller  └→ rviz
```

### Fake / Real mode
```
ros2_control_node → joint_state_broadcaster → xarm6_traj_controller ─┬→ move_group
                                             → bio_gripper_controller  └→ rviz
```

---

## Dependencies

- `catalyst_description` — Robot URDF
- `catalyst_moveit_config` — MoveIt configuration
- `catalyst_gazebo` — Gz Harmonic world/models (runtime, Gazebo mode)
- `xarm_controller` — xArm hardware interface
- `controller_manager` — ros2_control
- `ros_gz_sim` — Gz Harmonic ROS 2 integration
- `gz_ros2_control` — ros2_control plugin for Gz Harmonic
- `ros_gz_bridge` — Clock bridge for Gz Harmonic

---

## See Also

- [catalyst_description README](../catalyst_description/README.md) — Robot URDF
- [catalyst_moveit_config README](../catalyst_moveit_config/README.md) — MoveIt configuration
- [catalyst_gazebo README](../catalyst_gazebo/README.md) — Gz Harmonic simulation environment
