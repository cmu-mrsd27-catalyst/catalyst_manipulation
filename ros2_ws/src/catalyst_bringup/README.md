# Catalyst Bringup Package

Bringup for the Catalyst Manipulator (xArm-6 + Bio Gripper): **unified launch** for real hardware, fake hardware, and Gz Harmonic simulation, including MoveIt, optional vision, world model, motion planner, and scene manager.

## Package structure

```
catalyst_bringup/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── ros2_controllers.yaml       # Arm + gripper (fake + Gazebo)
│   └── ros2_controllers_real.yaml  # Arm + F/T broadcaster (real; no ros2_control gripper)
└── launch/
    ├── demo.launch.py              # Main unified stack (recommended)
    ├── robot_bringup.launch.py     # Controllers + hardware only
    └── admittance_test.launch.py   # Admittance / F-T test helper
```

---

## Quick start

```bash
cd <workspace>/ros2_ws
source install/setup.bash

# Fake hardware (MoveIt + sim gripper controller, no camera)
ros2 launch catalyst_bringup demo.launch.py sim:=fake

# Gz Harmonic
ros2 launch catalyst_bringup demo.launch.py sim:=gazebo

# Real robot (xArm + F/T + Dynamixel gripper node + RealSense/AprilTag + filtered cloud + world model + planners)
ros2 launch catalyst_bringup demo.launch.py sim:=false robot_ip:=192.168.1.212 ft_sensor_ip:=192.168.2.1
```

---

## `demo.launch.py` (main entry)

Single entry point for the full manipulation stack appropriate to each mode.

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `sim` | `false` | `false` = real, `fake` = fake HW, `gazebo` = Gz Harmonic |
| `robot_ip` | `192.168.1.212` | xArm IP (`sim:=false` only) |
| `ft_sensor_ip` | `192.168.2.1` | F/T sensor IP (`sim:=false` only) |
| `rviz` | `true` | Launch RViz with MoveIt config |

### Per-mode summary

| Aspect | `sim:=false` (real) | `sim:=fake` | `sim:=gazebo` |
|--------|---------------------|-------------|---------------|
| Plugin | `UFRobotSystemHardware` | `UFRobotFakeSystemHardware` | `GazeboSimSystem` |
| Controller YAML | `ros2_controllers_real.yaml` | `ros2_controllers.yaml` | `ros2_controllers.yaml` |
| `ros2_control_node` | yes | yes | no (Gz plugin) |
| Gripper | `catalyst_gripper` node + `/gripper_command` | `bio_gripper_controller` | `bio_gripper_controller` |
| MoveIt controller list | arm only | arm + gripper | arm + gripper |
| OctoMap / `sensors_3d` | yes (real) | no | no |
| `pointcloud_filter` + `detection_launch` | yes (real) | no | no |
| `world_model` | yes | yes | yes |
| `motion_planner` | yes (`enable_guide_mode` on real; `enable_gripper_service` off on real) | yes | yes |
| `scene_manager` | yes | yes | yes |
| `use_sim_time` | false | false | true |

**Sequencing:** After `xarm6_traj_controller` is spawned, `move_group` and RViz start; `motion_planner` and `scene_manager` start after a short delay; on real hardware, `world_model` and vision launch are also delayed so dependencies are up.

**Admittance:** The admittance controller is intentionally **not** auto-spawned on real hardware (conflict with trajectory controller after teach mode). Load on demand via `admittance_test.launch.py` or related tools.

---

## Other launch files

### `robot_bringup.launch.py`

Controllers and hardware interface only (no MoveIt, RViz, vision, or execute stack).

```bash
ros2 launch catalyst_bringup robot_bringup.launch.py
```

### `admittance_test.launch.py`

Focused launch for admittance / F-T experiments.

```bash
ros2 launch catalyst_bringup admittance_test.launch.py
```

---

## Controllers (reference)

| Controller | Role |
|------------|------|
| `joint_state_broadcaster` | Publishes `/joint_states` |
| `xarm6_traj_controller` | Arm trajectory |
| `bio_gripper_controller` | Gripper (fake / Gazebo only) |
| `force_torque_sensor_broadcaster` | F/T (real only) |

---

## Dependencies (high level)

- `catalyst_description`, `catalyst_moveit_config`, `catalyst_motion_planner`, `catalyst_world_model`
- `catalyst_vision`, `catalyst_gripper` (real mode)
- `catalyst_gazebo` (Gazebo mode)
- `xarm_controller`, `controller_manager`, MoveIt, `ros_gz_*` (sim)

---

## See also

- [Repository README](../../../README.md) — end-to-end demo commands
- [catalyst_motion_planner README](../catalyst_motion_planner/README.md) — JSON motion services
- [catalyst_bt README](../catalyst_bt/README.md) — `/bt_execute` orchestration
