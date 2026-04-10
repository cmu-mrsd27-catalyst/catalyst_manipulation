# catalyst_motion_planner

Move planning and planning scene utilities for the Catalyst Manipulator. Exposes MoveIt 2 **`MoveGroupInterface`** behind **`catalyst_interfaces/srv/JsonCommand`** (and a few standard ROS services) so Python nodes, action servers, and the behavior tree can command the arm without linking MoveIt directly.

## Executables

| Node | Description |
|------|-------------|
| `motion_planner` | Joint / Cartesian / gripper / guide / octomap / EEF bounds / collision sensitivity / stop |
| `scene_manager` | Load static world objects; `/scene_command` for add/remove collision objects |

---

## `motion_planner` services

All JSON services use **`JsonCommand`**: request field **`command`** is a JSON string; **`response`** is a JSON string.

### `/joint_command`

Joint-space motion (named SRDF pose or six angles in **degrees**).

```bash
ros2 service call /joint_command catalyst_interfaces/srv/JsonCommand \
  '{"command": "{\"joints\": [0, -30, 0, 60, 0, 30], \"speed\": 0.5}"}'
```

```bash
ros2 service call /joint_command catalyst_interfaces/srv/JsonCommand \
  '{"command": "{\"pose\": \"home\"}"}'
```

| Field | Notes |
|-------|--------|
| `pose` or `joints` (length 6) | One required |
| `speed` | Optional; scaled 0.01–1.0 |

### `/cartesian_command`

Pose in **`link_base`**: position (m) + quaternion `qx,qy,qz,qw`.

| Field | Notes |
|-------|--------|
| `x,y,z,qx,qy,qz,qw` | Required |
| `speed` | Optional 0.01–1.0 |
| `straight_line` | Optional; Cartesian path |
| `keep_orientation` | Optional; orientation path constraints |

Implementation uses multi-seed IK and optional orientation / EEF box constraints (see source).

### `/gripper_command` (sim / fake only when enabled)

`{"action": "open"}` or `"close"` via MoveIt named targets. **Disabled on real hardware** when `enable_gripper_service` is false (real gripper uses **`catalyst_gripper`**).

### `/guide_mode` (real hardware when enabled)

xArm SDK teach mode: `action` = `enable` | `disable` | `status`; optional `sensitivity` 1–5 (used on enable; default in code is **5** if omitted).

### `/set_octomap_enabled` (`std_srvs/SetBool`)

Toggles planning vs collision-free behavior w.r.t. octomap via cached allowed-collision matrix (requires working **`/get_planning_scene`** from MoveIt).

### `/set_collision_sensitivity` (`JsonCommand`)

`{"level": <0-5>}` — xArm collision sensitivity (only when xArm SDK is connected for guide mode on the same node).

### `/set_eef_bounds` (`JsonCommand`)

`{"action": "set", "x_min", "x_max", "z_min", "z_max"}` or `{"action": "clear"}` — position box on constraint link for path planning.

### `/stop_motion` (`std_srvs/srv/Empty`)

Emergency stop: sets internal flag and **`MoveGroupInterface::stop()`** for the current trajectory.

**Internal client:** subscribes/calls **`/get_planning_scene`** (MoveIt) at startup for octomap ACM caching.

---

## `scene_manager`

### `/scene_command` (`JsonCommand`)

Runtime add/remove of collision objects (boxes, etc.). At startup, loads **`world_objects_yaml`** if set (see `demo.launch.py`).

---

## Parameters (`motion_planner`)

| Parameter | Typical demo | Meaning |
|-----------|--------------|---------|
| `enable_gripper_service` | `true` fake/gazebo, `false` real | Expose `/gripper_command` via MoveIt |
| `enable_guide_mode` | `false` sim, `true` real | xArm SDK + `/guide_mode` + collision service |
| `robot_ip` | xArm IP | SDK connection |

MoveIt parameter server entries (`robot_description`, semantic, kinematics, …) are supplied by **`demo.launch.py`**.

---

## Architecture

Two internal **`rclcpp::Node`** instances share a **`MultiThreadedExecutor`**:

- **`motion_planner`** — services, clients, subscriptions
- **`motion_planner_moveit`** — **`MoveGroupInterface`** only (avoids executor deadlocks with MoveIt actions)

---

## Build

```bash
colcon build --packages-select xarm_sdk catalyst_motion_planner
```

(`xarm_sdk` needed for guide mode / collision sensitivity paths.)

---

## See also

- [catalyst_bringup README](../catalyst_bringup/README.md)
- [Repository README](../../../README.md)
