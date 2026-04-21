# Catalyst Manipulation (ROS 2 Jazzy)

Workspace for the **Catalyst Manipulator**: xArm-6, Bio Gripper, MoveIt 2, AprilTag vision, behavior-tree orchestration, and force-assisted placement.

The ROS 2 workspace lives under **`ros2_ws/`**.

## Prerequisites

- ROS 2 **Jazzy** (desktop or similar)
- Dependencies pulled in via `rosdep` and package `package.xml` files (MoveIt, RealSense, xArm stack, BehaviorTree.CPP, etc.)

## Build

```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # if needed
colcon build --symlink-install
source install/setup.bash
```

Use `colcon build --packages-select <pkg>` while iterating on a single package.

---

## Full demo (real hardware)

Typical layout uses **multiple terminals**, all after `source ros2_ws/install/setup.bash`.

### 1. Robot stack (MoveIt, controllers, vision, world model, motion planner)

Starts `ros2_control` (real xArm + F/T), MoveIt `move_group`, RViz, `catalyst_gripper`, RealSense + AprilTag (`detection_launch.py`), point cloud filter → OctoMap, `world_model`, `scene_manager`, and `motion_planner`.

```bash
ros2 launch catalyst_bringup demo.launch.py sim:=false robot_ip:=192.168.1.212 ft_sensor_ip:=192.168.2.1
```

Optional launch arguments:

| Argument | Default | Meaning |
|----------|---------|---------|
| `sim` | `false` | `false` = real arm, `fake` = fake HW, `gazebo` = Gz Harmonic |
| `robot_ip` | `192.168.1.212` | xArm controller IP (`sim:=false` only) |
| `ft_sensor_ip` | `192.168.2.1` | OnRobot F/T sensor IP (`sim:=false` only) |
| `rviz` | `true` | Set `false` to skip RViz |

**Note:** Vision, point cloud filter, and OctoMap-related pieces are started only when `sim:=false` in the current `demo.launch.py`.

### 2. Execute-layer services (separate processes)

Pick/place use SDK and pose helpers that are **not** started by `demo.launch.py`:

```bash
ros2 run catalyst_execute sdk_control_service
```

```bash
ros2 run catalyst_execute compute_poses_service
```

### 3. Action servers (ExecuteTask)

```bash
ros2 run catalyst_execute explore_action_server
```

```bash
ros2 run catalyst_execute detect_well_plate_action_server
```

Runs **`/detect_well_plate`**: move to a tag-calibrated inspection pose, grab a camera frame, POST to the remote GPU FastAPI server (`detect_well_plate_action_server` in `catalyst_execute_params.yaml`). Requires a color image topic (e.g. RealSense) and a reachable `server_url`.

```bash
ros2 run catalyst_execute pick_action_server
```

```bash
ros2 run catalyst_execute place_action_server
```

**Robot-side container** (hardcoded hover/grasp poses; relaxes planning-scene collisions with `workspace_box` between pre-hover and return — see `catalyst_execute_params.yaml`):

```bash
ros2 run catalyst_execute place_on_robot_action_server
```

```bash
ros2 run catalyst_execute pick_from_robot_container_action_server
```

| Action | `ExecuteTask` name | Role |
|--------|-------------------|------|
| Place on robot | `/place_on_robot` | Transit (`down_right` → pre-place) → SDK place → retract → same `down_right` → home |
| Pick from robot container | `/pick_from_robot_container` | Transit (`down_right` → pre-pick) → grasp → retract → same `down_right` → home |
| Well-plate detect | `/detect_well_plate` | Cartesian to calibrated view → HTTP detect → `pick_safe` / `place_safe` (optional `require` in goal) |

**Where does “home” run?** In the default orchestration trees, **joint home is owned by the behavior tree** (`InitMinimal` at the start of a task, `HomeEnd` after pick/place). The pick/place **action servers** do not send the arm home on success; they stop at approach/retract poses so the next BT node or a manual move can run.

### 4. Behavior tree executor

Registers trees from `catalyst_bt/trees/orchestration.xml` and waits on **`/bt_execute`**:

```bash
ros2 launch catalyst_bt pick_and_place_bt.launch.py
```

### 5. Run a task (from another terminal)

Tasks are defined in `orchestration.xml` (e.g. `MainPickOnly`, `MainPickPlace`, `MainPickThenPlaceOnRobot`).

```bash
ros2 service call /bt_execute catalyst_interfaces/srv/JsonCommand \
  "{command: '{\"task\": \"pick_only\"}'}"
```

| `task` (JSON string) | Tree | Typical use |
|----------------------|------|---------------|
| `pick_only` | `MainPickOnly` | Pick from workpiece |
| `place_only` | `MainPlaceOnly` | Place on workpiece |
| `pick_place` | `MainPickPlace` | Pick then place |
| `place_pick` | `MainPlacePick` | Place then pick |
| `pick_then_place_on_robot` | `MainPickThenPlaceOnRobot` | Pick from workpiece, then place into robot-side container |
| `pick_from_robot_container` | `MainPickFromRobotContainer` | Pick from robot-side container |
| `pick_only_with_detect` | `MainPickOnlyWithDetect` | Explore → detect (`require: pick_safe`) → pick → home |
| `place_only_with_detect` | `MainPlaceOnlyWithDetect` | Explore → detect (`require: place_safe`) → place → home |
| `pick_place_with_detect` | `MainPickPlaceWithDetect` | Explore → detect (`require: pick_safe`) → pick → place → home |
| `place_pick_with_detect` | `MainPlacePickWithDetect` | Explore → detect (`require: place_safe`) → place → pick → home |
| `demo_orchestration` | `MainDemoOrchestration` | Full demo: vision+pick machine → `place_on_robot` → home → vision+`place_safe` → pick container → `place` on machine → home |
| `demo_orchestration_2` | `MainDemoOrchestration2` | explore + detect `place_safe` (empty hand) → `pick_from_robot_container` → `place` → home → explore + detect `pick_safe` → `pick` → home → `place_on_robot` → home |
| `pick_container_then_place_on_robot` | `MainPickContainerThenPlaceOnRobot` | `/pick_from_robot_container` (retract, MoveIt, `down_right`, home) then `/place_on_robot` (transit to `pre_place`, SDK admittance, release, restart, home) |

Requires the corresponding action servers to be running (including `/place_on_robot` and `/pick_from_robot_container` where those flows appear). Vision tasks (including **`demo_orchestration`**) also need **`detect_well_plate_action_server`**, camera streaming, and the GPU detection HTTP server.

---

## Simulation quick start

**Fake hardware** (no robot, gripper via MoveIt in stack):

```bash
ros2 launch catalyst_bringup demo.launch.py sim:=fake
```

**Gazebo Harmonic** (full sim; primary path is unified demo):

```bash
ros2 launch catalyst_bringup demo.launch.py sim:=gazebo
```

---

## Other launch files (`catalyst_bringup`)

| Launch file | Purpose |
|-------------|---------|
| `demo.launch.py` | Main unified bringup (see above) |
| `robot_bringup.launch.py` | Hardware + controllers only (no MoveIt/RViz) |
| `admittance_test.launch.py` | Admittance / F-T testing helper |

---

## Important topics and services (reference)

| Name | Type | Provided by (typical) |
|------|------|------------------------|
| `/joint_command`, `/cartesian_command` | `JsonCommand` | `motion_planner` |
| `/gripper_command` | `JsonCommand` | `motion_planner` (sim/fake) or `catalyst_gripper` (real) |
| `/guide_mode`, `/set_collision_sensitivity`, `/set_eef_bounds` | `JsonCommand` | `motion_planner` (when enabled / xArm connected) |
| `/set_octomap_enabled` | `SetBool` | `motion_planner` |
| `/stop_motion` | `std_srvs/Empty` | `motion_planner` |
| `/world_model` | `std_msgs/String` (JSON) | `catalyst_world_model` |
| `/bt_execute` | `JsonCommand` | `catalyst_bt` / `bt_executor` |
| `/pick`, `/place`, `/explore`, `/place_on_robot`, `/pick_from_robot_container` | `ExecuteTask` action | `catalyst_execute` action servers |
| `/sdk_control`, `/compute_poses` | `JsonCommand` | `catalyst_execute` (run separately) |
| `/scene_command` | `JsonCommand` | `scene_manager` (planning scene objects + ACM) |

### `/scene_command` (`catalyst_interfaces/srv/JsonCommand`)

`command` is a **JSON string**. Set `action` to one of:

| `action` | Required fields | Notes |
|----------|-----------------|--------|
| `add` | `id`, `shape` (`box` / `cylinder` / `sphere` / `mesh`), `position` `[x,y,z]` | For `box`/`cylinder`/`sphere`: `dimensions`; for `mesh`: `mesh_path`, optional `scale` `[sx,sy,sz]`; optional `frame_id`, `orientation` `[qx,qy,qz,qw]` |
| `remove` | `id` | |
| `move` | `id`, `position` | Optional `orientation` |
| `attach` | `id`, `link` | Robot link name |
| `detach` | `id` | |
| `clear` | — | Remove tracked world objects |
| `list` | — | Response includes `objects` array |
| `allow_object_default_collisions` | `object_id`, `allow` (`true` / `false`) | Updates MoveIt **allowed collision matrix** default entry for that world object (e.g. `workspace_box`): when `true`, the object is ignored vs **all** robot links until set back to `false` |

Examples:

```bash
# List collision object ids known to scene_manager
ros2 service call /scene_command catalyst_interfaces/srv/JsonCommand \
  "{command: '{\"action\": \"list\"}'}"
```

```bash
# Allow all links vs the static workspace box (restore with allow: false)
ros2 service call /scene_command catalyst_interfaces/srv/JsonCommand \
  "{command: '{\"action\": \"allow_object_default_collisions\", \"object_id\": \"workspace_box\", \"allow\": true}'}"
```

```bash
# Add a box in link_base (meters, quaternion w last)
ros2 service call /scene_command catalyst_interfaces/srv/JsonCommand \
  "{command: '{\"action\": \"add\", \"id\": \"my_box\", \"shape\": \"box\", \"position\": [0.5, 0.0, 0.1], \"dimensions\": [0.2, 0.2, 0.05]}'}"
```

---

## Package documentation

Each package under `ros2_ws/src/<package>/README.md` describes that package in more detail. Start with:

- [`catalyst_bringup`](ros2_ws/src/catalyst_bringup/README.md) — launch matrix and controllers
- [`catalyst_bt`](ros2_ws/src/catalyst_bt/README.md) — behavior trees and `/bt_execute`
- [`catalyst_execute`](ros2_ws/src/catalyst_execute/README.md) — actions, services, YAML params
- [`catalyst_motion_planner`](ros2_ws/src/catalyst_motion_planner/README.md) — MoveIt JSON services
- [`catalyst_world_model`](ros2_ws/src/catalyst_world_model/README.md) — `/world_model` JSON + health
- [`catalyst_vision`](ros2_ws/src/catalyst_vision/README.md) — AprilTag + point cloud filter
- [`catalyst_interfaces`](ros2_ws/src/catalyst_interfaces/README.md) — `JsonCommand`, `ExecuteTask`

---

## Repository layout

```
catalyst-manipulation/
├── README.md                 # This file
└── ros2_ws/
    └── src/
        ├── catalyst_bringup
        ├── catalyst_bt
        ├── catalyst_description
        ├── catalyst_execute
        ├── catalyst_gazebo
        ├── catalyst_gripper
        ├── catalyst_interfaces
        ├── catalyst_motion_planner
        ├── catalyst_moveit_config
        ├── catalyst_vision
        └── catalyst_world_model
```
