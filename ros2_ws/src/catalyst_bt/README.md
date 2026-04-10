# catalyst_bt — Behavior tree orchestration

**BehaviorTree.CPP v4** package that orchestrates high-level pick / place / explore flows for the Catalyst Manipulator. Low-level motion and grasp sequences live in **`catalyst_execute`** action servers; this package wires them together and monitors **`/world_model`** health.

## Architecture

```
catalyst_bt (bt_executor)
  ├─ TaskAction      → ROS 2 actions /pick, /place, /explore (ExecuteTask)
  ├─ JsonService     → /joint_command (e.g. home)
  └─ SystemHealthy   → condition on /world_model JSON (system_health.healthy)
```

Trees are loaded from **`trees/orchestration.xml`** at runtime (installed under `share/catalyst_bt/trees/`). The executor node does **not** take a `tree_file` parameter; to change trees, edit that file (or the install copy) and rebuild/reinstall.

## Running the executor

```bash
source install/setup.bash
ros2 launch catalyst_bt pick_and_place_bt.launch.py
```

Equivalent:

```bash
ros2 run catalyst_bt bt_executor
```

This exposes:

| Interface | Type | Purpose |
|-----------|------|---------|
| `/bt_execute` | `catalyst_interfaces/srv/JsonCommand` | Start a named task (see below) |

### Invoking a task

```bash
ros2 service call /bt_execute catalyst_interfaces/srv/JsonCommand \
  "{command: '{\"task\": \"pick_only\"}'}"
```

**Tasks** (must match `orchestration.xml` tree IDs):

| `task` value | Behavior tree ID |
|--------------|------------------|
| `pick_only` | `MainPickOnly` |
| `place_only` | `MainPlaceOnly` |
| `pick_place` | `MainPickPlace` |
| `place_pick` | `MainPlacePick` |

Each main tree uses a **`ReactiveSequence`**: **`SystemHealthy`** is ticked every cycle; if it fails, the sequence fails and running **`TaskAction`** nodes are halted (see **`TaskActionNode::onHalted`**): **`/stop_motion`** plus action cancel.

## Typical full stack (with bringup + execute)

1. **`ros2 launch catalyst_bringup demo.launch.py`** (or `sim:=fake` / `sim:=gazebo`)
2. **`ros2 run catalyst_execute sdk_control_service`**
3. **`ros2 run catalyst_execute compute_poses_service`**
4. **`ros2 run catalyst_execute explore_action_server`**
5. **`ros2 run catalyst_execute pick_action_server`**
6. **`ros2 run catalyst_execute place_action_server`**
7. **`ros2 launch catalyst_bt pick_and_place_bt.launch.py`**
8. Call **`/bt_execute`** as above.

Steps 2–6 can start in any order before step 7; all must be up before running a task that needs them.

## Tree behavior (summary)

- **`InitMinimal`**: home via `/joint_command`.
- **Main trees**: `SystemHealthy` → `Sequence` of Explore → Pick and/or Place → home (pick/place start from the arm pose at end of sweep, not an extra home between explore and pick/place).
- **Explore**: `/explore` action; result JSON (e.g. `tag_pose`) is merged into pick/place goals via blackboard **`overrides="{explore_result}"`**.

## Registered node types

| XML node | C++ type | Role |
|----------|----------|------|
| `JsonService` | `JsonServiceNode` | Synchronous `JsonCommand` service client |
| `SetBoolService` | `SetBoolServiceNode` | `SetBool` client |
| `EmptyService` | `EmptyServiceNode` | `Empty` client |
| `TaskAction` | `TaskActionNode` | `ExecuteTask` action client (async) |
| `SystemHealthy` | `SystemHealthyNode` | Parses `/world_model`; fails if stale or unhealthy |

`SystemHealthy` ports: `topic` (default `/world_model`), `timeout_sec` (default `2.0`).

## Safety / health (high level)

`catalyst_world_model` publishes **`system_health`** inside the world JSON. Individual checks (camera, gripper, F/T, controllers, robot state, octomap input, collision force, etc.) use **consecutive-failure debouncing** and **phase-aware suppression** during SDK / teach / place-force phases. See **`catalyst_world_model`** README.

**Stale `/world_model`** (no message within `timeout_sec`) causes **`SystemHealthy`** to fail immediately (separate from per-check debounce).

## Build

```bash
colcon build --packages-select catalyst_bt
```

Depends on `behaviortree_cpp`, `catalyst_interfaces`, `rclcpp`, `std_srvs`, `std_msgs`, `nlohmann_json`.

## See also

- [Repository README](../../../README.md)
- [catalyst_execute README](../catalyst_execute/README.md)
- [catalyst_world_model README](../catalyst_world_model/README.md)
