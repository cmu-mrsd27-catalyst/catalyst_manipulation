# catalyst_world_model

Single **`world_model`** node that aggregates robot, gripper, vision, and health data into one **`std_msgs/String`** JSON message on **`/world_model`**.

## Published topic

| Topic | Type | Content (high level) |
|-------|------|------------------------|
| `/world_model` | `std_msgs/String` | JSON: `timestamp`, `joint_states`, `tcp_pose`, `gripper`, `apriltags`, `robot_status`, `robot_phase`, **`system_health`** |

Rate: **10 Hz** (timer).

## Subscriptions (typical)

Includes **`/joint_states`**, **`/gripper_state`**, **`/detections`** (AprilTag), **`/xarm/robot_states`** (if available), **`/robot_phase`**, camera image, F/T wrench, filtered point cloud — see `world_model.py` for the authoritative list.

## Parameters

Defaults are merged from **`config/world_model.yaml`** (`world_model_node.ros__parameters`) when installed under share, with code fallbacks in `_WORLD_MODEL_PARAM_DEFAULTS`.

Health checks (camera, gripper, F/T, controllers, robot errors, octomap stream, collision force, etc.) can be toggled via `health.check_*` parameters. Failures are **debounced**: a check must fail **5 times in a row** (at publish rate) before it marks that check failed, except for **stale-data** semantics handled inside each check. Several checks are **suppressed** during specific **`robot_phase`** values (SDK, teach, force placement, etc.).

## Consumers

- **Behavior tree** **`SystemHealthy`** node
- **Explore** action / **`TagExplorer`** (fresh tag read after joint moves)

## Build / run

```bash
colcon build --packages-select catalyst_world_model
ros2 run catalyst_world_model world_model
```

## See also

- [catalyst_bt README](../catalyst_bt/README.md)
- [Repository README](../../../README.md)
