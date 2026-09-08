# catalyst_execute

Python **ament** package: high-level manipulation **action servers**, supporting **services**, calibration helpers, and test / utility scripts for the Catalyst stack.

## Configuration

Default parameters are loaded from **`config/catalyst_execute_params.yaml`** (installed to `share/catalyst_execute/config/`). Code uses **`catalyst_execute.utils.execute_config.section(...)`** so nodes share service names, timeouts, exploration joint sweep, action names, etc. ROS parameters and launch **`--ros-args -p`** still override declared defaults where used.

## Action servers (`ExecuteTask`)

| Command | Action name (default, from YAML) | Role |
|---------|----------------------------------|------|
| `explore_action_server` | `/explore` | Joint sweep + `/world_model` → `tag_pose` |
| `pick_action_server` | `/pick` | Full pick sequence (MoveIt + guide + gripper + octomap) |
| `place_action_server` | `/place` | Place sequence (MoveIt + SDK force phases) |

Goals and results are **JSON strings** in the action definition. Explore result is merged into pick/place by the behavior tree **`overrides`** port.

## Long-running services

| Command | Service (default) | Role |
|---------|-------------------|------|
| `compute_poses_service` | `/compute_poses` | Tag pose → pick/place/approach poses; corner correction helper |
| `sdk_control_service` | `/sdk_control` | xArm SDK (connect, velocity modes, cleanup, controller restart) |
| `explore_tag_service` | `/explore_tag` | JsonCommand request/response wrapper around **`TagExplorer`** (alternate to `/explore` action) |

These are **not** started by `demo.launch.py`; run alongside bringup when using pick/place actions.

## Other entry points (selection)

| Command | Purpose |
|---------|---------|
| `guide_mode` | Interactive teach / joint / Cartesian CLI |
| `test_torque_placement`, `test_eef_bounds_viz`, `sdk_admittance` | Development and experiments |

Use `ros2 pkg executables catalyst_execute` for the full list.

## Dependencies

Declared in `package.xml`: `rclpy`, `catalyst_interfaces`, geometry/tf/MoveIt-related msgs as needed; **`python3-yaml`**, **`ament_index_python`** for config resolution.

## See also

- [catalyst_bt README](../catalyst_bt/README.md) — orchestration
- [catalyst_motion_planner README](../catalyst_motion_planner/README.md) — `/joint_command`, `/cartesian_command`
- [Repository README](../../../README.md)
