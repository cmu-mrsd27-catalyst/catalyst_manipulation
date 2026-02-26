# catalyst_motion_planner

Motion planning and scene management for the Catalyst Manipulator. Wraps MoveIt 2's `MoveGroupInterface` behind JSON-based ROS 2 services so any client (Python scripts, web UI, etc.) can command the arm without linking MoveIt directly.

## Nodes

### `motion_planner`

Exposes three (optionally four) services for arm control. All use the `catalyst_interfaces/srv/JsonCommand` interface — send a JSON string in `command`, get a JSON string back in `response`.

### `scene_manager`

Manages the MoveIt planning scene (collision objects). Loads world geometry from a YAML file at startup and exposes a `/scene_command` service for runtime additions/removals.

## Services

### `/joint_command`

Move the arm in joint space.

**By joint angles (degrees):**
```bash
ros2 service call /joint_command catalyst_interfaces/srv/JsonCommand \
  '{"command": "{\"joints\": [0, -30, 0, 60, 0, 30], \"speed\": 0.5}"}'
```

**By named pose (defined in SRDF):**
```bash
ros2 service call /joint_command catalyst_interfaces/srv/JsonCommand \
  '{"command": "{\"pose\": \"home\"}"}'
```

| Field   | Type       | Required | Description                              |
|---------|------------|----------|------------------------------------------|
| `joints`| float[6]   | *        | Joint angles in degrees                  |
| `pose`  | string     | *        | Named pose from SRDF (e.g. `"home"`)     |
| `speed` | float      | no       | Velocity/acceleration scaling (0.01–1.0, default 1.0) |

*One of `joints` or `pose` is required.

---

### `/cartesian_command`

Move the arm to a Cartesian pose (position + quaternion) in the `link_base` frame.

```bash
ros2 service call /cartesian_command catalyst_interfaces/srv/JsonCommand \
  '{"command": "{\"x\": 0.3, \"y\": 0.0, \"z\": 0.5, \"qx\": 0, \"qy\": 0.707, \"qz\": 0, \"qw\": 0.707}"}'
```

| Field              | Type   | Required | Description                                    |
|--------------------|--------|----------|------------------------------------------------|
| `x`, `y`, `z`      | float  | yes      | Target position in meters                      |
| `qx`, `qy`, `qz`, `qw` | float | yes  | Target orientation as quaternion                |
| `speed`            | float  | no       | Velocity/acceleration scaling (0.01–1.0)        |
| `straight_line`    | bool   | no       | Use `computeCartesianPath` for linear motion    |
| `keep_orientation` | bool   | no       | Constrain X/Y rotation along the path           |

IK is solved with TRAC-IK. When `keep_orientation` is set, the planner retries with progressively relaxed tolerances (up to 5 attempts).

---

### `/gripper_command` (sim only)

Open/close the bio gripper via MoveIt. Only available when `enable_gripper_service` is true (sim modes). On real hardware the `catalyst_gripper` package drives the gripper directly.

```bash
ros2 service call /gripper_command catalyst_interfaces/srv/JsonCommand \
  '{"command": "{\"action\": \"open\"}"}'
```

| Field    | Type   | Required | Description               |
|----------|--------|----------|---------------------------|
| `action` | string | yes      | `"open"` or `"close"`     |

---

### `/guide_mode` (real hardware only)

Toggle teach/guide mode so the arm can be moved by hand. Only available when `enable_guide_mode` is true. Uses the xArm SDK to switch arm modes directly.

```bash
# Enable teach mode
ros2 service call /guide_mode catalyst_interfaces/srv/JsonCommand \
  '{"command": "{\"action\": \"enable\"}"}'

# Disable teach mode (return to servo)
ros2 service call /guide_mode catalyst_interfaces/srv/JsonCommand \
  '{"command": "{\"action\": \"disable\"}"}'

# Query current mode/state
ros2 service call /guide_mode catalyst_interfaces/srv/JsonCommand \
  '{"command": "{\"action\": \"status\"}"}'
```

| Field         | Type   | Required | Description                                  |
|---------------|--------|----------|----------------------------------------------|
| `action`      | string | yes      | `"enable"`, `"disable"`, or `"status"`       |
| `sensitivity` | int    | no       | Teach sensitivity 1–5 (default 3). Only used with `"enable"`. Higher = easier to move. |

**Enable sequence:** `set_state(STOP)` → 2 s wait → `set_mode(TEACH)` + `set_state(START)` → poll until confirmed
**Disable sequence:** `clean_error` → `set_mode(SERVO)` + `set_state(START)` → poll until confirmed

The service is **blocking** — it polls the arm's reported mode and state (every 100 ms, up to 5 s timeout) and only returns `success: true` once the arm confirms it has reached the requested mode. If confirmation times out, the response returns `success: false`. This guarantees the caller can rely on the arm being in the correct mode before proceeding.

Response includes `mode` and `state` integers from the arm. Teach mode is automatically disabled on node shutdown.

---

### `/scene_command`

Add or remove collision objects from the MoveIt planning scene (provided by the `scene_manager` node). World objects are also auto-loaded from a YAML file at startup.

## Parameters

### motion_planner

| Parameter               | Type   | Default          | Description                                  |
|-------------------------|--------|------------------|----------------------------------------------|
| `enable_gripper_service`| bool   | `true`           | Enable `/gripper_command` (set false for real HW) |
| `enable_guide_mode`     | bool   | `false`          | Enable `/guide_mode` (set true for real HW)  |
| `robot_ip`              | string | `192.168.1.212`  | xArm IP address (only used when guide mode enabled) |

Standard MoveIt parameters (`robot_description`, `robot_description_semantic`, `robot_description_kinematics`) are also required.

### scene_manager

| Parameter            | Type   | Default | Description                                     |
|----------------------|--------|---------|-------------------------------------------------|
| `world_objects_yaml` | string | `""`    | Path to YAML file defining static collision objects |
| `base_frame_z`       | float  | `0.80`  | Z-offset of `link_base` above world origin      |

## Response Format

All services return a JSON object with at minimum:

```json
{
  "success": true,
  "message": "Description of result"
}
```

The `/guide_mode` service additionally returns `mode` and `state` integers.

## Architecture

The node creates two internal rclcpp nodes sharing a `MultiThreadedExecutor`:

- **`motion_planner`** — owns the ROS 2 services
- **`motion_planner_moveit`** — owns the `MoveGroupInterface` (isolated to avoid executor conflicts with MoveIt's internal action clients)

## Build

```bash
colcon build --packages-select catalyst_motion_planner
```

Requires `xarm_sdk` to be built first (for the guide mode feature):

```bash
colcon build --packages-select xarm_sdk catalyst_motion_planner
```
