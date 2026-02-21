# catalyst_gripper

ROS 2 driver for the **Dynamixel AX-18A** servo that actuates the Bio Gripper on the Catalyst Manipulator. This package is used **only with real hardware** — in simulation and fake modes, the gripper is controlled by the `bio_gripper_controller` (a `JointTrajectoryController` managed by ros2_control).

## Overview

The package provides two main components:

- **`gripper_controller.py`** — Low-level Dynamixel AX-18A driver using `dynamixel-sdk`. Handles serial communication, mode switching (joint mode for opening, wheel mode for closing with contact detection), and torque-limited holding.
- **`gripper_node.py`** — ROS 2 node that wraps the controller and exposes a service interface plus joint state publishing.

## Node: `gripper_node`

### Service

| Name | Type | Description |
|------|------|-------------|
| `/gripper_command` | `catalyst_interfaces/srv/JsonCommand` | JSON-based command interface |

**Request format** (JSON string in `command` field):

```json
{"action": "open"}
{"action": "open", "position": 100}
{"action": "close"}
{"action": "release"}
```

- **open** — Moves to `open_pos` (or a custom position). Blocks until reached.
- **close** — Wheel-mode closing with contact detection. Returns `success: true` if an object is grasped, then holds with limited torque.
- **release** — Stops motion and disables torque (gripper becomes backdrivable).

**Response format** (JSON string in `response` field):

```json
{"success": true, "message": "gripper opened", "position": 30}
```

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | 30 Hz | `right_finger_joint` position in meters (converted from servo ticks) |
| `/gripper_state` | `std_msgs/String` | 30 Hz | JSON with position (ticks + meters), velocity, torque, grasping status, and state (`open`/`closed`/`holding`/`moving`) |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `config_path` | `<package>/config/gripper_params.yaml` | Path to YAML config file |
| `max_position` | `0.037` | Maximum finger joint position in meters |
| `publish_rate` | `30.0` | Joint state publish rate (Hz) |

## Configuration (`config/gripper_params.yaml`)

Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `devicename` | `/dev/ttyUSB0` | Serial port for the Dynamixel servo |
| `baudrate` | `1000000` | Serial baud rate |
| `dxl_id` | `7` | Servo ID on the Dynamixel bus |
| `pos_min` / `pos_max` | `30` / `720` | Position limits in servo ticks |
| `open_pos` | `30` | Target position when opening |
| `close_speed` | `150` | Motor speed during closing approach |
| `load_threshold` | `80` | Opposing load threshold for contact detection |
| `hold_torque_limit` | `1023` | Max torque while holding an object |

## Hardware Setup

- **Servo**: Dynamixel AX-18A (Protocol 1.0)
- **Connection**: USB-to-Dynamixel adapter on `/dev/ttyUSB0`
- **Docker**: Container must be run with `--privileged` and `-v /dev:/dev` for serial port access

## Usage

This node is launched automatically by `catalyst_bringup` in real hardware mode:

```bash
ros2 launch catalyst_bringup demo.launch.py sim:=false
```

For standalone testing:

```bash
ros2 run catalyst_gripper gripper_node --ros-args -p config_path:=/path/to/gripper_params.yaml
```

To call the service manually:

```bash
ros2 service call /gripper_command catalyst_interfaces/srv/JsonCommand "{command: '{\"action\": \"open\"}'}"
ros2 service call /gripper_command catalyst_interfaces/srv/JsonCommand "{command: '{\"action\": \"close\"}'}"
ros2 service call /gripper_command catalyst_interfaces/srv/JsonCommand "{command: '{\"action\": \"release\"}'}"
```

## Dependencies

- `catalyst_interfaces` — Custom service definition (`JsonCommand.srv`)
- `dynamixel-sdk` — Robotis Dynamixel SDK (`pip3 install dynamixel-sdk`)
