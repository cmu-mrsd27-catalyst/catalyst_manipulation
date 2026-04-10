# catalyst_interfaces

Custom **ROS 2 interface** package for the Catalyst Manipulator: services and actions shared across C++ and Python nodes.

## Interfaces

| Definition | Type | Purpose |
|------------|------|---------|
| `srv/JsonCommand.srv` | Service | `string command` → `string response` (JSON payloads for flexible RPC) |
| `action/ExecuteTask.action` | Action | `string command` goal; `string feedback` / `string response` result (JSON task commands and progress) |

## Usage

- **C++ / Python** clients include generated headers / modules after building:

```bash
colcon build --packages-select catalyst_interfaces
```

Depend in `package.xml` / `CMakeLists.txt` with `catalyst_interfaces` and `rosidl_default_generators` as appropriate.

## See also

- [catalyst_execute README](../catalyst_execute/README.md) — main Python consumers
- [catalyst_bt README](../catalyst_bt/README.md) — `ExecuteTask` client
- [catalyst_motion_planner README](../catalyst_motion_planner/README.md) — `JsonCommand` services
