# catalyst_motion_planner

ROS 2 package providing motion planning and scene management services for the Catalyst Manipulator (xArm6 + BIO gripper).

## Nodes

### `motion_planner`

MoveIt-based motion planning node exposing arm and gripper control via ROS 2 services.

**Services:**

| Service | Type | Description |
|---------|------|-------------|
| `/joint_command` | `catalyst_interfaces/srv/GripperCommand` | Joint-space arm control |
| `/cartesian_command` | `catalyst_interfaces/srv/GripperCommand` | Cartesian arm control (IK + path planning) |
| `/gripper_command` | `catalyst_interfaces/srv/GripperCommand` | Gripper open/close (sim only) |

---

### `scene_manager`

Manages MoveIt planning scene collision objects. Automatically loads static world geometry (table, robot stand, ground) on startup and provides a service for dynamic object manipulation.

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `world_objects_yaml` | string | `""` | Path to `world_objects.yaml` for auto-loading static objects |
| `base_frame_z` | double | `0.80` | Height of `link_base` in world frame (meters) |

**Auto-loaded objects** (from `world_objects.yaml`):

| Object ID | Shape | Dimensions | Position (link_base frame) |
|-----------|-------|------------|---------------------------|
| `robot_stand` | box | 0.5 x 0.5 x 0.8 m | (0, 0, -0.4) |
| `table_top` | box | 0.8 x 0.6 x 0.03 m | (0.55, 0, -0.015) |
| `ground_plane` | box | 10 x 10 x 0.01 m | (0, 0, -0.805) |

All positions from the YAML (world frame) are converted to `link_base` frame by subtracting `base_frame_z` from z.

**Service:** `/scene_command` (`catalyst_interfaces/srv/GripperCommand`)

The `command` field is a JSON string with an `action` key. Supported actions:

#### `add` — Add a primitive shape

```json
{
  "action": "add",
  "id": "my_box",
  "shape": "box",
  "dimensions": [0.1, 0.1, 0.1],
  "position": [0.3, 0.0, 0.0],
  "orientation": [0, 0, 0, 1],
  "frame_id": "link_base"
}
```

- `shape`: `box` (dims: [x, y, z]), `cylinder` (dims: [height, radius]), or `sphere` (dims: [radius])
- `orientation`: optional, defaults to `[0, 0, 0, 1]` (qx, qy, qz, qw)
- `frame_id`: optional, defaults to `link_base`

#### `add` — Add a mesh

```json
{
  "action": "add",
  "id": "my_mesh",
  "shape": "mesh",
  "mesh_path": "package://my_package/meshes/object.stl",
  "scale": [1.0, 1.0, 1.0],
  "position": [0.3, 0.0, 0.0],
  "orientation": [0, 0, 0, 1],
  "frame_id": "link_base"
}
```

- `mesh_path`: supports `package://` and `file://` URIs (STL/DAE)
- `scale`: optional, defaults to `[1, 1, 1]`

#### `remove` — Remove an object

```json
{"action": "remove", "id": "my_box"}
```

#### `move` — Move an existing object

```json
{
  "action": "move",
  "id": "my_box",
  "position": [0.5, 0.0, 0.0],
  "orientation": [0, 0, 0, 1]
}
```

- `orientation`: optional, defaults to `[0, 0, 0, 1]`

#### `attach` — Attach object to a robot link

```json
{"action": "attach", "id": "my_box", "link": "link_tcp"}
```

#### `detach` — Detach object from robot

```json
{"action": "detach", "id": "my_box"}
```

#### `clear` — Remove all known objects

```json
{"action": "clear"}
```

#### `list` — List all known object IDs

```json
{"action": "list"}
```

Returns: `{"success": true, "objects": ["robot_stand", "table_top", "ground_plane"]}`

## Example Usage

```bash
# Add a test box
ros2 service call /scene_command catalyst_interfaces/srv/GripperCommand \
  "{command: '{\"action\":\"add\",\"id\":\"test_box\",\"shape\":\"box\",\"dimensions\":[0.1,0.1,0.1],\"position\":[0.3,0.0,0.0]}'}"

# List all objects
ros2 service call /scene_command catalyst_interfaces/srv/GripperCommand \
  "{command: '{\"action\":\"list\"}'}"

# Move the box
ros2 service call /scene_command catalyst_interfaces/srv/GripperCommand \
  "{command: '{\"action\":\"move\",\"id\":\"test_box\",\"position\":[0.4,0.1,0.0]}'}"

# Remove the box
ros2 service call /scene_command catalyst_interfaces/srv/GripperCommand \
  "{command: '{\"action\":\"remove\",\"id\":\"test_box\"}'}"
```

## Dependencies

- `moveit_ros_planning_interface`
- `geometric_shapes`
- `shape_msgs`
- `moveit_msgs`
- `catalyst_interfaces`
- `nlohmann-json-dev`
- `yaml-cpp` (transitive)
