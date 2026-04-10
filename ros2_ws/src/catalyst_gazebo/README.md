# Catalyst Gazebo Simulation Package

Gz Harmonic simulation environment for the Catalyst Manipulator (xArm-6 + Bio Gripper).

## Package Structure

```
catalyst_gazebo/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── world_objects.yaml      # Dimensions and positions (reference only)
├── launch/
│   └── gazebo.launch.py        # Gz Harmonic + robot + controllers
├── models/
│   ├── robot_stand/            # Vertical stand for robot mounting
│   │   ├── model.config
│   │   └── model.sdf
│   └── table/                  # Work table in front of robot
│       ├── model.config
│       └── model.sdf
└── worlds/
    └── catalyst_workspace.sdf  # Main simulation world
```

---

## Quick Start

### Recommended: unified demo (Gazebo + MoveIt + planners)

From the workspace:

```bash
source install/setup.bash
ros2 launch catalyst_bringup demo.launch.py sim:=gazebo
```

This pulls in Gz Harmonic, robot spawn, controllers, MoveIt, `motion_planner`, `scene_manager`, and `world_model` as configured in **`catalyst_bringup`**.

### Standalone Gazebo launch (package-local)

```bash
colcon build --packages-select catalyst_gazebo
source install/setup.bash
ros2 launch catalyst_gazebo gazebo.launch.py
```

Use this when you only need the sim world + robot without the full bringup stack.

### Verify Everything is Running

```bash
# Check controllers loaded
ros2 control list_controllers

# Check clock is bridged from Gz
ros2 topic echo /clock
```

---

## Launch Arguments

### gazebo.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `catalyst_workspace.sdf` | Path to Gz world file |
| `use_sim_time` | `true` | Use simulation time |

**Example:**
```bash
# Launch with custom world
ros2 launch catalyst_gazebo gazebo.launch.py world:=/path/to/custom.sdf
```

---

## World Environment

### Default Layout

```
     +Y (Left)
      |
      |     +-------------------+
      |     |                   |
      |     |   WORK TABLE      |
      |     |   (0.8 x 0.6m)   |
      |     |                   |
      |     +-------------------+
      |            |
      |        gap | 0.05m
      |            |
      |     +-------------------+
      |     |                   |
      |     |  ROBOT STAND      |
      |     |  (0.5 x 0.5m)    |
      |     |    +-------+     |
      |     |    | ROBOT |     |
      |     |    +-------+     |
      |     |                   |
      |     +-------------------+
      |
------+-----------------------------> +X (Forward)
      |
      Origin (0,0,0)
```

### Dimensions Reference

| Object | Size (X x Y x Z) | Position (X, Y, Z) | Notes |
|--------|-------------------|---------------------|-------|
| Robot Stand | 0.5 x 0.5 x 0.8 m | (0, 0, 0.4) | Center of stand |
| Robot Base | - | (0, 0, 0.8) | Top of stand |
| Work Table | 0.8 x 0.6 x 0.03 m | (0.55, 0, 0.8) | Table top surface |
| Table Legs | r=0.03, h=0.77 m | - | 4 cylindrical legs |

---

## Modifying the Environment

### Change Object Dimensions

#### Robot Stand

Edit `models/robot_stand/model.sdf` — change the `<size>` in both `<collision>` and `<visual>`:

```xml
<size>0.50 0.50 0.80</size>  <!-- X Y Z in meters -->
```

**After changing stand height**, also update:
1. Stand position in world file: `Z = new_height / 2`
2. Robot spawn position in launch file: `Z = new_height`
3. Table position in world file to match height

#### Work Table

Edit `models/table/model.sdf`:

```xml
<!-- Table top dimensions -->
<size>0.80 0.60 0.03</size>  <!-- X(width) Y(depth) Z(thickness) -->

<!-- Leg dimensions (in each leg link) -->
<cylinder>
  <radius>0.03</radius>
  <length>0.77</length>  <!-- height = total_height - top_thickness -->
</cylinder>
```

### Change Object Positions

Edit `worlds/catalyst_workspace.sdf`:

```xml
<!-- Robot Stand Position -->
<include>
  <name>robot_stand</name>
  <uri>model://robot_stand</uri>
  <pose>0.0  0.0  0.4  0    0     0</pose>
</include>

<!-- Work Table Position -->
<include>
  <name>work_table</name>
  <uri>model://table</uri>
  <pose>0.65  0.0  0.8  0    0     1.5708</pose>
</include>
```

### Change Object Colors

Edit the `<material>` section in model SDF files:

```xml
<material>
  <ambient>0.6 0.4 0.2 1</ambient>   <!-- Base color (RGBA 0.0-1.0) -->
  <diffuse>0.6 0.4 0.2 1</diffuse>   <!-- Main color -->
  <specular>0.2 0.2 0.2 1</specular>  <!-- Shininess -->
</material>
```

---

## Adding New Objects

### Simple Shapes

1. Create `models/my_object/model.config` and `models/my_object/model.sdf`
2. Add to world file:

```xml
<include>
  <name>my_object</name>
  <uri>model://my_object</uri>
  <pose>0.5 0.2 0.85 0 0 0</pose>
</include>
```

### Custom Mesh

Place mesh files in the model directory and reference them:

```xml
<visual name="visual">
  <geometry>
    <mesh>
      <uri>model://my_mesh_object/meshes/visual.dae</uri>
    </mesh>
  </geometry>
</visual>
```

---

## Robot Spawn Position

The robot spawn position is set in `launch/gazebo.launch.py`:

```python
spawn_robot = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', 'catalyst_manipulator',
        '-topic', 'robot_description',
        '-x', '0.0',
        '-y', '0.0',
        '-z', '0.8',   # Should match stand height
    ],
)
```

---

## Architecture Notes (Gz Harmonic)

### Clock Bridge

Gz Harmonic does not auto-bridge the `/clock` topic like Gazebo Classic did. This package runs a `ros_gz_bridge` node to bridge `/clock` from Gz to ROS 2:

```
/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

### Physics Engine

The world uses **bullet-featherstone** physics (preferred for Gz Harmonic), passed via:

```
gz_args: '-r -v 3 <world> --physics-engine gz-physics-bullet-featherstone-plugin'
```

### World Plugins

The world file declares three required Gz Harmonic system plugins:

- `gz::sim::systems::Physics` — physics simulation
- `gz::sim::systems::UserCommands` — entity spawn/delete commands
- `gz::sim::systems::SceneBroadcaster` — scene state broadcasting

### ros2_control Integration

The Gazebo ros2_control plugin is loaded via the URDF (not the world file):

```
ros2_control_plugin:=gz_ros2_control/GazeboSimSystem
```

The plugin `gz_ros2_control::GazeboSimROS2ControlPlugin` is added to the robot URDF when `use_gazebo:=true`.

### Model Path

Models are found via the `GZ_SIM_RESOURCE_PATH` environment variable (set automatically by the launch file). The `model://` URI scheme is used in the world file to reference models.

---

## Troubleshooting

### Models not found

Ensure the package is built and sourced:
```bash
colcon build --packages-select catalyst_gazebo
source install/setup.bash
```

The launch file sets `GZ_SIM_RESOURCE_PATH` to include the `models/` directory.

### Robot falls through stand

Check that:
1. Robot spawn Z position matches stand height (0.8 m)
2. Stand is marked as `<static>true</static>`
3. Collision geometry is defined

### Controllers not loading

Check that:
1. `gz_ros2_control` plugin is in URDF (`use_gazebo:=true`)
2. `ros2_control_plugin:=gz_ros2_control/GazeboSimSystem` is passed to xacro
3. Controller config file path is correct in `catalyst_bringup`

### No /clock topic

Verify the `ros_gz_bridge` node is running:
```bash
ros2 node list | grep gz_bridge
```

### Simulation runs slowly

Reduce physics update rate in `worlds/catalyst_workspace.sdf`:
```xml
<physics name="default_physics" type="bullet">
  <real_time_update_rate>500</real_time_update_rate>
</physics>
```

---

## Dependencies

- `ros_gz_sim` — Gz Harmonic ROS 2 integration
- `gz_ros2_control` — ros2_control plugin for Gz Harmonic
- `ros_gz_bridge` — Gz-ROS 2 topic bridge (clock)
- `catalyst_description` — Robot URDF
- `catalyst_bringup` — Controller configuration

---

## See Also

- [catalyst_description README](../catalyst_description/README.md) — Robot URDF
- [catalyst_bringup README](../catalyst_bringup/README.md) — Controller configuration
