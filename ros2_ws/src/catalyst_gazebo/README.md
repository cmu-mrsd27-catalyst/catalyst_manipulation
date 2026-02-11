# Catalyst Gazebo Simulation Package

Gazebo Classic simulation environment for the Catalyst Manipulator (xArm-6 + Bio Gripper).

## Package Structure

```
catalyst_gazebo/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── world_objects.yaml      # Dimensions and positions (reference only)
├── launch/
│   ├── gazebo.launch.py        # Gazebo + robot + controllers
│   └── gazebo_moveit.launch.py # Full setup with MoveIt + RViz
├── models/
│   ├── robot_stand/            # Vertical stand for robot mounting
│   │   ├── model.config
│   │   └── model.sdf
│   └── table/                  # Work table in front of robot
│       ├── model.config
│       └── model.sdf
└── worlds/
    └── catalyst_workspace.world # Main simulation world
```

---

## Quick Start

### Launch Gazebo with Robot

```bash
# Build the package
cd ~/catalyst-manipulation/ros2_ws
colcon build --packages-select catalyst_gazebo
source install/setup.bash

# Launch Gazebo simulation
ros2 launch catalyst_gazebo gazebo.launch.py
```

### Launch Full Setup (Gazebo + MoveIt + RViz)

**Recommended:** Use the unified launch file in `catalyst_bringup`:

```bash
ros2 launch catalyst_bringup demo.launch.py sim:=gazebo
```

Or use the standalone launch file in this package:

```bash
ros2 launch catalyst_gazebo gazebo_moveit.launch.py
```

---

## Launch Arguments

### gazebo.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `catalyst_workspace.world` | Path to Gazebo world file |
| `paused` | `false` | Start Gazebo paused |
| `gui` | `true` | Show Gazebo GUI |
| `use_sim_time` | `true` | Use simulation time |

### gazebo_moveit.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `catalyst_workspace.world` | Path to Gazebo world file |
| `paused` | `false` | Start Gazebo paused |
| `gui` | `true` | Show Gazebo GUI |
| `rviz` | `true` | Launch RViz |
| `use_sim_time` | `true` | Use simulation time |

**Example:**
```bash
# Launch without Gazebo GUI (headless)
ros2 launch catalyst_gazebo gazebo.launch.py gui:=false

# Launch with custom world
ros2 launch catalyst_gazebo gazebo.launch.py world:=/path/to/custom.world
```

---

## World Environment

### Default Layout

```
     +Y (Left)
      │
      │     ┌─────────────────┐
      │     │                 │
      │     │   WORK TABLE    │
      │     │   (0.8 x 0.6m)  │
      │     │                 │
      │     └─────────────────┘
      │            │
      │        gap │ 0.05m
      │            │
      │     ┌─────────────────┐
      │     │                 │
      │     │  ROBOT STAND    │
      │     │  (0.5 x 0.5m)   │
      │     │    ┌─────┐      │
      │     │    │ROBOT│      │
      │     │    └─────┘      │
      │     │                 │
      │     └─────────────────┘
      │
──────┼──────────────────────────→ +X (Forward)
      │
      Origin (0,0,0)
```

### Dimensions Reference

| Object | Size (X × Y × Z) | Position (X, Y, Z) | Notes |
|--------|------------------|---------------------|-------|
| Robot Stand | 0.5 × 0.5 × 0.8 m | (0, 0, 0.4) | Center of stand |
| Robot Base | - | (0, 0, 0.8) | Top of stand |
| Work Table | 0.8 × 0.6 × 0.03 m | (0.55, 0, 0.8) | Table top surface |
| Table Legs | r=0.03, h=0.77 m | - | 4 cylindrical legs |

---

## Modifying the Environment

### 1. Change Object Dimensions

#### Robot Stand

Edit `models/robot_stand/model.sdf`:

```xml
<collision name="stand_collision">
  <geometry>
    <box>
      <!-- Change these values (X Y Z in meters) -->
      <size>0.50 0.50 0.80</size>
    </box>
  </geometry>
</collision>

<visual name="stand_visual">
  <geometry>
    <box>
      <!-- MUST match collision size -->
      <size>0.50 0.50 0.80</size>
    </box>
  </geometry>
</visual>
```

**After changing stand height:**
1. Update stand position in world file: `Z = new_height / 2`
2. Update robot spawn position in launch file: `Z = new_height`
3. Update table position in world file to match height

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

**Leg position calculation:**
```
X offset = table_width/2 - 0.05
Y offset = table_depth/2 - 0.05
Z offset = -(top_thickness + leg_height/2)
```

### 2. Change Object Positions

Edit `worlds/catalyst_workspace.world`:

```xml
<!-- Robot Stand Position -->
<include>
  <name>robot_stand</name>
  <uri>model://robot_stand</uri>
  <!-- X    Y    Z    Roll Pitch Yaw -->
  <pose>0.0  0.0  0.4  0    0     0</pose>
</include>

<!-- Work Table Position -->
<include>
  <name>work_table</name>
  <uri>model://table</uri>
  <pose>0.55  0.0  0.8  0    0     0</pose>
</include>
```

**Position conventions:**
- Stand Z = stand_height / 2 (so bottom touches ground)
- Table Z = desired table top height
- Gap = table_X - (stand_size_X/2 + table_size_Y/2)

### 3. Change Object Colors

Edit the `<material>` section in model SDF files:

```xml
<material>
  <!-- RGBA values (0.0 to 1.0) -->
  <ambient>0.6 0.4 0.2 1</ambient>   <!-- Base color -->
  <diffuse>0.6 0.4 0.2 1</diffuse>   <!-- Main color -->
  <specular>0.2 0.2 0.2 1</specular> <!-- Shininess -->
</material>
```

**Common colors:**
| Color | R | G | B |
|-------|---|---|---|
| Dark Gray | 0.3 | 0.3 | 0.3 |
| Wood Brown | 0.6 | 0.4 | 0.2 |
| White | 1.0 | 1.0 | 1.0 |
| Red | 1.0 | 0.0 | 0.0 |
| Blue | 0.0 | 0.0 | 1.0 |

---

## Adding New Objects

### Option A: Simple Shapes (No Mesh)

1. Create a new model directory:
```bash
mkdir -p models/my_object
```

2. Create `model.config`:
```xml
<?xml version="1.0"?>
<model>
  <name>my_object</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>My custom object</description>
</model>
```

3. Create `model.sdf` (example: a box):
```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="my_object">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box><size>0.1 0.1 0.1</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.1 0.1 0.1</size></box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

4. Add to world file:
```xml
<include>
  <name>my_object</name>
  <uri>model://my_object</uri>
  <pose>0.5 0.2 0.85 0 0 0</pose>
</include>
```

### Option B: Custom Mesh

1. Place mesh files in the model directory:
```
models/my_mesh_object/
├── model.config
├── model.sdf
└── meshes/
    ├── visual.dae    # For rendering (can be detailed)
    └── collision.stl # For physics (should be simplified)
```

2. Reference mesh in `model.sdf`:
```xml
<visual name="visual">
  <geometry>
    <mesh>
      <uri>model://my_mesh_object/meshes/visual.dae</uri>
      <scale>1 1 1</scale>
    </mesh>
  </geometry>
</visual>

<collision name="collision">
  <geometry>
    <mesh>
      <uri>model://my_mesh_object/meshes/collision.stl</uri>
      <scale>1 1 1</scale>
    </mesh>
  </geometry>
</collision>
```

### Option C: Use Gazebo Model Database

```xml
<include>
  <uri>model://beer</uri>
  <name>beer_can</name>
  <pose>0.5 0 0.85 0 0 0</pose>
</include>
```

Browse available models at: http://models.gazebosim.org/

---

## Removing Objects

Simply delete or comment out the `<include>` block in the world file:

```xml
<!-- Commented out - table removed
<include>
  <name>work_table</name>
  <uri>model://table</uri>
  <pose>0.55 0.0 0.8 0 0 0</pose>
</include>
-->
```

---

## Robot Spawn Position

The robot spawn position is defined in the launch files.

Edit `launch/gazebo.launch.py`:

```python
spawn_robot = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'catalyst_manipulator',
        '-topic', 'robot_description',
        '-x', '0.0',   # X position
        '-y', '0.0',   # Y position
        '-z', '0.8',   # Z position (should match stand height)
        '-R', '0.0',   # Roll
        '-P', '0.0',   # Pitch
        '-Y', '0.0',   # Yaw (rotation around Z)
    ],
)
```

---

## Gazebo Plugins Used

### 1. gazebo_ros2_control

Bridges Gazebo physics with ros2_control controllers.

**Location:** Added to robot URDF via `use_gazebo:=true` argument

```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find catalyst_bringup)/config/ros2_controllers.yaml</parameters>
  </plugin>
</gazebo>
```

### 2. Mimic Joint Plugin

Synchronizes left finger with right finger in Gazebo.

```xml
<plugin name="bio_gripper_mimic_joint_plugin" filename="libgazebo_mimic_joint_plugin.so">
  <joint>right_finger_joint</joint>
  <mimicJoint>left_finger_joint</mimicJoint>
  <multiplier>-1.0</multiplier>
</plugin>
```

### 3. Grasp Fix Plugin

Helps the gripper hold objects in simulation.

```xml
<plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
  <arm>
    <arm_name>bio_gripper</arm_name>
    <palm_link>bio_gripper_base_link</palm_link>
    <gripper_link>bio_gripper_left_finger</gripper_link>
    <gripper_link>bio_gripper_right_finger</gripper_link>
  </arm>
</plugin>
```

---

## Troubleshooting

### Models not found

Ensure the package is built and sourced:
```bash
colcon build --packages-select catalyst_gazebo
source install/setup.bash
```

The model path is set via `package.xml`:
```xml
<gazebo_ros gazebo_model_path="${prefix}/models"/>
```

### Robot falls through stand

Check that:
1. Robot spawn Z position matches stand height
2. Stand is marked as `<static>true</static>`
3. Collision geometry is properly defined

### Controllers not loading

Check that:
1. `gazebo_ros2_control` plugin is in URDF (`use_gazebo:=true`)
2. `ros2_control_plugin:=gazebo_ros2_control/GazeboSystem` is passed to xacro
3. Controller config file path is correct

### Simulation runs slowly

Try:
```bash
# Reduce physics update rate in world file
<physics type="ode">
  <real_time_update_rate>500</real_time_update_rate>  # Default 1000
</physics>
```

---

## Dependencies

- `gazebo_ros` - Gazebo ROS2 integration
- `gazebo_ros2_control` - ros2_control plugin for Gazebo
- `description` - Robot URDF
- `catalyst_bringup` - Controller configuration
- `moveit_config` - MoveIt configuration (for gazebo_moveit.launch.py)

---

## See Also

- [catalyst_bringup README](../catalyst_bringup/README.md) - Fake hardware demo
- [moveit_config README](../moveit_config/README.md) - MoveIt configuration
- [description README](../description/README.md) - Robot URDF
