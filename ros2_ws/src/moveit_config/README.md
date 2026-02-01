# MoveIt Configuration for Catalyst Manipulator

MoveIt2 configuration package for the Catalyst Manipulator (xArm-6 + Bio Gripper).

## Package Structure

```
moveit_config/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── kinematics.yaml
└── srdf/
    └── catalyst_manipulator.srdf
```

## SRDF File Overview

The SRDF (Semantic Robot Description Format) file complements the URDF by adding semantic information that MoveIt needs for motion planning.

Location: `srdf/catalyst_manipulator.srdf`

Reference: Aligned with [xarm_ros2 conventions](https://github.com/xArm-Developer/xarm_ros2/blob/humble/xarm_moveit_config/srdf/_xarm6_macro.srdf.xacro)

---

## SRDF Components

### 1. Virtual Joint

```xml
<virtual_joint name="world_joint" type="fixed" parent_frame="world" child_link="link_base"/>
```

| Attribute | Value | Description |
|-----------|-------|-------------|
| name | `world_joint` | Joint name for reference |
| type | `fixed` | Robot base is stationary |
| parent_frame | `world` | Global reference frame |
| child_link | `link_base` | Robot's base link |

Connects the robot's base to the world frame. Required for MoveIt to know how the robot is positioned in space.

---

### 2. Planning Groups

#### xarm6 (Arm)

```xml
<group name="xarm6">
  <chain base_link="link_base" tip_link="link_eef"/>
</group>
```

- **Type**: Kinematic chain
- **DOF**: 6 (joint1 through joint6)
- **Base**: `link_base`
- **Tip**: `link_eef` (end effector flange)

#### bio_gripper (Gripper)

```xml
<group name="bio_gripper">
  <joint name="right_finger_joint"/>
  <joint name="left_finger_joint"/>
</group>
```

- **Type**: Joint group
- **Joints**: 2 prismatic finger joints
- **Motion**: Linear open/close

---

### 3. Group States (Named Poses)

Pre-defined poses that can be commanded via MoveIt:

#### Arm Poses (xarm6)

| State | Description | Joint Values |
|-------|-------------|--------------|
| `home` | All joints at zero | All 0 rad |
| `hold_up` | End effector pointing up | joint5 = -1.5708 rad (-90°) |

#### Gripper Poses (bio_gripper)

| State | Description | Values |
|-------|-------------|--------|
| `open` | Fingers fully extended | left = -0.04m, right = 0.04m |
| `close` | Fingers closed | left = 0m, right = 0m |

---

### 4. End Effector

```xml
<end_effector name="bio_gripper" parent_link="link_tcp" group="bio_gripper" parent_group="xarm6"/>
```

| Attribute | Value | Description |
|-----------|-------|-------------|
| name | `bio_gripper` | End effector identifier |
| parent_link | `link_tcp` | Tool Center Point (grasp location) |
| group | `bio_gripper` | Planning group for the gripper |
| parent_group | `xarm6` | Arm that carries this gripper |

---

### 5. Passive Joints

```xml
<passive_joint name="left_finger_joint"/>
```

The `left_finger_joint` is marked as passive because it mimics the `right_finger_joint` (defined in URDF with mimic tag). MoveIt will not plan for this joint independently.

---

### 6. Disabled Collisions

Collision pairs that MoveIt should skip during planning:

#### Collision Reasons

| Reason | Meaning |
|--------|---------|
| `Adjacent` | Links connected by a joint (always in contact) |
| `Never` | Links geometrically too far apart to ever collide |

#### Arm Chain - Adjacent Links (7 pairs)

| Link 1 | Link 2 |
|--------|--------|
| link1 | link2 |
| link1 | link_base |
| link2 | link3 |
| link3 | link4 |
| link4 | link5 |
| link5 | link6 |
| link6 | link_eef |

#### Arm Chain - Never Collide (6 pairs)

| Link 1 | Link 2 |
|--------|--------|
| link1 | link3 |
| link2 | link4 |
| link2 | link_base |
| link3 | link5 |
| link3 | link6 |
| link4 | link6 |

#### Bio Gripper Internal (3 pairs)

| Link 1 | Link 2 | Reason |
|--------|--------|--------|
| bio_gripper_left_finger | bio_gripper_base_link | Adjacent |
| bio_gripper_right_finger | bio_gripper_base_link | Adjacent |
| bio_gripper_left_finger | bio_gripper_right_finger | Never |

#### Bio Gripper to Arm (4 pairs)

| Link 1 | Link 2 | Reason |
|--------|--------|--------|
| link6 | bio_gripper_base_link | Adjacent |
| link5 | bio_gripper_base_link | Never |
| link_eef | bio_gripper_base_link | Adjacent |
| link_eef | bio_gripper_left_finger | Never |

#### Bio Gripper Fingers to Arm (6 pairs)

| Finger | Arm Link | Reason |
|--------|----------|--------|
| bio_gripper_left_finger | link4 | Never |
| bio_gripper_left_finger | link5 | Never |
| bio_gripper_left_finger | link6 | Never |
| bio_gripper_right_finger | link4 | Never |
| bio_gripper_right_finger | link5 | Never |
| bio_gripper_right_finger | link6 | Never |

**Total: 22 disabled collision pairs**

---

## Kinematics Configuration

Location: `config/kinematics.yaml`

Reference: Aligned with [xarm_ros2 kinematics](https://github.com/xArm-Developer/xarm_ros2/blob/humble/xarm_moveit_config/config/xarm6/kinematics.yaml)

### Configuration

```yaml
xarm6:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3
```

### Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `kinematics_solver` | `kdl_kinematics_plugin/KDLKinematicsPlugin` | KDL numerical IK solver |
| `kinematics_solver_search_resolution` | `0.005` | Joint space discretization (radians) |
| `kinematics_solver_timeout` | `0.005` | Max time per IK attempt (5ms) |
| `kinematics_solver_attempts` | `3` | Number of IK solve attempts |

### Notes

- Only the `xarm6` group requires IK configuration
- The `bio_gripper` group does not need IK (simple open/close motion)
- Alternative solver `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` is available but commented out

---

## Dependencies

- `description` - Robot URDF/Xacro files
- `moveit_ros_move_group` - MoveIt move_group node
- `moveit_ros_planning_interface` - MoveIt planning interface
- `moveit_kinematics` - Kinematics solvers
- `moveit_planners_ompl` - OMPL motion planners
- `moveit_ros_visualization` - RViz MoveIt plugins
- `robot_state_publisher` - TF broadcaster
- `joint_state_publisher` - Joint state publisher
- `rviz2` - Visualization

---

## Usage

After building the workspace:

```bash
# Build
cd ~/catalyst-manipulation/ros2_ws
colcon build --packages-select moveit_config

# Source
source install/setup.bash
```

---

## Future Additions

The following configuration files are planned:

- [x] `config/kinematics.yaml` - IK solver configuration
- [ ] `config/joint_limits.yaml` - Joint limits for planning
- [ ] `config/controllers.yaml` - ros2_control configuration
- [ ] `config/ompl_planning.yaml` - OMPL planner parameters
- [ ] `launch/move_group.launch.py` - MoveIt launch file
- [ ] `launch/moveit_rviz.launch.py` - RViz with MoveIt plugin
