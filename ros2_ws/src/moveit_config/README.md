# MoveIt Configuration for Catalyst Manipulator

MoveIt2 configuration package for the Catalyst Manipulator (xArm-6 + Bio Gripper).

## Package Structure

```
moveit_config/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── controllers.yaml
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   └── ompl_planning.yaml
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

## Joint Limits Configuration

Location: `config/joint_limits.yaml`

Reference: Aligned with [xarm_ros2 joint_limits](https://github.com/xArm-Developer/xarm_ros2/blob/humble/xarm_moveit_config/config/xarm6/joint_limits.yaml)

Overrides URDF joint limits for MoveIt planning (more conservative for safety).

### xArm-6 Joints (joint1-6)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `has_velocity_limits` | `true` | Enable velocity limits |
| `max_velocity` | `2.14` | Maximum velocity (rad/s) |
| `has_acceleration_limits` | `true` | Enable acceleration limits |
| `max_acceleration` | `10.0` | Maximum acceleration (rad/s²) |

### Bio Gripper Joints (left/right_finger_joint)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `has_velocity_limits` | `true` | Enable velocity limits |
| `max_velocity` | `3.14` | Maximum velocity (rad/s) |
| `has_acceleration_limits` | `true` | Enable acceleration limits |
| `max_acceleration` | `10.0` | Maximum acceleration (rad/s²) |

---

## Controllers Configuration

Location: `config/controllers.yaml`

Reference: Aligned with [xarm_ros2 controllers](https://github.com/xArm-Developer/xarm_ros2/blob/humble/xarm_moveit_config/config/xarm6/controllers.yaml)

Defines the controllers that MoveIt will use to execute trajectories.

### xarm6_traj_controller

```yaml
xarm6_traj_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
```

- **Type**: `FollowJointTrajectory` - Standard trajectory execution
- **Joints**: All 6 arm joints

### bio_gripper_controller

```yaml
bio_gripper_controller:
  action_ns: gripper_action
  type: GripperCommand
  default: true
  joints:
    - right_finger_joint
```

- **Type**: `GripperCommand` - Gripper-specific control interface
- **Joints**: Only `right_finger_joint` (left mimics right)

---

## OMPL Planning Configuration

Location: `config/ompl_planning.yaml`

Reference: Aligned with [xarm_ros2 ompl_planning](https://github.com/xArm-Developer/xarm_ros2/blob/humble/xarm_moveit_config/config/xarm6/ompl_planning.yaml)

Configures OMPL (Open Motion Planning Library) motion planners.

### Available Planners (23 total)

| Category | Planners |
|----------|----------|
| Tree-based | SBL, EST, KPIECE, BKPIECE, LBKPIECE, RRT, RRTConnect, RRTstar, TRRT, BiTRRT, LBTRRT |
| Graph-based | PRM, PRMstar, LazyPRM, LazyPRMstar, SPARS, SPARStwo |
| Other | FMT, BFMT, PDST, STRIDE, BiEST, ProjEST |

### Group Configuration

| Group | Default Planner | Notes |
|-------|-----------------|-------|
| `xarm6` | `RRTConnect` | All 23 planners available |
| `bio_gripper` | `RRTConnect` | Only RRTConnect (simple motion) |

### Recommended Planners

| Use Case | Planner | Description |
|----------|---------|-------------|
| Fast planning | `RRTConnect` | Bidirectional RRT, good for most cases |
| Optimal paths | `RRTstar` | Asymptotically optimal, slower |
| Narrow passages | `KPIECE` | Good for constrained spaces |
| Complex environments | `PRM` | Probabilistic roadmap |

### Planner Comparison

| Planner | Best For | Trade-off |
|---------|----------|-----------|
| **RRTConnect** | General use, fast planning | Not optimal paths |
| **RRTstar** | Optimal/smooth paths | Slower, needs more time |
| **PRM/PRMstar** | Repeated queries in same environment | Slow initial roadmap build |
| **KPIECE/BKPIECE** | High-DOF robots, narrow passages | More complex |
| **BiTRRT** | Cost-aware planning | Needs cost function |
| **FMT/BFMT** | Near-optimal, faster than RRT* | Memory intensive |
| **LazyPRM** | Quick approximate solutions | May need refinement |

### Why RRTConnect is Default

- **Bidirectional search** - Grows trees from both start and goal
- **Fast for most cases** - Works well in open/semi-cluttered environments
- **No parameter tuning** - Works out-of-the-box
- **Industry standard** - Used by xarm_ros2 and most MoveIt configs

### Changing the Planner

**Option 1: In Python code**

```python
from moveit_commander import MoveGroupCommander

move_group = MoveGroupCommander("xarm6")

# Change planner
move_group.set_planner_id("RRTstar")

# Optional: increase planning time for optimal planners
move_group.set_planning_time(5.0)

# Plan and execute
move_group.go()
```

**Option 2: In C++ code**

```cpp
#include <moveit/move_group_interface/move_group_interface.h>

auto move_group = moveit::planning_interface::MoveGroupInterface(node, "xarm6");

// Change planner
move_group.setPlannerId("RRTstar");

// Optional: increase planning time
move_group.setPlanningTime(5.0);

// Plan and execute
move_group.move();
```

**Option 3: Change default in config file**

Edit `config/ompl_planning.yaml`:

```yaml
xarm6:
  default_planner_config: RRTstar  # Change from RRTConnect
```

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
- [x] `config/joint_limits.yaml` - Joint limits for planning
- [x] `config/controllers.yaml` - ros2_control configuration
- [x] `config/ompl_planning.yaml` - OMPL planner parameters
- [ ] `launch/move_group.launch.py` - MoveIt launch file
- [ ] `launch/moveit_rviz.launch.py` - RViz with MoveIt plugin
