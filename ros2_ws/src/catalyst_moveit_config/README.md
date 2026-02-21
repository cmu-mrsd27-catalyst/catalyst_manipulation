# Catalyst MoveIt Config Package

MoveIt2 configuration for the Catalyst Manipulator (xArm-6 + Bio Gripper).

## Package Structure

```
catalyst_moveit_config/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── controllers.yaml       # MoveIt controller interface definitions
│   ├── joint_limits.yaml      # Velocity and acceleration limits
│   ├── kinematics.yaml        # IK solver configuration
│   └── ompl_planning.yaml     # OMPL planner settings (Jazzy format)
├── launch/
│   ├── move_group.launch.py   # move_group node only
│   └── moveit_rviz.launch.py  # move_group + RViz + joint GUI
├── rviz/
│   └── moveit.rviz            # RViz config with MotionPlanning plugin
└── srdf/
    └── catalyst_manipulator.srdf  # Semantic robot description
```

---

## Quick Start

```bash
cd ~/catalyst_repo_jazzy/catalyst-manipulation/ros2_ws
colcon build --packages-select catalyst_moveit_config
source install/setup.bash

# Launch MoveIt with RViz and joint GUI (standalone, no Gazebo)
ros2 launch catalyst_moveit_config moveit_rviz.launch.py

# Launch move_group only (for use with other launch files)
ros2 launch catalyst_moveit_config move_group.launch.py use_sim_time:=true
```

---

## Launch Arguments

### move_group.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation clock |

### moveit_rviz.launch.py

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation clock |
| `rviz_config` | `moveit.rviz` | Path to RViz config file |

---

## SRDF — Planning Groups

The SRDF defines three planning groups:

| Group | Type | Joints | Description |
|-------|------|--------|-------------|
| `xarm6` | Chain | joint1–joint6 | Arm (link_base → link_eef) |
| `bio_gripper` | Joint | right_finger_joint | Gripper (left finger is mimic) |
| `xarm6_with_gripper` | Chain + Joint | joint1–joint6 + right_finger_joint | Combined |

### Named Poses

| Pose | Group | Description |
|------|-------|-------------|
| `home` | xarm6 | All joints at 0 |
| `hold_up` | xarm6 | joint5 at -90 degrees |
| `open` | bio_gripper | right_finger_joint at 0.04 m |
| `close` | bio_gripper | right_finger_joint at 0 m |

### End Effector

The `bio_gripper` group is registered as an end effector attached to `link_eef`, with `xarm6` as the parent group.

### Passive Joints

`left_finger_joint` is marked passive — it mimics `right_finger_joint` with multiplier -1 and is excluded from planning.

---

## Configuration Files

### kinematics.yaml

IK solver for the `xarm6` group:

| Parameter | Value |
|-----------|-------|
| Solver | `kdl_kinematics_plugin/KDLKinematicsPlugin` |
| Search resolution | 0.005 |
| Timeout | 0.005 s |
| Attempts | 3 |

Alternative solvers (TRAC-IK, pick_ik) are available as commented options.

### joint_limits.yaml

Overrides URDF joint limits for MoveIt planning:

| Joints | Max velocity | Max acceleration |
|--------|-------------|-----------------|
| joint1–joint6 (arm) | 2.14 rad/s | 10.0 rad/s² |
| finger joints (gripper) | 3.14 rad/s | 10.0 rad/s² |

### controllers.yaml

Maps MoveIt planning groups to ros2_control controllers:

| Controller | Type | Joints |
|------------|------|--------|
| `xarm6_traj_controller` | FollowJointTrajectory | joint1–joint6 |
| `bio_gripper_controller` | FollowJointTrajectory | right_finger_joint |

### ompl_planning.yaml

OMPL planner configuration using Jazzy format (array-based adapters):

- **Default planner:** RRTConnect
- **Available planners:** 23 (SBL, EST, KPIECE, RRT, RRTConnect, RRTstar, TRRT, PRM, PRMstar, FMT, BFMT, PDST, STRIDE, BiTRRT, LBTRRT, BiEST, ProjEST, LazyPRM, LazyPRMstar, SPARS, SPARStwo, LBKPIECE, BKPIECE)

**Jazzy adapter format** (differs from Humble):
- `planning_plugins`: array (was singular `planning_plugin` string)
- `request_adapters`: array of individual adapter strings (was space-separated string)
- `response_adapters`: separate array (was combined with request_adapters)

---

## Disabled Collisions

The SRDF disables collision checking for:
- **Adjacent arm links:** link_base↔link1, link1↔link2, etc.
- **Distant arm links:** link1↔link3, link2↔link4, etc.
- **Gripper internals:** base↔fingers, finger↔finger
- **Gripper-to-arm:** gripper links↔link4/link5/link6/link_eef

---

## Dependencies

- `catalyst_description` — Robot URDF (xArm-6 + bio gripper)
- `moveit_ros_move_group` — move_group node
- `moveit_ros_planning_interface` — Planning interface
- `moveit_kinematics` — IK solvers (KDL)
- `moveit_planners_ompl` — OMPL planning plugin
- `moveit_ros_visualization` — RViz MoveIt plugin
- `robot_state_publisher` — TF publishing from URDF
- `joint_state_publisher` — Joint state GUI for testing

---

## See Also

- [catalyst_description README](../catalyst_description/README.md) — Robot URDF
- [catalyst_gazebo README](../catalyst_gazebo/README.md) — Gz Harmonic simulation
- [catalyst_bringup README](../catalyst_bringup/README.md) — Controller configuration and unified launch
