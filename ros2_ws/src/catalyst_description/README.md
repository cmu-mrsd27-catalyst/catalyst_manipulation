# Catalyst Description Package

URDF and meshes for the Catalyst Manipulator (xArm-6 + Bio Gripper).

## Package Structure

```
catalyst_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── display.launch.py          # Visualize robot in RViz
├── meshes/
│   └── gripper/
│       ├── link_base.stl          # Gripper palm mesh
│       ├── left_finger.stl        # Left finger mesh
│       └── right_finger.stl       # Right finger mesh
└── urdf/
    ├── gazebo/
    │   └── gazebo_ros2_control.xacro  # Gz ros2_control plugin
    ├── gripper/
    │   ├── bio_gripper.urdf.xacro         # Gripper kinematic structure
    │   ├── bio_gripper.ros2_control.xacro # ros2_control interfaces
    │   └── bio_gripper.gazebo.xacro       # Gazebo-specific config
    └── gripper_and_arm/
        └── arm_gripper_combined.urdf.xacro # Top-level composition
```

---

## Quick Start

### Visualize the Robot

```bash
cd ~/catalyst_repo_jazzy/catalyst-manipulation/ros2_ws
colcon build --packages-select catalyst_description
source install/setup.bash

# Launch RViz with joint control GUI
ros2 launch catalyst_description display.launch.py
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_gui` | `true` | Show joint state publisher GUI |
| `rviz_config` | `rviz/display.rviz` | Path to RViz config file |

```bash
# Launch without joint GUI
ros2 launch catalyst_description display.launch.py use_gui:=false
```

---

## Robot Composition

The Catalyst Manipulator combines an **xArm-6** arm (from upstream `xarm_description`) with a custom **Bio Gripper** end-effector.

### Kinematic Chain

```
world
  └── link_base (xArm-6)
        └── link1
              └── link2
                    └── link3
                          └── link4
                                └── link5
                                      └── link6
                                            └── link_eef
                                                  └── bio_gripper_base_link (fixed)
                                                        ├── bio_gripper_right_finger (prismatic, Y-axis)
                                                        ├── bio_gripper_left_finger  (prismatic, mimic)
                                                        └── link_tcp (fixed, tool center point)
```

### Gripper Details

| Property | Value |
|----------|-------|
| Type | Bio Gripper (2-finger parallel) |
| Actuated joint | `right_finger_joint` (prismatic, Y-axis) |
| Mimic joint | `left_finger_joint` (mirrors right, multiplier=-1) |
| Range | 0.0 to 0.04 m (per finger) |
| Max velocity | 0.1 m/s |
| Max effort | 2.0 N |
| TCP offset | (0.135, 0, 0.055) from gripper base |

---

## URDF Xacro Files

### `arm_gripper_combined.urdf.xacro` (Top-level)

Main composition file. Includes the xArm-6 from upstream and attaches the bio gripper.

**Key xacro arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `prefix` | `""` | Namespace prefix for links/joints |
| `hw_ns` | `xarm` | Hardware namespace |
| `robot_ip` | `192.168.1.212` | Real robot IP address |
| `limited` | `false` | Enforce joint limits |
| `velocity_control` | `false` | Use velocity control mode |
| `ros2_control_plugin` | `uf_robot_hardware/UFRobotFakeSystemHardware` | Hardware plugin |
| `use_gazebo` | `false` | Include Gazebo plugins |
| `add_gripper_ros2_control` | `true` | Include gripper ros2_control interfaces |

**Hardware plugin options:**

| Use case | Plugin value |
|----------|-------------|
| Fake hardware (default) | `uf_robot_hardware/UFRobotFakeSystemHardware` |
| Gz Harmonic simulation | `gz_ros2_control/GazeboSimSystem` |
| Real robot | `uf_robot_hardware/UFRobotSystemHardware` |

**Example xacro generation:**

```bash
# Fake hardware (for RViz visualization)
xacro arm_gripper_combined.urdf.xacro

# Gz Harmonic simulation
xacro arm_gripper_combined.urdf.xacro use_gazebo:=true ros2_control_plugin:=gz_ros2_control/GazeboSimSystem

# Real robot
xacro arm_gripper_combined.urdf.xacro ros2_control_plugin:=uf_robot_hardware/UFRobotSystemHardware robot_ip:=192.168.1.212
```

### `bio_gripper.urdf.xacro`

Defines the gripper kinematic structure, collision/visual geometry, and inertial properties. Uses STL meshes from `meshes/gripper/`.

### `bio_gripper.ros2_control.xacro`

Defines ros2_control command and state interfaces:
- **Right finger** (primary): position + velocity command, position + velocity state
- **Left finger** (mimic): position + velocity state only (tracks right finger)

### `bio_gripper.gazebo.xacro`

Gazebo-specific configuration:
- Disables self-collision for gripper links
- Grasp fix plugin definition (currently commented out)

### `gazebo_ros2_control.xacro`

Adds the `gz_ros2_control::GazeboSimROS2ControlPlugin` to the URDF. Points to the controller config in `catalyst_bringup`.

---

## Meshes

Three STL mesh files for the bio gripper, located in `meshes/gripper/`:

| File | Description |
|------|-------------|
| `link_base.stl` | Gripper palm/base |
| `left_finger.stl` | Left finger |
| `right_finger.stl` | Right finger |

These are used for both visual rendering and collision detection.

---

## Dependencies

- `xarm_description` — Upstream xArm-6 URDF and meshes
- `robot_state_publisher` — Publishes TF from URDF
- `joint_state_publisher_gui` — Joint control slider GUI
- `rviz2` — 3D visualization
- `xacro` — URDF macro processing

---

## See Also

- [catalyst_gazebo README](../catalyst_gazebo/README.md) — Gz Harmonic simulation
- [catalyst_bringup README](../catalyst_bringup/README.md) — Controller configuration and launch
