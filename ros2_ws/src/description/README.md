# Robot Description Package

URDF/Xacro description package for the Catalyst Manipulator (xArm-6 + Bio Gripper).

## Package Structure

```
description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── display.launch.py
├── meshes/
│   └── gripper/
│       ├── link_base.stl      (282 KB)
│       ├── left_finger.stl    (48 KB)
│       └── right_finger.stl   (48 KB)
└── urdf/
    ├── gripper/
    │   ├── bio_gripper.urdf.xacro
    │   ├── bio_gripper_macro.xacro
    │   ├── bio_gripper.ros2_control.xacro
    │   ├── bio_gripper.gazebo.xacro
    │   └── bio_gripper.transmission.xacro
    └── gripper_and_arm/
        └── arm_gripper_combined.urdf.xacro
```

---

## TF Tree Structure

```
world
  │
  └── link_base                    (xArm-6 base)
        │
        └── [joint1 - revolute]
              │
              └── link1
                    │
                    └── [joint2 - revolute]
                          │
                          └── link2
                                │
                                └── [joint3 - revolute]
                                      │
                                      └── link3
                                            │
                                            └── [joint4 - revolute]
                                                  │
                                                  └── link4
                                                        │
                                                        └── [joint5 - revolute]
                                                              │
                                                              └── link5
                                                                    │
                                                                    └── [joint6 - revolute]
                                                                          │
                                                                          └── link6
                                                                                │
                                                                                └── [joint_eef - fixed]
                                                                                      │
                                                                                      └── link_eef
                                                                                            │
                                                                                            └── [bio_gripper_fix - fixed]
                                                                                                  │
                                                                                                  └── bio_gripper_base_link
                                                                                                        │
                                                                                                        ├── [left_finger_joint - prismatic, mimic]
                                                                                                        │     │
                                                                                                        │     └── bio_gripper_left_finger
                                                                                                        │
                                                                                                        ├── [right_finger_joint - prismatic]
                                                                                                        │     │
                                                                                                        │     └── bio_gripper_right_finger
                                                                                                        │
                                                                                                        └── [joint_tcp - fixed]
                                                                                                              │
                                                                                                              └── link_tcp (Tool Center Point)
```

---

## Joint Summary

### xArm-6 Joints

| Joint | Type | Parent | Child | Axis |
|-------|------|--------|-------|------|
| joint1 | revolute | link_base | link1 | Z |
| joint2 | revolute | link1 | link2 | Y |
| joint3 | revolute | link2 | link3 | Y |
| joint4 | revolute | link3 | link4 | Y |
| joint5 | revolute | link4 | link5 | Y |
| joint6 | revolute | link5 | link6 | Y |
| joint_eef | fixed | link6 | link_eef | - |

### Bio Gripper Joints

| Joint | Type | Parent | Child | Axis | Limits |
|-------|------|--------|-------|------|--------|
| bio_gripper_fix | fixed | link_eef | bio_gripper_base_link | - | - |
| left_finger_joint | prismatic | bio_gripper_base_link | bio_gripper_left_finger | Y | [-0.04, 0] m |
| right_finger_joint | prismatic | bio_gripper_base_link | bio_gripper_right_finger | Y | [0, 0.04] m |
| joint_tcp | fixed | bio_gripper_base_link | link_tcp | - | - |

**Note:** `left_finger_joint` mimics `right_finger_joint` with multiplier -1 (symmetric closure).

---

## Link Summary

### xArm-6 Links

| Link | Description |
|------|-------------|
| link_base | Robot base, mounted to world |
| link1 | First arm segment (shoulder) |
| link2 | Second arm segment (upper arm) |
| link3 | Third arm segment (elbow) |
| link4 | Fourth arm segment (forearm) |
| link5 | Fifth arm segment (wrist 1) |
| link6 | Sixth arm segment (wrist 2) |
| link_eef | End effector flange |

### Bio Gripper Links

| Link | Description | Mass |
|------|-------------|------|
| bio_gripper_base_link | Gripper base structure | 0.388 kg |
| bio_gripper_left_finger | Left finger | 0.012 kg |
| bio_gripper_right_finger | Right finger | 0.012 kg |
| link_tcp | Tool Center Point (grasp reference) | - |

---

## URDF Components

### Main Composition File

`urdf/gripper_and_arm/arm_gripper_combined.urdf.xacro`

Composes the full robot by:
1. Including xArm-6 from `xarm_description` package
2. Including local bio gripper
3. Attaching gripper to `link_eef`

### Xacro Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `prefix` | `""` | Namespace prefix for multi-robot setups |
| `hw_ns` | `xarm` | Hardware namespace |
| `robot_ip` | `""` | Robot IP for real hardware |
| `robot_sn` | `""` | Robot serial number |
| `limited` | `false` | Use limited joint ranges |
| `velocity_control` | `false` | Enable velocity control mode |
| `ros2_control_plugin` | `UFRobotFakeSystemHardware` | Hardware interface plugin |

### Bio Gripper Files

| File | Purpose |
|------|---------|
| `bio_gripper.urdf.xacro` | Main gripper URDF with links and joints |
| `bio_gripper_macro.xacro` | Wrapper macro (legacy) |
| `bio_gripper.ros2_control.xacro` | ros2_control hardware interface |
| `bio_gripper.gazebo.xacro` | Gazebo simulation plugins |
| `bio_gripper.transmission.xacro` | Transmission specifications |

---

## Launch Files

### display.launch.py

Visualize the robot model in RViz with interactive joint control.

**Usage:**
```bash
ros2 launch description display.launch.py
```

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `use_gui` | `true` | Launch joint_state_publisher_gui |
| `rviz_config` | `rviz/display.rviz` | Path to RViz config |

**What it starts:**
- `robot_state_publisher` - Publishes robot state to TF
- `joint_state_publisher_gui` - GUI for manual joint control
- `rviz2` - 3D visualization

---

## Coordinate Frames

### Key Frames

| Frame | Description | Use For |
|-------|-------------|---------|
| `world` | Global fixed frame | Motion planning reference |
| `link_base` | Robot base | Mounting reference |
| `link_eef` | End effector flange | Gripper attachment |
| `link_tcp` | Tool Center Point | Grasp planning, IK target |

### TCP Position

The Tool Center Point (`link_tcp`) is offset from `bio_gripper_base_link`:
- X: 0.135 m (forward)
- Y: 0.0 m
- Z: 0.055 m (up)

---

## Dependencies

- `xarm_description` - xArm robot model (from xarm_ros2)
- `robot_state_publisher` - TF broadcaster
- `joint_state_publisher_gui` - Joint control GUI
- `rviz2` - Visualization

---

## Building

```bash
cd ~/catalyst-manipulation/ros2_ws
colcon build --packages-select description
source install/setup.bash
```

---

## Visualization

```bash
# View robot model with joint sliders
ros2 launch description display.launch.py

# Headless (no GUI)
ros2 launch description display.launch.py use_gui:=false
```
