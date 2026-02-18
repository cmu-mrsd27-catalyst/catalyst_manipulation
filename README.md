# Catalyst Manipulation Workspace

This repository contains the Catalyst Manipulation ROS 2 workspace, including core project packages, external drivers, and setup automation scripts.

---

## Installation & Setup

Follow these steps to set up the workspace on a new machine.

### 1. Clone this Repository

```bash
git clone https://github.com/cmu-mrsd27-catalyst/catalyst_manipulation.git
cd catalyst_manipulation/ros2_ws/src
```

---

### 2. Run the Setup Script

This script automates:

- Installation of the Intel RealSense SDK  
- Importing external repositories (xArm, RealSense)  
- Initializing Git submodules  

```bash
chmod +x setup_workspace.sh
./setup_workspace.sh
```

---

### 3. Install System Dependencies

Use `rosdep` to install any remaining system libraries required by the source code:

```bash
cd ros2_ws
rosdep update
rosdep install -i --from-path src --rosdistro humble -y --skip-keys "catalyst_gripper"
```

---

### 4. Build the Workspace

```bash
colcon build --symlink-install
```

---

### Manual Sourcing

```bash
source ~/catalyst_manipulation/ros2_ws/install/setup.bash
```


### Directory Overview

- **ros2_ws/src/external/**  
  Contains external drivers (xArm, RealSense) managed via `deps.repos`.

- **ros2_ws/src/catalyst_*/**  
  Core project packages (Bringup, Gazebo, Description).

- **setup_workspace.sh**  
  Automation script for environment initialization.

- **deps.repos**  
  Version-controlled list of external repository dependencies.

---

## Notes

- Ensure you are using **ROS 2 Humble**.
- Always source the workspace before running any ROS 2 nodes.
- If dependencies change, rerun `rosdep install` and rebuild the workspace.
