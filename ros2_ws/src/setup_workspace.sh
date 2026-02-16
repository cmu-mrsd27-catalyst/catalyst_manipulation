#!/bin/bash
# 1. Install system-level RealSense SDK
sudo apt update
sudo apt install -y ros-humble-librealsense2*

# 2. Import external repos from the .repos file
# The --recursive flag handles the xarm_sdk submodules automatically
vcs import src < src/deps.repos --recursive

# 3. Install all other ROS dependencies automatically
# We skip catalyst_gripper if it still has naming issues in package.xml
rosdep update
rosdep install -i --from-path src --rosdistro humble -y --skip-keys "catalyst_gripper"

echo "Workspace is ready to build with 'colcon build'"
