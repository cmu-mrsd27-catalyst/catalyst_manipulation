# Docker Setup for Catalyst Manipulator (ROS 2 Jazzy)

## Prerequisites

- Docker with NVIDIA Container Toolkit (`nvidia-docker2`)
- NVIDIA GPU with drivers installed
- X11 display server (for GUI apps like RViz and Gazebo)

## Image Contents

The `ros2_jazzy` image is based on `osrf/ros:jazzy-desktop` and includes:

- ROS 2 Jazzy (desktop)
- MoveIt 2 (planner, servo, setup assistant, visual tools)
- Gz Harmonic + ros_gz bridge + gz_ros2_control
- Intel RealSense SDK (librealsense2)
- AprilTag messages
- Dynamixel SDK (Python)
- Build tools (colcon, cmake, rosdep, pip)

## Build the Image

From the repository root (`catalyst-manipulation/`):

```bash
docker build -t ros2_jazzy .
```

To rebuild without cache (e.g. after updating the Dockerfile):

```bash
docker build --no-cache -t ros2_jazzy .
```

## Run a Container

```bash
./run_docker.sh
```

This starts an interactive container with:
- **GPU access** (NVIDIA runtime)
- **Privileged mode** (access to all host devices including USB serial ports)
- **Host networking** (`--net=host` for direct xArm TCP communication)
- **X11 forwarding** (RViz, Gazebo GUI)
- **Device mount** (`/dev:/dev` for serial ports like `/dev/ttyUSB0`)
- **Workspace mount** (`./ros2_ws` mounted to `/root/ros2_ws`)

The workspace is bind-mounted, so any changes inside the container (builds, edits) are reflected on the host and vice versa.

### Config paths (Docker)

ROS resolves package files through **install space** after `source install/setup.bash` (e.g. `/root/ros2_ws/install/<pkg>/share/<pkg>/`), not through your host path like `/home/...`.

`run_docker.sh` sets **`CATALYST_WORLD_MODEL_YAML=/root/ros2_ws/src/catalyst_world_model/config/world_model.yaml`** so `world_model` health toggles always follow the **source** tree you edit in the bind-mounted workspace. Override or unset that variable if you want to use only the installed copy.

## Attach Additional Terminals

To open another terminal in the same running container:

```bash
./attach_docker.sh
```

This finds the running `ros2_jazzy` container, attaches to it, and sources the workspace overlay if available.

## Build the Workspace (Inside the Container)

```bash
cd /root/ros2_ws
colcon build
source install/setup.bash
```

To build specific packages:

```bash
colcon build --packages-select catalyst_description catalyst_gazebo catalyst_bringup
source install/setup.bash
```

## Launch

**Fake hardware** (no robot, no simulation — quick MoveIt testing):

```bash
ros2 launch catalyst_bringup demo.launch.py sim:=fake
```

**Gazebo simulation** (Gz Harmonic):

```bash
ros2 launch catalyst_bringup demo.launch.py sim:=gazebo
```

**Real hardware** (xArm connected via Ethernet):

```bash
ros2 launch catalyst_bringup demo.launch.py sim:=false robot_ip:=192.168.1.212
```

## Useful Commands (Inside the Container)

Check running nodes:

```bash
ros2 node list
```

Check available topics:

```bash
ros2 topic list
```

Check controllers:

```bash
ros2 control list_controllers
```

Verify sim clock is bridged (Gazebo mode):

```bash
ros2 topic echo /clock --once
```

## Cleaning Build Artifacts

If you encounter stale build issues:

```bash
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

To clean a specific package:

```bash
rm -rf build/<package_name> install/<package_name>
colcon build --packages-select <package_name>
```

## Troubleshooting

**"cannot open display"** — Run `xhost +local:docker` on the host before starting the container (already included in `run_docker.sh`).

**"Failed to open port /dev/ttyUSB0"** — Ensure the Dynamixel USB adapter is plugged in and the container was started with `--privileged` and `-v /dev:/dev`.

**"Could not contact xArm"** — Verify the xArm is powered on and reachable at the specified IP. The container uses `--net=host` so it shares the host network — test with `ping 192.168.1.212` inside the container.

**"Failed to load system plugin gz_ros2_control-system"** — Rebuild the image to include `ros-jazzy-gz-ros2-control`, or install it manually: `apt-get update && apt-get install -y ros-jazzy-gz-ros2-control`.
