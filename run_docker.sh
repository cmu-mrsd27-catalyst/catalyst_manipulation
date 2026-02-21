#!/bin/bash
xhost +local:docker

docker run -it \
  --gpus all \
  --runtime=nvidia \
  --privileged \
  --net=host \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e __NV_PRIME_RENDER_OFFLOAD=1 \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v ./ros2_ws:/root/ros2_ws \
  ros2_jazzy
