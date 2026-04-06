FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install basic development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    wget \
    curl \
    libcurlpp-dev \
    vim \
    nlohmann-json3-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages (MoveIt, Gazebo, BehaviorTree, etc.) in one layer
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-moveit \
    ros-jazzy-moveit-setup-assistant \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-resources \
    ros-jazzy-trac-ik-kinematics-plugin \
    ros-jazzy-force-torque-sensor-broadcaster \
    ros-jazzy-admittance-controller \
    ros-jazzy-kinematics-interface-kdl \
    ros-jazzy-ros2controlcli \
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-behaviortree-cpp \
    ros-jazzy-asio-cmake-module \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-apriltag-msgs \
    libasio-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Intel RealSense SDK (Using direct keyserver fetch to avoid bad Intel .pgp files)
RUN apt-get update && apt-get install -y --no-install-recommends curl gpg ca-certificates \
    && mkdir -p /etc/apt/keyrings \
    && curl -sL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xFB0B24895113F120" \
       | gpg --dearmor -o /etc/apt/keyrings/librealsense.gpg \
    && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/librealsense.gpg] https://librealsense.intel.com/Debian/apt-repo noble main" \
       > /etc/apt/sources.list.d/librealsense.list \
    && apt-get update && apt-get install -y --no-install-recommends \
       librealsense2-dev \
       librealsense2-utils \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Copy source and install rosdep dependencies
COPY ./ros2_ws/src /root/ros2_ws/src

ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -y -r && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --break-system-packages dynamixel-sdk xarm-python-sdk requests

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
