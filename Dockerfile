FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install basic development tools
RUN apt-get update && apt-get install -y \
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

# Install MoveIt 2
RUN apt-get update && apt-get install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-setup-assistant \
    ros-jazzy-moveit-visual-tools \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-resources \
    ros-jazzy-force-torque-sensor-broadcaster \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic + ROS 2 bridge + ros2_control plugin
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Copy your src folder from the host to the image
# This allows rosdep to "see" your package.xml files during build
COPY ./ros2_ws/src /root/ros2_ws/src

# Initialize rosdep (only needed once in the image life)
# Then install dependencies defined in your package.xml files
RUN rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -y -r && \
    rm -rf /var/lib/apt/lists/*

# Install Intel RealSense SDK
RUN apt-get update && apt-get install -y software-properties-common \
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
    && apt-get update && apt-get install -y \
    librealsense2-dev \
    librealsense2-utils \
    && rm -rf /var/lib/apt/lists/*

# Install apriltag dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-apriltag-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --break-system-packages dynamixel-sdk xarm-python-sdk

# # Setup workspace directory
# RUN mkdir -p /root/ros2_ws/src
# WORKDIR /root/ros2_ws

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
