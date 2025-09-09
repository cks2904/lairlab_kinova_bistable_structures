# Start from the official ROS 2 Jazzy desktop image
FROM osrf/ros:jazzy-desktop

# Avoid interactive prompts during package installation
ARG DEBIAN_FRONTEND=noninteractive

# Install essential system dependencies, Python, and MoveIt
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-dev-tools \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-moveit \
    ros-jazzy-plotjuggler-ros \
    ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN python3 -m pip install numpy pandas pyserial --break-system-packages

# Set a default working directory
WORKDIR /workspaces

# Source the main ROS 2 setup file, but NOT the local one
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
