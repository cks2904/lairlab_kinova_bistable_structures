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

# If you need to install specific Python packages, you can add them here.
# For example:
# RUN python3 -m pip install numpy pandas --break-system-packages
RUN python3 -m pip install numpy pandas pyserial --break-system-packages

# Create a ROS 2 workspace directory
WORKDIR /ros2_ws

# Source the ROS 2 setup file in the bashrc to make it available in all terminals
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/local_setup.bash" >> ~/.bashrc
