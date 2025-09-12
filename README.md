# Variable Stiffness Kinova Gen3 Robotic Fingers with Bistable Structures

This project implements a variable stiffness control system for the Kinova Gen3 robotic arm using bistable finger structures, integrating tactile sensing and vision capabilities for precision manipulation. The system is containerized using Docker for consistent deployment across different machines.

## System Overview

The project integrates several key components:
- Kinova Gen3 robotic arm control
- Custom bistable finger structures
- Tactile sensing system
- RealSense D435 camera integration
- Arduino-based force feedback
- ROS2 control framework

## Prerequisites

- Docker
- Ubuntu-based system with:
  - X11 for GUI applications
  - USB support for hardware devices
  - Network capabilities for ROS2 communication

## Hardware Requirements

- Kinova Gen3 6-DOF robotic arm
- Intel RealSense D435 camera
- Arduino boards (2x):
  - One for servo force control
  - One for tactile sensing
- Network connection for robot communication

## Project Structure

```
.
├── src/
│   ├── arduino_servo_force_publisher/    # Arduino force feedback integration
│   ├── gripper_control/                  # Gripper control implementation
│   ├── librealsense-2.56.5/             # RealSense SDK
│   ├── realsense-ros/                    # ROS2 wrapper for RealSense
│   ├── ros2_arduino_servo/              # Arduino servo control
│   ├── ros2_control/                    # ROS2 control framework
│   ├── ros2_kortex/                     # Kinova robot ROS2 driver
│   ├── ros2_robotiq_gripper/           # Robotiq gripper integration
│   └── tactile_sensor_ros2_ws/         # Tactile sensor integration
```

## Setup Instructions

### 1. Docker Environment Setup

```bash
# Grant X server access to local user
xhost +local:

# Build and start the Docker container via VS Code
# 1. Open the project in VS Code
code .
# 2. When prompted, click "Reopen in Container" or use Command Palette (F1)
#    and select "Dev Containers: Reopen in Container"
```

#### Working with Docker Terminals

You can work with the Docker environment in two ways:

1. **VS Code Integrated Terminal**:
   - Open VS Code's integrated terminal
   - It's already connected to the container

2. **External Terminal**:
   ```bash
   # List running containers
   docker ps
   
   # Connect to the container (replace container_name with actual name)
   docker exec -it <container_name> /bin/bash
   ```

⚠️ **Important**: All development commands (colcon build, ros2 run, etc.) must be executed inside the Docker container terminal.

### 2. Arduino Setup

First-time Arduino setup requires udev rules configuration:

```bash
# Get device attributes
udevadm info -a -n /dev/ttyACM0 | grep -E "ATTRS{idVendor}|ATTRS{idProduct}|ATTRS{serial}"

# Create udev rules
sudo nano /etc/udev/rules.d/99-arduino.rules

# Add rules like:
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="ttyServo"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="ttyTactile"

# Apply rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Verify setup
ls -l /dev/ttyServo && ls -l /dev/ttyTactile
```

### 3. Build Workspace

Inside the Docker container:

```bash
cd /workspaces/lairlab_kinova_bistable_structures
colcon build
source install/setup.bash
```

## Running the System

1. Start the robot control node:
```bash
ros2 launch kortex_bringup gen3.launch.py robot_ip:=192.168.1.10 dof:=6
```

2. Launch the camera node:
```bash
ros2 run realsense2_camera realsense2_camera_node
```

3. Start tactile sensor publisher:

The tactile sensor implementation is located in the [src/tactile_sensor_ros2_ws](src/tactile_sensor_ros2_ws) submodule. Follow these steps in the Docker terminal:

```bash
# Build just the tactile sensor package
cd /workspaces/lairlab_kinova_bistable_structures
colcon build --packages-select tactile_sensor_pkg
source install/setup.bash

# Start the tactile sensor publisher
ros2 run tactile_sensor_pkg tactile_publisher --ros-args -p serial_port:='/dev/ttyTactile'

# In a separate terminal, configure sensor parameters if needed
ros2 param get /tactile_sensor_publisher touch_threshold_percentage
ros2 param set /tactile_sensor_publisher touch_threshold_percentage 15.0
```

Available tactile sensor topics:
- `/tactile_raw_values`: Raw sensor readings
- `/tactile_binary_array`: Binary contact detection array
- `/tactile_touch_detected`: Overall touch detection status

For detailed information about the tactile sensor implementation, refer to the [tactile sensor documentation](src/tactile_sensor_ros2_ws/README.md).

4. Launch servo force publisher:
```bash
ros2 run ros2_arduino_servo servo_node --ros-args -p serial_port:='/dev/ttyServo'
```

5. Start visualization (optional):
```bash
# For visualization with Foxglove
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# For PlotJuggler visualization
ros2 run plotjuggler plotjuggler
```

## Data Recording

Record experimental data using ROS2 bags:

```bash
ros2 bag record -o /path/to/output --topics \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /tactile_binary_array \
  /tactile_raw_values \
  /tactile_touch_detected \
  /gripper_moved_distance \
  /servo_feedback
```

## Development

### Docker Development Workflow

All development work should be done inside the Docker container:

1. **Initial Setup**:
   ```bash
   # In Docker terminal
   cd /workspaces/lairlab_kinova_bistable_structures
   colcon build
   source install/setup.bash
   ```

2. **Clean Build** (if needed):
   ```bash
   # In Docker terminal
   cd /workspaces/lairlab_kinova_bistable_structures
   rm -rf build install log
   colcon build
   source install/setup.bash
   ```

3. **Building Specific Packages**:
   ```bash
   # Build single package
   colcon build --packages-select my_package_name

   # Build with reduced parallelism (for slower machines)
   colcon build --parallel-workers 1
   ```

4. **Running Tests**:
   ```bash
   # Run tests for all packages
   colcon test

   # Run tests for specific package
   colcon test --packages-select my_package_name
   ```

Remember:
- Always run development commands in the Docker container
- Source setup.bash after every build: `. install/setup.bash`
- Use VS Code's integrated terminal or external terminal with `docker exec -it`

## Troubleshooting

### Common Issues

1. **Camera Not Detected**
   - Check USB connection
   - Verify using `rqt` image viewer
   - Ensure camera node is running

2. **Arduino Connection Issues**
   - Verify udev rules are properly set
   - Check USB port permissions
   - Confirm correct port assignments

3. **Robot Communication Failed**
   - Verify network settings
   - Check IP address configuration
   - Ensure ethernet connection is active

### Network Configuration

If robot connection fails:
```bash
# Check network interface
ip link

# Set manual IP (if needed)
sudo ip addr flush dev enp4s0
sudo ip addr add 192.168.1.11/24 dev enp4s0
sudo ip link set enp4s0 up
```

## License

[Insert License Information]

## Contributors

[List Contributors]