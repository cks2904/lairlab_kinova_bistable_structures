# Variable Stiffness Kinova Gen3 Robotic Fingers with Bistable Structures

This project implements a variable stiffness control system for the Kinova Gen3 robotic arm using bistable finger structures, integrating tactile sensing and vision capabilities for precision manipulation. The system is containerized using Docker for consistent deployment across different machines.

🚀 **Quick Start Guide**
1. Clone this repository with submodules (see [Clone Instructions](#clone-instructions) below)
2. Install Docker and VS Code (see [Docker Setup](#docker-setup) for your OS)
3. Open in VS Code and click "Reopen in Container"
4. Follow the Hardware Setup section below
5. Build and run the system

### Clone Instructions

```bash
# Clone with submodules (recommended)
git clone --recursive https://github.com/cks2904/lairlab_kinova_bistable_structures.git

# If you already cloned without --recursive, run:
git submodule update --init --recursive
```

### Managing Submodules

This project uses several submodules (e.g., tactile_sensor_pkg, image_extractor_pkg). Here's how to manage them:

#### Updating All Submodules
```bash
# Update all submodules to their latest versions
git submodule update --remote --merge

# Then commit the updates
git add .
git commit -m "chore: Update submodules"
git push
```

#### Updating Specific Submodule
```bash
# Navigate to submodule directory
cd src/image_extractor_pkg

# Pull latest changes
git pull origin main

# Return to main project and commit
cd ../..
git add src/image_extractor_pkg
git commit -m "chore: Update image_extractor_pkg submodule"
git push
```

#### Troubleshooting Submodules
```bash
# If submodules appear empty
git submodule update --init --recursive

# If submodules are on wrong commit
git submodule update --recursive

# Check submodule status
git submodule status
```

⚠️ **Important**: Always use `--recursive` when cloning or updating to ensure all nested submodules are properly initialized.

### Docker Setup �

Install Docker for your operating system:
- [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/)
- [Docker Desktop for Mac](https://docs.docker.com/desktop/install/mac-install/)
- [Docker Engine for Linux](https://docs.docker.com/engine/install/)

⚠️ **Note**: For Windows users, WSL2 is required. Follow the [WSL2 installation guide](https://learn.microsoft.com/en-us/windows/wsl/install) first.

### VS Code Setup
1. Install [VS Code](https://code.visualstudio.com/download)
2. Install the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

⭐ **Key Features**
- Containerized development environment for easy setup
- Integrated tactile sensing and vision systems
- Modular ROS2-based architecture
- Comprehensive hardware integration

## System Overview

The project integrates several key components:
- Kinova Gen3 robotic arm control
- Custom bistable finger structures
- Tactile sensing system
- RealSense D435 camera integration
- Arduino-based force feedback
- ROS2 control framework

## Prerequisites

⚠️ **Important System Requirements**
- This project was developed and tested on Ubuntu.
- **Native Ubuntu Users**: For full RealSense camera compatibility, it is critical to boot into the **6.08-generic** kernel. The camera drivers have been found to be most stable on this specific version.
- Other operating systems may work with Docker but are not officially tested.

### Software Requirements ✅
- Docker (v20.10 or higher)
- VS Code with Dev Containers extension
- Ubuntu-based system with:
  - X11 for GUI applications
  - USB support for hardware devices
  - Network capabilities for ROS2 communication

### First Time Setup Checklist 📋
1. [ ] Install Docker and VS Code
2. [ ] Clone this repository
3. [ ] Set up udev rules for Arduino devices
4. [ ] Configure network settings for robot
5. [ ] Build the workspace
6. [ ] Test basic functionality

## Hardware Requirements

### Required Hardware 🔧
- Kinova Gen3 6-DOF robotic arm
- Intel RealSense D435 camera
- Arduino boards (2x):
  - One for servo force control (Arduino Uno/Mega)
  - One for tactile sensing (Arduino Uno/Mega)
- Network connection for robot communication (Ethernet preferred)

### Hardware Setup Guide 🛠️
1. **Kinova Robot**
   - Ensure robot is properly mounted and powered
   - Connect ethernet cable directly to your computer
   - Note down the robot's IP address (default: 192.168.1.10)

2. **Arduino Boards**
   - Connect both Arduino boards via USB
   - Note which port is assigned to each board
   - Follow udev rules setup in section below

3. **RealSense Camera**
   - Mount camera securely near the workspace
   - Connect via USB 3.0 port for best performance
   - Ensure clear view of the manipulation area

## Project Structure

```
.
├── src/
│   ├── arduino_servo_force_publisher/    # Arduino force feedback integration
│   ├── gripper_control/                  # Gripper control implementation
│   ├── image_extractor_pkg/             # ROS2 package for image extraction and processing
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

#### Pre-container Setup

1. **Linux**: Enable X11 forwarding for GUI applications
   ```bash
   # Grant X server access to local user
   xhost +local:
   ```

2. **Windows with WSL2**:
   - Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or [GWSL](https://opticos.github.io/gwsl/)
   - Configure display export in WSL2:
     ```bash
     echo "export DISPLAY=:0" >> ~/.bashrc
     source ~/.bashrc
     ```

3. **macOS**:
   - Install [XQuartz](https://www.xquartz.org/)
   - Configure XQuartz:
     ```bash
     # Allow network connections
     defaults write org.xquartz.X11 nolisten_tcp 0
     # Restart XQuartz after changing settings
     ```

#### Start Development Container

1. Open the project in VS Code:
   ```bash
   code .
   ```

2. When prompted, click "Reopen in Container" 
   - Or use Command Palette (F1) and select "Dev Containers: Reopen in Container"
   - First build may take 10-15 minutes

#### Verify Container Setup

```bash
# Check ROS2 is available
ros2 --version

# Check GUI works
ros2 run rqt_graph rqt_graph
```

#### Verifying Submodule Setup

Check if all submodules are properly initialized:

```bash
# List all submodules and their status
git submodule status

# Expected output should show all submodules with their commit hashes
# If you see a '-' before the hash, the submodule needs initialization
```

Common submodule issues and fixes:
```bash
# If submodules are empty:
git submodule update --init --recursive

# If submodules are on wrong commit:
git submodule update --recursive

# To update all submodules to their latest versions:
git submodule update --remote --merge
```

### Working with Docker Terminals

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

### 2. First-Time Arduino Setup (Ubuntu Only)

⚠️ **One-Time Setup**: These steps are only required once on your Ubuntu system to configure USB permissions. You won't need to repeat this setup unless you reinstall your system.

#### Step-by-Step Arduino Configuration (Skip if already configured):

1. **Find Device Information**:
   ```bash
   # Get device attributes for your Arduino
   udevadm info -a -n /dev/ttyACM0 | grep -E "ATTRS{idVendor}|ATTRS{idProduct}|ATTRS{serial}"
   
   # If your Arduino is on a different port, replace ttyACM0 with the correct port
   # Common alternatives: ttyACM1, ttyUSB0, ttyUSB1

2. **Create udev Rules File**:
   ```bash
   sudo nano /etc/udev/rules.d/99-arduino.rules
   ```

3. **Add These Rules** (adjust vendor/product IDs if different):
   ```bash
   # For Servo Control Arduino
   SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="ttyServo", MODE="0666"
   # For Tactile Sensor Arduino
   SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="ttyTactile", MODE="0666"
   ```

4. **Apply and Verify Rules**:
   ```bash
   # Reload rules
   sudo udevadm control --reload-rules && sudo udevadm trigger

   # Verify symlinks were created
   ls -l /dev/ttyServo
   ls -l /dev/ttyTactile
   ```

5. **Troubleshooting**:
   - If symlinks aren't created, unplug and replug the Arduino boards
   - Check if IDs match your devices using the command from step 1
   - Ensure no permission errors with: `ls -l /dev/ttyACM*`
```

### 3. Build Workspace

Inside the Docker container:

```bash
cd /workspaces/lairlab_kinova_bistable_structures
colcon build
source install/setup.bash
```

## Running the System

### System Startup Sequence 🚀

Follow these steps in order to ensure proper system initialization:

1. **Start the Robot Control Node**:
   ```bash
   # Make sure the robot is powered on and ethernet is connected
   ros2 launch kortex_bringup gen3.launch.py robot_ip:=192.168.1.10 dof:=6
   
   # Wait for successful connection message
   # You should see "Robot is ready to use" in the terminal
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

### Project Team
- [Sudhaavan K.](https://github.com/brucewayne7777) - Research Intern
- [Yechan Kwon](https://github.com/cks2904) - Research Intern
### Project Supervisors
- [Dr. Christopher Yee Wong](https://www.concordia.ca/faculty/christopheryee-wong.html) - Assistant Professor
  - Concordia University, Department of Mechanical, Industrial and Aerospace Engineering
  - Director, Laboratory for Artificial Intelligence in Robotics (LAIR)

- [Dr. Hang Xu](https://www.concordia.ca/faculty/hang-xu.html) - Assistant Professor
  - Concordia University, Department of Mechanical, Industrial and Aerospace Engineering
  - Principal Investigator, Applied Mechanics and Design Lab