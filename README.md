# Tianracer ROS2 Workspace

A ROS2 workspace for Tianracer autonomous vehicle platform, including teleoperation, SLAM, and sensor drivers.

## Overview

This workspace contains ROS2 packages for:
- **Teleoperation**: Joystick and keyboard control for the Tianracer
- **SLAM**: Mapping and localization using slam_toolbox
- **Sensors**: RealSense camera and LiDAR drivers
- **Core**: Tianbot core communication package

## Prerequisites

- ROS2 (Humble/Iron/Rolling)
- Ubuntu 20.04/22.04
- CMake 3.8+
- Git

## Quick Start

### 1. Clone the Repository

```bash
git clone git@github.com:TheWindISeek/tianracer_ros2_ws.git
cd tianracer_ros2_ws
```

### 2. Install Dependencies

Install system dependencies:
```bash
sudo apt-get update
sudo apt-get install -y \
    libusb-1.0-0-dev libssl-dev libudev-dev pkg-config cmake git \
    build-essential python3-dev python3-numpy
```

**Additional dependencies for slam_toolbox:**
```bash
sudo apt install \
    libboost-system-dev \
    libboost-serialization-dev \
    libboost-filesystem-dev \
    libboost-thread-dev
```

**ROS2 packages available via apt (Ubuntu 22.04 only):**

On Ubuntu 22.04 with ROS2 Humble, the following packages can be installed via apt instead of building from source:

```bash
sudo apt install \
    ros-humble-librealsense2 \
    ros-humble-ackermann-msgs \
    ros-humble-slam-toolbox \
    ros-humble-realsense2-camera-msgs \
    ros-humble-realsense2-camera
```

**Note:** On Ubuntu 20.04, these packages need to be built from source (see Build section below).

### 3. Download Source Repositories

This workspace uses ROS2 repos file format to manage dependencies. Download all source repositories:

```bash
# Using vcstool (recommended)
sudo apt-get install python3-vcstool
vcs import src < tianracer_ros2_ws.repos

# Or manually clone each repository according to tianracer_ros2_ws.repos
```

### 4. Build the Workspace

```bash
# Source ROS2 environment
source /opt/ros/<distro>/setup.bash

# Build (automatically detects Ubuntu version and skips apt-available packages on 22.04)
./compile.sh

# Or build manually
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

**Note:** The `compile.sh` script automatically detects your Ubuntu version:
- **Ubuntu 22.04**: Only builds custom packages (tianbot_core_ros2, osight_lidar_ros2, tianracer_teleop_ros2, tianracer_slam_ros2). Other packages should be installed via apt (see step 2).
- **Ubuntu 20.04**: Builds all packages from source.

## Repository Structure

The workspace contains the following packages:

- `tianracer_teleop_ros2` - Teleoperation package
- `tianracer_slam_ros2` - SLAM configuration and launch files
- `tianbot_core_ros2` - Core communication package
- `osight_lidar_ros2` - LiDAR driver
- `realsense-ros` - Intel RealSense camera driver
- `slam_toolbox` - SLAM toolkit
- `ackermann_msgs` - Ackermann drive message definitions
- `librealsense` - Intel RealSense SDK

## Dependencies

All dependency information is recorded in `tianracer_ros2_ws.repos` file, including:
- Repository URLs
- Branch/tag versions
- Commit hashes

To update dependencies to their latest versions:
```bash
vcs pull src
```

## Workspace Synchronization

The workspace includes a synchronization script (`sync_workspace.sh`) for syncing code between local and remote machines.

### Usage Examples

Sync to a remote machine:
```bash
./sync_workspace.sh -h 172.26.130.38 -u lucifer
```

Sync to another remote machine:
```bash
./sync_workspace.sh -h 202.199.7.15
```

For more options and detailed usage, run:
```bash
./sync_workspace.sh --help
```

## Usage

### Teleoperation

Start joystick control:
```bash
ros2 launch tianracer_teleop_ros2 joystick_teleop.launch.py
```

### SLAM

Start SLAM:
```bash
ros2 launch tianracer_slam_ros2 slam.launch.py
```

### RealSense Camera

Start RealSense camera:
```bash
ros2 launch realsense2_camera rs_launch.py
```

## Documentation

For detailed documentation, see [docs/GUIDE.md](docs/GUIDE.md).

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Acknowledgments

- Intel RealSense team for the camera drivers
- Steve Macenski for slam_toolbox
- ROS2 community

# hardware info
RoboMaster C620 

HP8-U45

STM32

AGX Xavier

atti/gps

| 参数 | 值 | 说明 |
|------|-----|------|
| wheelbase_ | 0.40m | 轴距 L |
| track_width_ | 0.27m | 轮距 W |
| front_overhang_ | 0.13m | 前悬 |
| rear_overhang_ | 0.07m | 后悬 |
| lidar_x_offset_ | 0.20m | 激光雷达在 base_link 前方的偏移 |

# TO-DO
当前的根本问题是如何解决小车在狭窄空间内的规划/移动和定位
这几个问题都是一体的

根本问题还是odom计数不准确，而amcl即使增加粒子数也定位不准确
再加上这个过程本身是不断旋转，导致误差不断累计，而这个功能就是为了将小车完全倒到完全相反的方向，故而尤其难以使用。

但是在仿真里面这个算法就没有什么问题，无非是仿真里面的这个图窄了一点，不够智能。
但是换了一个空旷的地方，这个就搞得定了

后续的思路是基于摄像头做点东西，但是好像图像没法传过来，估计只能先到那边进行一些处理了再到这边想办法。