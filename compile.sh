#!/usr/bin/bash

# 编译项目所需的包
# 注意：realsense2_camera_msgs 必须在 realsense2_camera 之前编译

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检测 Ubuntu 版本
if [ -f /etc/os-release ]; then
    . /etc/os-release
    UBUNTU_VERSION=$(echo $VERSION_ID | cut -d. -f1)
else
    echo "Warning: Cannot detect Ubuntu version, assuming 20.04"
    UBUNTU_VERSION=20
fi

# 定义所有可用的包
ALL_PACKAGES=(
  librealsense2
  ackermann_msgs
  realsense2_camera_msgs
  realsense2_camera
  slam_toolbox
  tianbot_core_ros2
  osight_lidar_ros2
  tianracer_teleop_ros2
  tianracer_slam_ros2
)

# 在 Ubuntu 22.04 上可以通过 apt 安装的包（ROS2 Humble）
# 这些包在 20.04 上需要源码编译
APT_AVAILABLE_PACKAGES_22_04=(
  librealsense2
  ackermann_msgs
  slam_toolbox
  realsense2_camera_msgs
  realsense2_camera
)

# 必须源码编译的包（无论哪个版本）
# - tianbot_core_ros2: 自定义包
# - osight_lidar_ros2: 自定义包
# - tianracer_teleop_ros2: 自定义包
# - tianracer_slam_ros2: 自定义包
MUST_BUILD_PACKAGES=(
  tianbot_core_ros2
  osight_lidar_ros2
  tianracer_teleop_ros2
  tianracer_slam_ros2
)

# 根据 Ubuntu 版本决定编译哪些包
if [ "$UBUNTU_VERSION" = "22" ]; then
    # Ubuntu 22.04: 只编译无法通过 apt 安装的包
    echo -e "${BLUE}Detected Ubuntu 22.04 - Building only packages not available via apt${NC}"
    echo ""
    packages=("${MUST_BUILD_PACKAGES[@]}")
    
    # 检查并提示可以通过 apt 安装的包
    echo -e "${YELLOW}Note: The following packages can be installed via apt on Ubuntu 22.04:${NC}"
    for pkg in "${APT_AVAILABLE_PACKAGES_22_04[@]}"; do
        echo -e "${YELLOW}  - $pkg (install with: sudo apt install ros-humble-${pkg//_/-})${NC}"
    done
    echo ""
else
    # Ubuntu 20.04 或其他版本: 编译所有包
    echo -e "${BLUE}Detected Ubuntu 20.04 or other - Building all packages from source${NC}"
    packages=("${ALL_PACKAGES[@]}")
fi

echo -e "${GREEN}Packages to build: ${packages[*]}${NC}"
echo ""

colcon build --packages-select "${packages[@]}" \
  --symlink-install \
  --cmake-args \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GLSL_EXTENSIONS=OFF \
  -DBUILD_PC_STITCHING=OFF \
  -DBUILD_TOOLS=OFF \
  -DGLFW_USE_WAYLAND=OFF

source install/setup.bash