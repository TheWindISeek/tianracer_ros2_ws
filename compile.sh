#!/usr/bin/bash

# =============================================================================
# 编译项目所需的包
# =============================================================================
# 
# 【重要依赖关系说明】
# 1. realsense2_camera_msgs 必须在 realsense2_camera 之前编译
#    - 如果注释掉 realsense2_camera_msgs，realsense2_camera 会编译失败
# 2. librealsense2 必须在 realsense2_camera 之前编译
#    - 如果注释掉 librealsense2，realsense2_camera 会编译失败
# 3. ackermann_msgs 必须在 tianbot_core_ros2 之前编译（可选依赖）
#    - 如果注释掉 ackermann_msgs，tianbot_core_ros2 仍可编译但会缺少 Ackermann 支持
# 4. realsense2_camera_msgs 必须在 realsense2_description 之前编译
#    - 如果注释掉 realsense2_camera_msgs，realsense2_description 会编译失败
# 
# 【编译顺序说明】
# 基础依赖包（必须按顺序）:
#   librealsense2 -> ackermann_msgs -> realsense2_camera_msgs -> realsense2_camera -> realsense2_description
# 
# 自定义包（可并行编译，但依赖基础包）:
#   tianbot_core_ros2 (依赖 ackermann_msgs)
#   osight_lidar_ros2
#   tianracer_teleop_ros2
#   tianracer_slam_ros2 (依赖 slam_toolbox)
#   tianracer_navigation_ros2
#   tianbot_description_ros2
#   tianracer_bringup_ros2
#   map2gazebo
# 
# 【注释掉会影响编译的部分】
# - 如果注释掉整个 ALL_PACKAGES 数组定义，脚本会报错
# - 如果注释掉 Ubuntu 版本检测部分（第49-57行），默认会按 Ubuntu 20.04 处理
# - 如果注释掉 colcon build 命令（第146-154行），不会进行任何编译
# - 如果注释掉 source install/setup.bash（第159行），编译后需要手动 source
# - 如果注释掉颜色定义（第41-47行），输出会没有颜色但功能正常
# - 如果注释掉 --symlink-install 参数，会使用普通安装而非符号链接
# - 如果注释掉 --cmake-args 及其参数（第148-154行），librealsense2 可能会编译更多不需要的组件
# =============================================================================

# 颜色定义
# 【注意】注释掉这部分不会影响编译，但输出会没有颜色提示
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 检测 Ubuntu 版本
# 【注意】如果注释掉这部分，脚本会假设是 Ubuntu 20.04，可能导致在 22.04 上编译不必要的包
if [ -f /etc/os-release ]; then
    . /etc/os-release
    UBUNTU_VERSION=$(echo $VERSION_ID | cut -d. -f1)
else
    echo "Warning: Cannot detect Ubuntu version, assuming 20.04"
    UBUNTU_VERSION=20
fi

# 定义所有可用的包
# 【注意】包的顺序很重要！依赖包必须在被依赖包之前
# 【警告】如果注释掉某个包，依赖它的包可能会编译失败
ALL_PACKAGES=(
  # 基础依赖包（必须按顺序）
  librealsense2              # Intel RealSense SDK - realsense2_camera 的依赖
  ackermann_msgs             # Ackermann 消息定义 - tianbot_core_ros2 的可选依赖
  realsense2_camera_msgs     # RealSense 消息定义 - 必须在 realsense2_camera 之前
  realsense2_camera          # RealSense 相机驱动 - 依赖 librealsense2 和 realsense2_camera_msgs
  realsense2_description     # RealSense URDF 描述 - 依赖 realsense2_camera_msgs
  slam_toolbox               # SLAM 工具箱 - tianracer_slam_ros2 的依赖
  
  # 自定义核心包
  tianbot_core_ros2          # 天博核心通信包 - 可选依赖 ackermann_msgs
  osight_lidar_ros2          # 激光雷达驱动包
  tianbot_description_ros2   # 机器人 URDF 描述包
  
  # 应用层包
  tianracer_teleop_ros2      # 遥控操作包
  tianracer_slam_ros2        # SLAM 配置包 - 依赖 slam_toolbox
  tianracer_navigation_ros2  # 导航配置包
  tianracer_bringup_ros2     # 启动配置包
  map2gazebo                 # 地图转 Gazebo 工具
)

# 在 Ubuntu 22.04 上可以通过 apt 安装的包（ROS2 Humble）
# 这些包在 20.04 上需要源码编译
# 【注意】如果注释掉这个数组，Ubuntu 22.04 上也会尝试编译这些包（虽然可以通过 apt 安装）
APT_AVAILABLE_PACKAGES_22_04=(
  librealsense2
  ackermann_msgs
  slam_toolbox
  realsense2_camera_msgs
  realsense2_camera
  realsense2_description
)

# 必须源码编译的包（无论哪个版本）
# 【注意】这些是自定义包，无法通过 apt 安装，必须源码编译
# 【警告】如果注释掉某个包，相关功能将不可用
MUST_BUILD_PACKAGES=(
  tianbot_core_ros2          # 自定义包 - 核心通信功能
  osight_lidar_ros2          # 自定义包 - 激光雷达驱动
  tianbot_description_ros2  # 自定义包 - 机器人描述
  tianracer_teleop_ros2      # 自定义包 - 遥控操作
  tianracer_slam_ros2        # 自定义包 - SLAM 配置
  tianracer_navigation_ros2 # 自定义包 - 导航配置
  tianracer_bringup_ros2    # 自定义包 - 启动配置
  map2gazebo                 # 自定义包 - 地图工具
)

# 根据 Ubuntu 版本决定编译哪些包
# 【注意】如果注释掉整个 if-else 块，需要手动设置 packages 变量，否则脚本会报错
if [ "$UBUNTU_VERSION" = "22" ]; then
    # Ubuntu 22.04: 只编译无法通过 apt 安装的包
    # 【注意】如果注释掉这部分，Ubuntu 22.04 上也会编译所有包（包括可通过 apt 安装的）
    echo -e "${BLUE}Detected Ubuntu 22.04 - Building only packages not available via apt${NC}"
    echo ""
    packages=("${MUST_BUILD_PACKAGES[@]}")
    
    # 检查并提示可以通过 apt 安装的包
    # 【注意】注释掉这部分不会影响编译，只是不会显示提示信息
    echo -e "${YELLOW}Note: The following packages can be installed via apt on Ubuntu 22.04:${NC}"
    for pkg in "${APT_AVAILABLE_PACKAGES_22_04[@]}"; do
        echo -e "${YELLOW}  - $pkg (install with: sudo apt install ros-humble-${pkg//_/-})${NC}"
    done
    echo ""
else
    # Ubuntu 20.04 或其他版本: 编译所有包
    # 【注意】如果注释掉这部分，非 22.04 系统会报错（packages 变量未定义）
    echo -e "${BLUE}Detected Ubuntu 20.04 or other - Building all packages from source${NC}"
    packages=("${ALL_PACKAGES[@]}")
fi

# 显示要编译的包列表
# 【注意】注释掉这部分不会影响编译，只是不会显示包列表
echo -e "${GREEN}Packages to build: ${packages[*]}${NC}"
echo ""

# 执行编译命令
# 【警告】这是核心编译命令，注释掉整个 colcon build 命令将不会进行任何编译
# 【重要参数说明】
#   --packages-select: 选择要编译的包，如果注释掉会编译所有包（可能包含不需要的包）
#   --symlink-install: 使用符号链接安装，修改源码后无需重新编译
#                     如果注释掉，会使用普通安装，每次修改都需要重新编译
#   --cmake-args: 传递给 CMake 的参数，主要用于 librealsense2 的编译选项
#                 如果注释掉，librealsense2 可能会编译更多不需要的组件，编译时间更长
colcon build --packages-select "${packages[@]}" \
  --symlink-install \
  --cmake-args \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GLSL_EXTENSIONS=OFF \
  -DBUILD_PC_STITCHING=OFF \
  -DBUILD_TOOLS=OFF \
  -DGLFW_USE_WAYLAND=OFF

# Source 编译后的工作空间
# 【注意】如果注释掉这行，编译后需要手动执行: source install/setup.bash
# 【警告】如果不 source，ROS2 将无法找到刚编译的包
source install/setup.bash