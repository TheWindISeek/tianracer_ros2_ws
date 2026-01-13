# Tianracer ROS2 完整指南

## 目录

1. [项目概述](#项目概述)
2. [依赖安装](#依赖安装)
3. [编译说明](#编译说明)
4. [使用说明](#使用说明)
5. [测试指南](#测试指南)
6. [故障排除](#故障排除)

---

## 项目概述

本项目包含以下ROS2包：

- **tianracer_teleop_ros2** - 小车遥控功能包
  - `tianracer_joy.py` - 手柄/键盘控制节点，发布Ackermann格式的控制命令
  - `cmd_vel_to_ackermann.py` - cmd_vel到Ackermann的转换节点，用于补偿前轮歪斜问题

- **realsense2_camera** - Intel RealSense摄像头驱动
- **tianbot_core_ros2** - 天博核心通信包
- **osight_lidar_ros2** - 激光雷达驱动包

---

## 依赖安装

### 一、系统级依赖（必须从源码编译）

#### 1. librealsense2 (Intel RealSense SDK 2.0)

**下载地址：** https://github.com/IntelRealSense/librealsense  
**推荐版本：** v2.57.0 或最新稳定版

**安装步骤：**

```bash
# 1. 安装基础依赖（ROS2使用，禁用图形功能）
sudo apt-get update
sudo apt-get install -y \
    libusb-1.0-0-dev \
    libssl-dev \
    libudev-dev \
    pkg-config \
    cmake \
    git \
    dkms \
    udev

# 2. 克隆源码
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense

# 3. 切换到稳定版本
git fetch --tags
LATEST_TAG=$(git tag | grep -E '^v?2\.' | sort -V | tail -1)
git checkout $LATEST_TAG  # 或使用 git checkout v2.57.0

# 4. 编译安装（禁用图形功能，ROS2不需要）
mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_GLSL_EXTENSIONS=OFF \
    -DBUILD_PC_STITCHING=OFF

make -j$(nproc)
sudo make install

# 5. 更新库缓存
sudo ldconfig

# 6. 添加USB规则
sudo cp ~/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

# 7. 验证安装
pkg-config --modversion librealsense2
```

**如果CMake找不到librealsense2：**

```bash
# 查找配置文件
find /usr/local -name "realsense2Config.cmake" 2>/dev/null

# 设置环境变量（永久）
echo 'export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH' >> ~/.bashrc
source ~/.bashrc
```

#### 2. OpenCV

**下载地址：** https://github.com/opencv/opencv  
**推荐版本：** 4.8.0 或最新稳定版

**安装步骤：**

```bash
# 1. 安装依赖
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libatlas-base-dev \
    python3-dev \
    python3-numpy

# 2. 克隆源码
cd ~
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.8.0  # 或使用最新稳定版

# 3. 编译安装
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install

# 4. 更新库缓存
sudo ldconfig
```

### 二、ROS2依赖包

#### 必须安装的包

##### 1. cv_bridge

**用途：** OpenCV与ROS2图像消息之间的桥接库  
**依赖关系：** `realsense2_camera` 包必需  
**源码下载：** https://github.com/ros-perception/vision_opencv

```bash
# 创建工作空间（如果还没有）
mkdir -p ~/ros2_deps_ws/src
cd ~/ros2_deps_ws/src

# 克隆源码
git clone https://github.com/ros-perception/vision_opencv.git
cd vision_opencv
# 切换到对应ROS2版本分支（根据您的ROS2版本）
# git checkout humble  # 或 iron, rolling 等
cd ..

# 编译
cd ~/ros2_deps_ws
colcon build --packages-select cv_bridge
source install/setup.bash
```

##### 2. diagnostic_updater

**用途：** 诊断信息更新器，用于发布设备状态  
**依赖关系：** `realsense2_camera` 包必需  
**源码下载：** https://github.com/ros/diagnostics

```bash
cd ~/ros2_deps_ws/src
git clone https://github.com/ros/diagnostics.git
cd diagnostics
# 切换到对应ROS2版本分支
# git checkout ros2  # 或对应版本分支
cd ..

# 编译
cd ~/ros2_deps_ws
colcon build --packages-select diagnostic_updater
source install/setup.bash
```

#### 可选包

##### 3. ackermann_msgs

**用途：** Ackermann底盘控制消息定义  
**依赖关系：** 仅在使用Ackermann底盘时需要  
**源码下载：** https://github.com/ros-drivers/ackermann_msgs

```bash
cd ~/ros2_deps_ws/src
git clone https://github.com/ros-drivers/ackermann_msgs.git -b ros2

# 编译
cd ~/ros2_deps_ws
colcon build --packages-select ackermann_msgs
source install/setup.bash
```

#### ROS2标准包（通常通过apt安装）

```bash
# 安装基础依赖
sudo apt install ros-<distro>-joy ros-<distro>-teleop-twist-keyboard

# 如果ackermann_msgs不在workspace中，也可以通过apt安装（如果可用）
sudo apt install ros-<distro>-ackermann-msgs
```

**注意：** 以下包通常已包含在ROS2基础安装中，无需额外安装：
- rclcpp, geometry_msgs, nav_msgs, sensor_msgs, std_msgs
- tf2, tf2_ros, image_transport
- builtin_interfaces, lifecycle_msgs, rclcpp_components

### 三、快速安装脚本

创建 `install_dependencies.sh`：

```bash
#!/bin/bash
set -e

echo "=== 安装系统依赖 ==="
sudo apt-get update
sudo apt-get install -y \
    libusb-1.0-0-dev libssl-dev libudev-dev pkg-config cmake git dkms udev \
    build-essential libgtk-3-dev libavcodec-dev libavformat-dev \
    libswscale-dev libv4l-dev libxvidcore-dev libx264-dev \
    libjpeg-dev libpng-dev libtiff-dev libatlas-base-dev \
    python3-dev python3-numpy

echo "=== 安装librealsense2 ==="
cd ~
if [ ! -d "librealsense" ]; then
    git clone https://github.com/IntelRealSense/librealsense.git
fi
cd librealsense
git fetch --tags
LATEST_TAG=$(git tag | grep -E '^v?2\.' | sort -V | tail -1)
git checkout $LATEST_TAG
mkdir -p build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_GLSL_EXTENSIONS=OFF \
    -DBUILD_PC_STITCHING=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
sudo cp ~/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

echo "=== 安装OpenCV ==="
cd ~
if [ ! -d "opencv" ]; then
    git clone https://github.com/opencv/opencv.git
fi
cd opencv
git checkout 4.8.0
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig

echo "=== 安装ROS2依赖包 ==="
mkdir -p ~/ros2_deps_ws/src
cd ~/ros2_deps_ws/src
git clone https://github.com/ros-perception/vision_opencv.git
git clone https://github.com/ros/diagnostics.git
git clone https://github.com/ros-drivers/ackermann_msgs.git -b ros2

cd ~/ros2_deps_ws
colcon build --packages-select cv_bridge diagnostic_updater ackermann_msgs
source install/setup.bash

echo "=== 验证安装 ==="
pkg-config --modversion librealsense2
ros2 pkg list | grep -E "cv_bridge|diagnostic_updater|ackermann_msgs"

echo "=== 安装完成 ==="
```

---

## 编译说明

### 编译前准备

1. **确保所有依赖已安装**（参考上一节）
2. **设置环境变量**

```bash
# 如果依赖包在单独工作空间
source ~/ros2_deps_ws/install/setup.bash

# Source ROS2环境
source /opt/ros/<distro>/setup.bash
```

### 编译命令

```bash
cd /home/lucifer/tianracer_ros2_ws

# 方法1: 使用提供的编译脚本（推荐）
./compile.sh

# 方法2: 手动编译
source rm.sh  # 清理之前的编译
colcon build --packages-select \
    realsense2_camera_msgs \
    realsense2_camera \
    tianbot_core_ros2 \
    osight_lidar_ros2 \
    tianracer_teleop_ros2 \
    --symlink-install

# 设置环境
source install/setup.bash

# 或者添加到 ~/.bashrc
echo "source /home/lucifer/tianracer_ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 编译顺序

colcon会自动处理依赖关系，按以下顺序编译：

```
realsense2_camera_msgs (消息包)
    ↓
realsense2_camera (主驱动包，依赖消息包)
    ↓
tianbot_core_ros2 (独立包)
osight_lidar_ros2 (独立包)
tianracer_teleop_ros2 (独立包)
```

### 不需要编译的包

以下包在 `realsense-ros` 仓库中，但**不需要编译**：
- `realsense2_description` - 仅包含URDF描述文件
- `realsense2_rgbd_plugin` - RGBD插件，本项目不需要
- `realsense2_ros_mqtt_bridge` - MQTT桥接，本项目不需要

---

## 使用说明

### 遥控功能使用

#### 方案1: 使用cmd_vel_to_ackermann补偿节点（推荐）

适用于所有控制命令都需要补偿前轮歪斜的情况。

**启动补偿节点：**

```bash
# 使用launch文件启动（推荐）
ros2 launch tianracer_teleop_ros2 cmd_vel_to_ackermann.launch.py

# 或者自定义参数
ros2 launch tianracer_teleop_ros2 cmd_vel_to_ackermann.launch.py \
    input_topic:=/cmd_vel \
    output_topic:=/tianracer/ackermann_cmd \
    wheelbase:=0.26 \
    steering_offset_degrees:=28.65
```

**启动手柄控制：**

```bash
# 启动手柄控制节点
ros2 launch tianracer_teleop_ros2 joystick_teleop.launch.py

# 或者自定义参数
ros2 launch tianracer_teleop_ros2 joystick_teleop.launch.py \
    joy_mode:=D \
    throttle_scale:=1.0 \
    servo_scale:=1.0
```

**使用键盘控制：**

```bash
# 安装键盘控制包（如果未安装）
sudo apt install ros-<distro>-teleop-twist-keyboard

# 启动键盘控制（发布到/cmd_vel，会被补偿节点转换）
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### 方案2: 直接使用手柄控制（不使用补偿）

如果不需要补偿，可以直接使用手柄控制：

```bash
ros2 launch tianracer_teleop_ros2 joystick_teleop.launch.py
```

手柄控制节点会直接发布到 `/tianracer/ackermann_cmd` 话题。

### 话题说明

**输入话题：**
- `/cmd_vel` (geometry_msgs/msg/Twist) - 标准速度控制命令
- `/joy` (sensor_msgs/msg/Joy) - 手柄输入

**输出话题：**
- `/tianracer/ackermann_cmd` (ackermann_msgs/msg/AckermannDrive) - Ackermann格式的控制命令

### 参数说明

**cmd_vel_to_ackermann节点参数：**
- `input_topic` (string, 默认: `/cmd_vel`) - 输入话题
- `output_topic` (string, 默认: `/tianracer/ackermann_cmd`) - 输出话题
- `output_type` (string, 默认: `auto`) - 输出类型: `auto`, `twist`, `ackermann`
- `wheelbase` (double, 默认: `0.26`) - 轴距（米）
- `steering_offset_degrees` (double, 默认: `28.65`) - 舵机偏移角度（度），用于补偿前轮歪斜

**tianracer_joy节点参数：**
- `joy_mode` (string, 默认: `D`) - 手柄模式: `D` 或 `X`
- `throttle_scale` (double, 默认: `0.5`) - 油门缩放因子
- `servo_scale` (double, 默认: `1.0`) - 舵机缩放因子

### 工作流程

```
[手柄/键盘] 
    ↓
[/joy 或 /cmd_vel]
    ↓
[cmd_vel_to_ackermann] (补偿前轮歪斜)
    ↓
[/tianracer/ackermann_cmd]
    ↓
[小车控制节点]
```

---

## 测试指南

### RealSense摄像头测试

#### 1. 检查USB设备连接

```bash
lsusb | grep -i "intel\|realsense"
```

**预期结果：**
```
Bus 003 Device 004: ID 8086:0b5c Intel Corp. RealSense D455
```

#### 2. 检查设备权限

```bash
ls -la /dev/video* | grep -i "video"
```

**如果权限不对：**
```bash
sudo usermod -aG video $USER
newgrp video
```

#### 3. 启动ROS2节点

```bash
source /opt/ros/<distro>/setup.bash
source ~/tianracer_ros2_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py
```

**预期输出：**
```
[INFO] [realsense2_camera_node]: RealSense Node Is Up!
[INFO] [realsense2_camera_node]: Device with physical ID /dev/video2 was found.
[INFO] [realsense2_camera_node]: Device Name: Intel RealSense D455
```

#### 4. 检查发布的Topic

```bash
# 查看所有topic
ros2 topic list

# 应该看到：
# /camera/color/image_raw
# /camera/depth/image_rect_raw
# /camera/depth/color/points
# ...
```

#### 5. 使用rviz2可视化

```bash
rviz2
```

在rviz2中：
1. 点击 "Add" 按钮
2. 选择 "By topic"
3. 添加以下显示：
   - `/camera/color/image_raw` - 彩色图像
   - `/camera/depth/image_rect_raw` - 深度图像
   - `/camera/depth/color/points` - 点云（PointCloud2）

#### 6. 检查数据频率

```bash
# 检查彩色图像频率（应该是 30Hz）
ros2 topic hz /camera/color/image_raw

# 检查深度图像频率（应该是 30Hz）
ros2 topic hz /camera/depth/image_rect_raw
```

### 遥控功能调试

```bash
# 查看所有话题
ros2 topic list

# 查看Ackermann命令
ros2 topic echo /tianracer/ackermann_cmd

# 查看手柄输入
ros2 topic echo /joy

# 查看cmd_vel
ros2 topic echo /cmd_vel

# 查看运行中的节点
ros2 node list

# 查看节点信息
ros2 node info /cmd_vel_to_ackermann
ros2 node info /tianracer_joy_teleop
```

---

## 故障排除

### 编译问题

#### 问题1：找不到librealsense2

**错误信息：** `Could not find a package configuration file provided by "realsense2"`

**解决方法：**
```bash
# 检查是否已安装
pkg-config --modversion librealsense2
ls /usr/local/lib/librealsense2*

# 如果未安装，参考"依赖安装"章节安装
# 如果已安装但找不到，设置环境变量
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
echo 'export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH' >> ~/.bashrc
```

#### 问题2：找不到OpenCV或cv_bridge

**错误信息：** `OpenCV not found` 或 `cv_bridge not found`

**解决方法：**
```bash
# 检查OpenCV是否安装
pkg-config --modversion opencv4

# 如果未安装，参考"依赖安装"章节安装OpenCV
# 然后安装cv_bridge（参考"依赖安装"章节）
```

#### 问题3：ackermann_msgs警告

**警告信息：** `ackermann_msgs not found - Ackermann chassis support disabled`

**说明：** 这是正常的，只有在使用Ackermann底盘时才需要。如果需要，参考"依赖安装"章节安装。

### RealSense问题

#### 问题1：设备未识别

```bash
# 检查USB连接
lsusb

# 检查内核模块
lsmod | grep uvcvideo

# 重新加载模块
sudo modprobe -r uvcvideo
sudo modprobe uvcvideo
```

#### 问题2：权限问题

```bash
# 添加udev规则
sudo cp ~/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

# 添加用户到组
sudo usermod -aG video,dialout $USER
newgrp video
```

#### 问题3：USB 3.0问题

```bash
# 检查USB速度
dmesg | grep -i "usb.*speed"

# 应该看到 "SuperSpeed" 或 "480M"
# 如果速度低，尝试使用USB 3.0端口
```

#### 问题4：数据频率低或为0

- USB连接问题（可能不是USB 3.0）
- 供电不足
- 驱动问题

### 遥控功能问题

#### 问题1：节点无法启动

```bash
# 检查是否已source workspace
source install/setup.bash

# 检查依赖是否安装
ros2 pkg list | grep ackermann_msgs
ros2 pkg list | grep joy
```

#### 问题2：手柄无响应

```bash
# 检查手柄设备
ls -l /dev/tianbot_joystick

# 检查joy节点是否运行
ros2 node list | grep joy

# 检查话题是否有数据
ros2 topic echo /joy
```

#### 问题3：补偿不准确

- 调整`steering_offset_degrees`参数
- 检查`wheelbase`参数是否正确
- 查看节点日志：`ros2 run tianracer_teleop_ros2 cmd_vel_to_ackermann.py`

### 注意事项

1. **补偿节点必须启动**: 如果使用键盘控制或其他发布到`/cmd_vel`的控制方式，必须启动`cmd_vel_to_ackermann`节点进行补偿。

2. **手柄模式**: 根据你的手柄类型设置`joy_mode`参数：
   - `D` 模式：使用axis 2作为转向
   - `X` 模式：使用axis 3作为转向

3. **偏移角度调整**: 根据实际小车前轮的歪斜情况，调整`steering_offset_degrees`参数。

4. **轴距设置**: 根据实际小车轴距调整`wheelbase`参数。

---

## 快速参考

### 完整测试流程

1. **硬件检查**：`lsusb | grep -i intel`
2. **权限检查**：`ls -la /dev/video*`
3. **启动RealSense节点**：`ros2 launch realsense2_camera rs_launch.py`
4. **检查topic**：`ros2 topic list`
5. **可视化**：`rviz2` 并添加图像显示
6. **检查频率**：`ros2 topic hz /camera/color/image_raw`
7. **启动遥控**：`ros2 launch tianracer_teleop_ros2 joystick_teleop.launch.py`

### 验证安装

```bash
# 检查librealsense2
pkg-config --modversion librealsense2

# 检查OpenCV
pkg-config --modversion opencv4

# 检查ROS2包
ros2 pkg list | grep -E "cv_bridge|diagnostic_updater|ackermann_msgs|realsense2_camera|tianracer_teleop"
```

---

**最后更新：** 整合自多个文档，包含完整的安装、编译、使用和测试指南。

