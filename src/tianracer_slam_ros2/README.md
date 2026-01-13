# Tianracer SLAM ROS2 Package

这是 Tianracer SLAM 包的 ROS2 版本，支持多种 SLAM 算法进行建图。

## 依赖包

### 必需依赖

1. **ROS2 核心包**（通常已安装）：
   ```bash
   sudo apt install ros-humble-rclcpp ros-humble-rclpy
   sudo apt install ros-humble-sensor-msgs ros-humble-nav-msgs ros-humble-geometry-msgs
   sudo apt install ros-humble-tf2 ros-humble-tf2-ros
   ```

2. **SLAM 算法包**（根据使用的算法选择）：

   **对于 Gmapping（推荐使用 slam_toolbox）：**
   ```bash
   sudo apt install ros-humble-slam-toolbox
   ```

   **对于 Cartographer：**
   ```bash
   sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
   ```

   **对于 Hector SLAM：**
   - 注意：Hector SLAM 在 ROS2 中可能不可用
   - 替代方案：使用 `slam_toolbox` 或检查是否有 ROS2 移植版本

3. **地图服务器**：
   ```bash
   sudo apt install ros-humble-nav2-map-server
   ```

4. **可视化工具**：
   ```bash
   sudo apt install ros-humble-rviz2
   ```

### 可选依赖

- `tianracer_rviz` - 用于 RViz 配置文件（如果存在）

## 安装和编译

1. **进入工作空间**：
   ```bash
   cd /home/lucifer/tianracer_ws
   ```

2. **编译包**：
   ```bash
   colcon build --packages-select tianracer_slam_ros2
   ```

3. **Source 工作空间**：
   ```bash
   source install/setup.bash
   ```

## 使用方法

### 1. 使用 SLAM Toolbox（替代 Gmapping）

```bash
ros2 launch tianracer_slam_ros2 tianracer_gmapping.launch.py
```

### 2. 使用 Cartographer（2D）

```bash
ros2 launch tianracer_slam_ros2 tianracer_cartographer.launch.py
```

### 3. 使用 Cartographer（3D 点云）

```bash
ros2 launch tianracer_slam_ros2 tianracer_3d_cartographer.launch.py
```

### 4. 使用 Hector SLAM

```bash
ros2 launch tianracer_slam_ros2 tianracer_hector.launch.py
```

**注意**：Hector SLAM 在 ROS2 中可能不可用，如果报错，请使用 `slam_toolbox`。

### 5. 保存地图

有两种方式保存地图：

#### 方式 1: 使用 map_saver_server（推荐，持续运行）

启动地图保存服务节点：

```bash
ros2 launch tianracer_slam_ros2 map_save.launch.py
```

然后通过服务调用保存地图：

```bash
# 对于非 Cartographer 算法
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_url: '/path/to/maps/tianbot_office', image_format: 'pgm', map_mode: 'trinary', free_thresh: 0.19, occupied_thresh: 0.65}"

# 对于 Cartographer
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_url: '/path/to/maps/tianbot_office', image_format: 'pgm', map_mode: 'trinary', free_thresh: 0.49, occupied_thresh: 0.51}"
```

**优点**：
- 持续运行，可以多次保存地图
- 更符合 ROS2 服务化设计
- 类似 ROS1 的 `map_saver` 节点行为

#### 方式 2: 使用 map_saver_cli（一次性执行）

```bash
# 对于非 Cartographer 算法
ros2 launch tianracer_slam_ros2 map_save_cli.launch.py slam_methods:=gmapping

# 对于 Cartographer
ros2 launch tianracer_slam_ros2 map_save_cli.launch.py slam_methods:=cartographer
```

**优点**：
- 简单直接，执行一次保存后退出
- 适合脚本自动化

## 验证步骤

### 1. 检查包是否正确安装

```bash
# 检查包是否存在
ros2 pkg list | grep tianracer_slam_ros2

# 检查包路径
ros2 pkg prefix tianracer_slam_ros2

# 列出所有可执行文件/launch文件
ros2 pkg executables tianracer_slam_ros2
```

### 2. 检查依赖包

```bash
# 检查 SLAM Toolbox
ros2 pkg list | grep slam_toolbox

# 检查 Cartographer
ros2 pkg list | grep cartographer

# 检查 Nav2 Map Server
ros2 pkg list | grep nav2_map_server
```

### 3. 测试 Launch 文件语法

```bash
# 检查 launch 文件语法（不会实际运行）
ros2 launch tianracer_slam_ros2 tianracer_gmapping.launch.py --show-args
```

### 4. 验证话题和 TF

启动 SLAM 后，在另一个终端检查：

```bash
# 检查发布的话题
ros2 topic list

# 应该看到：
# /map (nav_msgs/msg/OccupancyGrid)
# /scan (sensor_msgs/msg/LaserScan) - 如果激光雷达在运行

# 检查 TF 树
ros2 run tf2_tools view_frames

# 检查特定变换
ros2 run tf2_ros tf2_echo map odom
```

### 5. 可视化验证

```bash
# 启动 RViz2
rviz2

# 添加以下显示：
# - Map (话题: /map)
# - LaserScan (话题: /scan)
# - TF (显示坐标系)
```

### 6. 完整测试流程

1. **启动机器人驱动**（如果使用真实机器人）：
   ```bash
   ros2 launch tianbot_core_ros2 tianbot_core.launch
   ```

2. **启动 SLAM**：
   ```bash
   ros2 launch tianracer_slam_ros2 tianracer_gmapping.launch.py
   ```

3. **启动 RViz2**：
   ```bash
   rviz2
   ```

4. **检查输出**：
   - 应该能看到地图逐渐构建
   - `/scan` 数据应该正确显示
   - TF 树应该完整（map → odom → base_footprint → laser）

5. **保存地图**（在另一个终端）：
   ```bash
   ros2 launch tianracer_slam_ros2 map_save.launch.py
   ```

## 常见问题

### 问题 1: 找不到 slam_toolbox 包

**解决方案**：
```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

### 问题 2: 找不到 cartographer_ros 包

**解决方案**：
```bash
sudo apt update
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
```

### 问题 3: 找不到 nav2_map_server

**解决方案**：
```bash
sudo apt update
sudo apt install ros-humble-nav2-map-server
```

### 问题 4: Launch 文件报错 "No module named 'launch'"

**解决方案**：
```bash
sudo apt install python3-ros2launch
```

### 问题 5: 地图不更新或 /scan 数据不显示

**检查项**：
1. 确认 `/scan` 话题有数据：`ros2 topic echo /scan`
2. 检查 TF 树是否完整：`ros2 run tf2_tools view_frames`
3. 确认 SLAM 节点正在运行：`ros2 node list`

### 问题 6: Hector SLAM 不可用

**解决方案**：
- 使用 `slam_toolbox` 替代（已在 `tianracer_gmapping.launch.py` 中配置）
- 或检查是否有 ROS2 版本的 hector_slam

### 问题 7: Message Filter 队列满错误

**错误信息**：
```
[sync_slam_toolbox_node]: Message Filter dropping message: frame 'laser' at time ... for reason 'discarding message because the queue is full'
```

**原因分析**：

1. **TF 变换缺失或延迟**（最常见）：
   - `laser` 帧的 TF 变换不可用或延迟
   - TF 树不完整（缺少 `base_footprint → laser` 变换）
   - TF 时间戳不同步

2. **消息队列太小**：
   - 默认队列大小可能不足以处理高频激光数据
   - 消息处理速度跟不上接收速度

3. **计算资源不足**：
   - CPU 处理能力不足，无法及时处理消息
   - 导致消息积压

**影响**：

- **地图质量下降**：丢失的扫描数据会导致地图不完整
- **定位精度降低**：缺少数据会影响位姿估计
- **建图失败**：严重时可能导致建图完全失败

**解决方案**：

1. **检查 TF 树**：
   ```bash
   # 查看 TF 树
   ros2 run tf2_tools view_frames
   
   # 检查特定变换是否存在
   ros2 run tf2_ros tf2_echo base_footprint laser
   
   # 实时监控 TF
   ros2 topic echo /tf
   ```

2. **确保静态 TF 发布**：
   ```bash
   # 如果缺少 base_footprint → laser 变换，需要发布静态 TF
   ros2 run tf2_ros static_transform_publisher 0.15 0.0 0.0 0.0 0.0 0.0 base_footprint laser
   ```

3. **增加队列大小**（已在 launch 文件中配置）：
   - `message_filter_queue_size: 50` - 增加消息队列大小
   - `transform_tolerance: 0.5` - 增加 TF 查找时间容差
   - `tf_buffer_duration: 10.0` - 增加 TF 缓冲区持续时间

4. **降低激光频率**（如果可能）：
   - 在激光驱动节点中降低发布频率
   - 使用 `laser_filters` 包进行降采样

5. **检查系统性能**：
   ```bash
   # 监控 CPU 使用率
   htop
   
   # 检查消息频率
   ros2 topic hz /scan
   ```

6. **使用异步模式**（如果问题持续）：
   - 考虑使用 `async_slam_toolbox_node` 替代 `sync_slam_toolbox_node`
   - 异步模式对 TF 延迟更宽容

## 配置文件

- `param/2d_scan.lua` - Cartographer 2D 配置
- `param/3d_points.lua` - Cartographer 3D 配置

这些配置文件不需要修改，它们会被自动安装到正确的位置。

## 与 ROS1 版本的主要区别

1. **Launch 文件格式**：从 XML 改为 Python
2. **构建系统**：从 catkin 改为 ament_cmake
3. **包格式**：从 format="2" 改为 format="3"
4. **SLAM 算法**：
   - Gmapping → slam_toolbox（ROS2 推荐）
   - Cartographer → cartographer_ros（有官方 ROS2 版本）
   - Hector → 可能需要替代方案
5. **地图保存**：使用 `nav2_map_server` 的 `map_saver_cli`

## 许可证

TODO

