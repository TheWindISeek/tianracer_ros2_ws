# 地图使用和 Nav2 导航指南

本指南介绍如何使用已保存的地图进行可视化和 Nav2 导航。

## 目录

1. [地图可视化](#地图可视化)
2. [Nav2 导航配置](#nav2-导航配置)
3. [导航节点运行位置](#导航节点运行位置)
4. [完整使用流程](#完整使用流程)
5. [常见问题](#常见问题)

---

## 地图可视化

### 方法 1: 使用 map_load.launch.py（推荐）

这是最简单的方法，直接加载地图并发布到 `/map` 话题。

```bash
# 使用默认地图路径（~/tianracer_ros2_ws/my_map.yaml）
ros2 launch tianracer_slam_ros2 map_load.launch.py

# 或指定自定义地图路径
ros2 launch tianracer_slam_ros2 map_load.launch.py map_file:=/path/to/your/map.yaml
```

### 方法 2: 手动启动 map_server

```bash
# 启动 map_server 节点
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/lucifer/tianracer_ros2_ws/my_map.yaml

# 在另一个终端激活节点
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

### 在 RViz2 中可视化地图

1. **启动 RViz2**：
   ```bash
   rviz2
   ```

2. **添加地图显示**：
   - 点击 "Add" 按钮
   - 选择 "By topic"
   - 找到 `/map` 话题
   - 添加 "Map" 显示

3. **设置固定坐标系**：
   - 在 "Global Options" 中，将 "Fixed Frame" 设置为 `map`

4. **调整地图显示**：
   - 在 Map 显示中，可以调整颜色方案和透明度
   - 默认情况下，黑色表示障碍物，白色表示自由空间，灰色表示未知区域

### 检查地图是否正确加载

```bash
# 检查 /map 话题是否有数据（推荐方法）
ros2 topic echo /map --once

# 查看地图信息
ros2 topic info /map

# 查看地图 QoS 设置
ros2 topic info /map -v
```

**重要说明**：
- `nav2_map_server` 使用 `TRANSIENT_LOCAL` QoS，地图只在激活时发布一次
- 新订阅者（如 RViz2）会立即收到最后发布的地图
- `ros2 topic hz /map` 和 `ros2 topic bw /map` 可能显示为空或频率为 0，这是**正常行为**
- 静态地图不需要持续发布，只在有新订阅者时发送一次

---

## Nav2 导航配置

### 前置条件

1. **安装 Nav2 相关包**：
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2 \
                   ros-humble-nav2-bringup \
                   ros-humble-nav2-map-server \
                   ros-humble-nav2-amcl \
                   ros-humble-nav2-controller \
                   ros-humble-nav2-planner \
                   ros-humble-nav2-recoveries \
                   ros-humble-nav2-bt-navigator \
                   ros-humble-nav2-waypoint-follower \
                   ros-humble-nav2-velocity-smoother \
                   ros-humble-nav2-lifecycle-manager
   ```

2. **确保机器人驱动正在运行**：
   ```bash
   # 启动机器人核心驱动
   ros2 launch tianbot_core_ros2 tianbot_core.launch.py
   ```

3. **确保有激光雷达数据**（用于定位）：
   ```bash
   # 检查 /scan 话题
   ros2 topic echo /scan --once
   ```

4. **确保有里程计数据**（用于定位）：
   ```bash
   # 检查 /odom 话题
   ros2 topic echo /odom --once
   ```

### 启动 Nav2 导航

#### 方法 1: 使用 nav2_navigation.launch.py（推荐）

```bash
# 使用默认地图路径
ros2 launch tianracer_slam_ros2 nav2_navigation.launch.py

# 或指定自定义地图路径
ros2 launch tianracer_slam_ros2 nav2_navigation.launch.py map_file:=/path/to/your/map.yaml
```

#### 方法 2: 使用 Nav2 官方 bringup

```bash
# 使用 Nav2 官方启动文件（需要配置参数文件）
ros2 launch nav2_bringup navigation_launch.py \
    map:=/home/lucifer/tianracer_ros2_ws/my_map.yaml \
    params_file:=/path/to/nav2_params.yaml
```

### 在 RViz2 中使用 Nav2

1. **启动导航后，启动 RViz2**：
   ```bash
   rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
   ```

2. **或者手动配置 RViz2**：
   - 添加 "Map" 显示（话题：`/map`）
   - 添加 "RobotModel" 显示
   - 添加 "LaserScan" 显示（话题：`/scan`）
   - 添加 "TF" 显示
   - 添加 "Path" 显示（话题：`/plan` 和 `/follow_path`）
   - 添加 "PoseArray" 显示（话题：`/particlecloud`，用于查看 AMCL 粒子）

3. **设置初始位置**（重要！）：
   - 点击 RViz2 工具栏中的 "2D Pose Estimate" 按钮
   - 在地图上点击并拖拽，设置机器人的初始位置和朝向
   - 这会让 AMCL 知道机器人在地图上的初始位置

4. **发送导航目标**：
   - 点击 RViz2 工具栏中的 "2D Nav Goal" 按钮
   - 在地图上点击目标位置并拖拽设置朝向
   - 机器人将自动规划路径并导航到目标位置

### 使用命令行发送导航目标

```bash
# 使用 nav2_simple_commander（需要安装）
ros2 run nav2_simple_commander simple_commander

# 或者使用服务调用
ros2 service call /navigate_to_pose nav2_msgs/srv/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 导航节点运行位置

### 可以在电脑上运行吗？

**答案：可以！** Nav2 导航节点可以完全运行在你的电脑上，不需要在机器人上运行。

### 架构说明

Nav2 采用分布式架构，支持以下运行方式：

1. **所有节点在电脑上运行**（推荐用于开发和测试）：
   - 电脑：运行所有 Nav2 节点（map_server, amcl, planner, controller 等）
   - 机器人：只运行传感器驱动和底层控制节点
   - 通信：通过 ROS2 网络（ROS_DOMAIN_ID）连接

2. **节点分布在电脑和机器人上**：
   - 电脑：运行规划节点（planner）、可视化（RViz2）
   - 机器人：运行控制节点（controller）、定位节点（amcl）
   - 通信：通过 ROS2 网络连接

3. **所有节点在机器人上运行**：
   - 机器人：运行所有节点
   - 电脑：只运行可视化（RViz2）
   - 通信：通过 ROS2 网络连接

### 网络配置

如果机器人和电脑不在同一台机器上，需要配置 ROS2 网络：

1. **设置 ROS_DOMAIN_ID**（在同一网络中）：
   ```bash
   # 在电脑上
   export ROS_DOMAIN_ID=0
   
   # 在机器人上（如果使用）
   export ROS_DOMAIN_ID=0
   ```

2. **配置网络**（跨网络）：
   - 使用 ROS2 的 DDS 配置（FastRTPS 或 CycloneDDS）
   - 配置防火墙允许 ROS2 通信端口

3. **检查连接**：
   ```bash
   # 查看所有节点
   ros2 node list
   
   # 查看所有话题
   ros2 topic list
   ```

### 性能考虑

- **规划节点**：计算量较大，建议在性能较好的机器上运行
- **控制节点**：需要实时性，建议在机器人上运行（如果可能）
- **定位节点（AMCL）**：计算量中等，可以在电脑上运行
- **地图服务器**：计算量小，可以在任意位置运行

---

## 完整使用流程

### 1. 可视化已保存的地图

```bash
# 终端 1: 加载地图
ros2 launch tianracer_slam_ros2 map_load.launch.py

# 终端 2: 启动 RViz2
rviz2
# 在 RViz2 中添加 Map 显示，话题选择 /map
```

### 2. 使用 Nav2 进行导航

```bash
# 终端 1: 启动机器人驱动
ros2 launch tianbot_core_ros2 tianbot_core.launch.py

# 终端 2: 启动激光雷达（如果需要）
ros2 launch osight_lidar_ros2 osight_iexxx.launch.py

# 终端 3: 启动 Nav2 导航
ros2 launch tianracer_slam_ros2 nav2_navigation.launch.py

# 终端 4: 启动 RViz2
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### 3. 在 RViz2 中操作

1. **设置初始位置**：
   - 点击 "2D Pose Estimate"
   - 在地图上点击机器人实际位置

2. **发送导航目标**：
   - 点击 "2D Nav Goal"
   - 在地图上点击目标位置

3. **监控导航过程**：
   - 观察路径规划（绿色/蓝色线条）
   - 观察 AMCL 粒子云（如果显示）
   - 观察机器人移动

---

## 常见问题

### 问题 1: 地图精度问题

**症状**：地图精度与之前使用的地图差异很大

**可能原因**：
1. 地图分辨率设置不正确
2. 地图保存时的阈值设置不当
3. SLAM 建图质量不佳

**解决方案**：

1. **检查地图分辨率**：
   ```bash
   # 查看 my_map.yaml
   cat ~/tianracer_ros2_ws/my_map.yaml
   # 检查 resolution 字段（应该是 0.05 或你建图时使用的分辨率）
   ```

2. **重新建图**（如果地图质量确实不好）：
   ```bash
   # 使用更高质量的 SLAM 参数重新建图
   ros2 launch tianracer_slam_ros2 tianracer_gmapping.launch.py
   ```

3. **调整地图阈值**（如果地图文件已保存）：
   - 编辑 `my_map.yaml` 文件
   - 调整 `occupied_thresh` 和 `free_thresh` 参数
   - 重新加载地图

### 问题 2: 地图无法加载

**错误信息**：`Failed to load map file`

**解决方案**：
```bash
# 检查地图文件是否存在
ls -la ~/tianracer_ros2_ws/my_map.yaml
ls -la ~/tianracer_ros2_ws/my_map.pgm

# 检查文件路径是否正确
# 确保 .yaml 文件中的 image 路径指向正确的 .pgm 文件
cat ~/tianracer_ros2_ws/my_map.yaml
```

### 问题 2.5: map_server 已激活但 /map 话题没有数据流

**症状**：`ros2 lifecycle get /map_server` 显示 `active`，但 `ros2 topic hz /map` 或 `ros2 topic bw /map` 没有输出

**原因**：这是**正常行为**！`nav2_map_server` 使用 `TRANSIENT_LOCAL` QoS：
- 地图只在节点激活时发布一次
- 新订阅者会立即收到最后发布的地图
- 静态地图不需要持续发布数据流

**验证地图是否正常**：
```bash
# 方法 1: 使用 echo 检查（推荐）
ros2 topic echo /map --once
# 如果能看到地图数据（header, info, data），说明地图已正确加载

# 方法 2: 检查节点信息
ros2 node info /map_server
# 应该看到 Publishers: /map: nav_msgs/msg/OccupancyGrid

# 方法 3: 在 RViz2 中添加 Map 显示
# 如果能看到地图，说明一切正常
```

**如果 RViz2 中看不到地图**：

1. **最重要：设置 Fixed Frame 为 `map`**
   - 在 RViz2 左侧面板的 "Global Options" 中
   - 将 "Fixed Frame" 从 `robot_footprint` 或 `odom` 改为 `map`
   - 这是最关键的设置！地图的 frame_id 是 `map`，Fixed Frame 必须匹配

2. **检查 Map 显示配置**：
   - 确保 Topic 设置为 `/map`
   - 确保 QoS 设置匹配：
     - Reliability Policy: `Reliable`
     - Durability Policy: `Transient Local`
     - History Policy: `Keep Last`
     - Depth: `5` 或更大

3. **重新订阅地图**：
   - 在 RViz2 中取消勾选 Map 显示（取消勾选复选框）
   - 等待 1-2 秒
   - 重新勾选 Map 显示
   - 这会让 RViz2 重新订阅并接收地图数据

4. **检查 map_server 状态**：
   ```bash
   ros2 lifecycle get /map_server
   # 应该显示: active [3]
   
   ros2 topic echo /map --once
   # 应该能看到地图数据
   ```

5. **如果仍然看不到**：
   - 关闭 RViz2
   - 重新启动 map_server（如果已停止）
   - 重新启动 RViz2
   - 确保在启动 RViz2 之前 map_server 已经激活

### 问题 3: AMCL 定位失败

**症状**：机器人无法定位，粒子云不收敛

**解决方案**：

1. **正确设置初始位置**：
   - 在 RViz2 中使用 "2D Pose Estimate" 设置准确的初始位置

2. **检查 TF 树**：
   ```bash
   ros2 run tf2_tools view_frames
   # 确保 map -> odom -> base_footprint 链路完整
   ```

3. **检查传感器数据**：
   ```bash
   # 检查激光数据
   ros2 topic echo /scan --once
   
   # 检查里程计数据
   ros2 topic echo /odom --once
   ```

4. **调整 AMCL 参数**：
   - 增加粒子数量（`min_particles`, `max_particles`）
   - 调整初始协方差（`initial_cov_xx`, `initial_cov_yy`, `initial_cov_aa`）

### 问题 4: 导航路径规划失败

**症状**：无法规划路径到目标点

**解决方案**：

1. **检查目标点是否在自由空间**：
   - 确保目标点不在障碍物上
   - 确保目标点在地图范围内

2. **检查代价地图**：
   ```bash
   # 查看全局代价地图
   ros2 topic echo /global_costmap/costmap --once
   
   # 查看局部代价地图
   ros2 topic echo /local_costmap/costmap --once
   ```

3. **调整规划器参数**：
   - 在 launch 文件中调整 `tolerance` 参数
   - 设置 `allow_unknown: true` 允许在未知区域规划

### 问题 5: 机器人不移动

**症状**：路径规划成功，但机器人不移动

**解决方案**：

1. **检查控制命令发布**：
   ```bash
   # 查看控制命令
   ros2 topic echo /cmd_vel
   ```

2. **检查机器人驱动**：
   ```bash
   # 确保机器人驱动正在运行
   ros2 node list | grep tianbot_core
   ```

3. **检查速度限制**：
   - 在 launch 文件中检查 `desired_linear_vel` 和 `desired_angular_vel` 参数
   - 确保速度值合理（不为 0）

### 问题 6: 导航节点无法启动

**错误信息**：找不到 nav2 相关包

**解决方案**：
```bash
# 安装 Nav2 完整包
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# 检查包是否安装
ros2 pkg list | grep nav2
```

### 问题 7: RViz2 GLSL 着色器错误

**错误信息**：
```
[ERROR] [rviz2]: Vertex Program:rviz/glsl120/indexed_8bit_image.vert Fragment Program:rviz/glsl120/indexed_8bit_image.frag GLSL link result : 
active samplers with a different type refer to the same texture image unit
```

**原因**：这是 RViz2 的已知问题，通常是由于 OpenGL 驱动或着色器版本兼容性问题导致的。

**解决方案**：
1. **通常可以忽略**：这个错误通常不影响地图显示，只是警告
2. **更新显卡驱动**（如果问题严重）：
   ```bash
   # 检查显卡驱动
   glxinfo | grep "OpenGL version"
   
   # 如果是 NVIDIA 显卡
   nvidia-smi
   ```
3. **使用软件渲染**（如果硬件加速有问题）：
   ```bash
   LIBGL_ALWAYS_SOFTWARE=1 rviz2
   ```
4. **更新 RViz2**：
   ```bash
   sudo apt update
   sudo apt upgrade ros-humble-rviz2
   ```

### 问题 8: 地图显示位置偏离

**症状**：地图在 RViz2 中显示的位置与预期不符，或者地图显示在奇怪的位置

**可能原因**：
1. 地图原点（origin）设置不正确
2. Fixed Frame 设置错误
3. 地图坐标系与机器人坐标系不匹配

**解决方案**：

1. **检查地图原点**：
   ```bash
   cat ~/tianracer_ros2_ws/my_map.yaml
   # 查看 origin 字段：[x, y, yaw]
   ```

2. **调整 RViz2 视角**：
   - 在 RViz2 中，使用鼠标中键拖拽移动视角
   - 使用滚轮缩放
   - 使用鼠标右键旋转视角
   - 点击 "Reset" 按钮重置视角

3. **检查地图原点设置**：
   - 地图原点 `[0.32, 14.5, 0]` 表示地图左下角在世界坐标系中的位置
   - 如果地图显示位置不对，可能需要调整原点
   - 通常原点应该设置为 `[0, 0, 0]` 或地图中心位置

4. **重新设置地图原点**（如果需要）：
   ```yaml
   # 编辑 my_map.yaml
   origin: [0, 0, 0]  # 将原点设置为 [0, 0, 0]
   ```
   然后重新加载地图

5. **使用 RViz2 的 "2D Pose Estimate" 工具**：
   - 如果地图位置正确但机器人位置不对，使用 "2D Pose Estimate" 设置机器人初始位置

6. **检查地图尺寸和分辨率**：
   ```bash
   ros2 topic echo /map --once | grep -E "width|height|resolution"
   # 地图实际尺寸 = width * resolution (米) x height * resolution (米)
   ```

---

## 地图文件说明

### my_map.yaml 文件结构

```yaml
image: my_map.pgm          # PGM 图像文件路径（相对于 .yaml 文件）
mode: trinary              # 地图模式：trinary, scale, raw
resolution: 0.05           # 地图分辨率（米/像素）
origin: [0.32, 14.5, 0]    # 地图原点 [x, y, yaw]（米，弧度）
negate: 0                  # 是否反转颜色（0=否，1=是）
occupied_thresh: 0.65      # 障碍物阈值（0-1）
free_thresh: 0.19          # 自由空间阈值（0-1）
```

### 地图精度说明

- **resolution: 0.05** 表示每个像素代表 0.05 米（5 厘米）
- 这是比较高的精度，适合室内导航
- 如果地图精度不够，可能需要：
  1. 使用更高的分辨率重新建图（例如 0.02 或 0.03）
  2. 检查建图时的传感器数据质量
  3. 使用更高质量的 SLAM 算法

---

## 快速参考

### 常用命令

```bash
# 加载地图
ros2 launch tianracer_slam_ros2 map_load.launch.py

# 启动导航
ros2 launch tianracer_slam_ros2 nav2_navigation.launch.py

# 查看地图话题
ros2 topic echo /map --once

# 查看所有 Nav2 节点
ros2 node list | grep nav2

# 查看 Nav2 服务
ros2 service list | grep nav2

# 检查 TF 树
ros2 run tf2_tools view_frames
```

### 地图文件位置

- 默认地图路径：`~/tianracer_ros2_ws/my_map.yaml`
- 地图图像文件：`~/tianracer_ros2_ws/my_map.pgm`

---

---

## 相关文档

- [RViz2 问题排查指南](RVIZ2_TROUBLESHOOTING.md) - 详细的 RViz2 显示问题解决方案

---

**最后更新**：2026-01-18

