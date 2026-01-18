# Tianracer Navigation ROS2 Package

这是 Tianracer 导航包的 ROS2 版本，基于 Nav2 实现自主导航功能。

## 功能

- **地图加载**：加载已保存的地图
- **定位（AMCL）**：使用自适应蒙特卡洛定位算法
- **路径规划**：全局路径规划
- **路径跟踪**：局部路径跟踪控制
- **恢复行为**：处理导航失败情况（旋转、后退、等待）
- **行为树导航**：使用行为树管理导航流程
- **速度平滑**：平滑控制命令输出

## 依赖

### 系统依赖

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

## 编译

```bash
cd ~/tianracer_ros2_ws
colcon build --packages-select tianracer_navigation_ros2
source install/setup.bash
```

## 使用方法

### 1. 启动导航（完整流程）

```bash
# 终端 1: 启动机器人驱动
ros2 launch tianbot_core_ros2 tianbot_core.launch.py

# 终端 2: 启动激光雷达（如果需要）
ros2 launch osight_lidar_ros2 osight_iexxx.launch.py

# 终端 3: 启动导航
ros2 launch tianracer_navigation_ros2 navigation.launch.py

# 终端 4: 启动 RViz2
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### 2. 使用自定义地图

```bash
ros2 launch tianracer_navigation_ros2 navigation.launch.py \
    map_file:=/path/to/your/map.yaml
```

### 3. 在 RViz2 中使用导航

1. **设置初始位置**（重要！）：
   - 点击 RViz2 工具栏中的 "2D Pose Estimate" 按钮
   - 在地图上点击并拖拽，设置机器人的初始位置和朝向
   - 这会让 AMCL 知道机器人在地图上的初始位置

2. **发送导航目标**：
   - 点击 RViz2 工具栏中的 "2D Nav Goal" 按钮
   - 在地图上点击目标位置并拖拽设置朝向
   - 机器人将自动规划路径并导航到目标位置

## 启动的节点

导航 launch 文件会启动以下节点：

1. **map_server** - 地图服务器，加载并发布地图
2. **amcl** - 自适应蒙特卡洛定位
3. **controller_server** - 路径跟踪控制器
4. **planner_server** - 全局路径规划器
5. **recoveries_server** - 恢复行为服务器
6. **bt_navigator** - 行为树导航器（管理导航流程）
7. **waypoint_follower** - 航点跟随器
8. **velocity_smoother** - 速度平滑器
9. **lifecycle_manager_navigation** - 生命周期管理器

## 话题

### 订阅的话题

- `/map` - 地图数据
- `/scan` - 激光雷达扫描数据
- `/odom` - 里程计数据
- `/cmd_vel` - 速度控制命令（输入）

### 发布的话题

- `/cmd_vel` - 平滑后的速度控制命令（输出）
- `/plan` - 全局路径规划结果
- `/follow_path` - 局部路径跟踪
- `/particlecloud` - AMCL 粒子云
- `/amcl_pose` - AMCL 估计的位姿

## 服务

- `/navigate_to_pose` - 导航到目标位姿
- `/compute_path_to_pose` - 计算路径
- `/clear_costmap_*` - 清除代价地图
- `/reinitialize_global_localization` - 重新初始化全局定位

## 参数调整

### AMCL 参数

在 `launch/navigation.launch.py` 中可以调整：
- `min_particles` / `max_particles` - 粒子数量
- `initial_pose_x/y/a` - 初始位姿
- `laser_max_beams` - 激光束数量

### 控制器参数

- `desired_linear_vel` - 期望线速度
- `desired_angular_vel` - 期望角速度
- `xy_goal_tolerance` - 位置容差
- `yaw_goal_tolerance` - 角度容差

### 规划器参数

- `tolerance` - 路径容差
- `allow_unknown` - 是否允许在未知区域规划

## 故障排除

### 问题 1: 找不到 Nav2 包

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### 问题 2: AMCL 定位失败

1. 确保在 RViz2 中正确设置初始位置
2. 检查 `/scan` 和 `/odom` 话题是否有数据
3. 检查 TF 树是否完整

### 问题 3: 路径规划失败

1. 确保目标点不在障碍物上
2. 检查地图是否正确加载
3. 调整 `allow_unknown` 参数

### 问题 4: 机器人不移动

1. 检查 `/cmd_vel` 话题是否有输出
2. 检查机器人驱动是否运行
3. 检查速度限制参数

## 相关文档

- [地图使用和导航指南](../tianracer_slam_ros2/docs/MAP_NAVIGATION_GUIDE.md)
- [RViz2 问题排查](../tianracer_slam_ros2/docs/RVIZ2_TROUBLESHOOTING.md)

## 许可证

TODO

