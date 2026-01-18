# Nav2 导航快速开始指南

## 前置条件

1. **已保存的地图**（`my_map.yaml` 和 `my_map.pgm`）
2. **机器人驱动正在运行**
3. **激光雷达数据可用**（`/scan` 话题）
4. **里程计数据可用**（`/odom` 话题）

## 快速启动步骤

### 步骤 1: 启动机器人驱动

```bash
ros2 launch tianbot_core_ros2 tianbot_core.launch.py
```

### 步骤 2: 启动激光雷达（如果需要）

```bash
ros2 launch osight_lidar_ros2 osight_iexxx.launch.py
```

### 步骤 3: 启动导航

```bash
ros2 launch tianracer_navigation_ros2 navigation.launch.py
```

### 步骤 4: 启动 RViz2

```bash
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

或者手动启动 RViz2 并配置显示。

## 在 RViz2 中操作

### 1. 设置初始位置（必须！）

1. 点击 RViz2 工具栏中的 **"2D Pose Estimate"** 按钮
2. 在地图上点击机器人**实际位置**
3. 拖拽设置机器人**朝向**
4. 这会让 AMCL 知道机器人在地图上的初始位置

### 2. 发送导航目标

1. 点击 RViz2 工具栏中的 **"2D Nav Goal"** 按钮
2. 在地图上点击**目标位置**
3. 拖拽设置**目标朝向**
4. 机器人将自动规划路径并导航

## 验证导航是否工作

### 检查节点

```bash
ros2 node list | grep nav2
```

应该看到：
- `/map_server`
- `/amcl`
- `/controller_server`
- `/planner_server`
- `/bt_navigator`
- 等

### 检查话题

```bash
ros2 topic list | grep nav
```

应该看到：
- `/plan` - 路径规划
- `/follow_path` - 路径跟踪
- `/particlecloud` - AMCL 粒子
- `/amcl_pose` - 定位结果

### 检查服务

```bash
ros2 service list | grep nav
```

应该看到：
- `/navigate_to_pose` - 导航服务
- `/compute_path_to_pose` - 路径计算服务

## 常见问题

### Q: 看不到地图？

A: 确保 Fixed Frame 设置为 `map`，并添加 Map 显示。

### Q: AMCL 定位失败？

A: 确保在 RViz2 中设置了初始位置（2D Pose Estimate）。

### Q: 路径规划失败？

A: 确保目标点不在障碍物上，且在地图范围内。

### Q: 机器人不移动？

A: 检查 `/cmd_vel` 话题是否有输出，检查机器人驱动是否运行。

## 下一步

- 调整导航参数以获得更好的性能
- 配置代价地图参数
- 使用航点导航功能
- 集成到完整的机器人系统中

