# Nav2 真机调试记录

本文档记录了 Tianracer 真机环境下 Nav2 导航系统的调试过程，包括遇到的问题、原因分析和解决方案。

## 目录

1. [路径规划失败：GridBased failed to create plan](#1-路径规划失败gridbased-failed-to-create-plan)
2. [AMCL 初始位置不生效](#2-amcl-初始位置不生效)
3. [DWB 控制器报错：No valid trajectories](#3-dwb-控制器报错no-valid-trajectories)
4. [里程计漂移：机器人直行但 RViz 显示偏移](#4-里程计漂移机器人直行但-rviz-显示偏移)
5. [狭窄通道无法通过](#5-狭窄通道无法通过)
6. [动态参数调整方法](#6-动态参数调整方法)
7. [常用运维命令](#7-常用运维命令)
8. [核心概念解释](#8-核心概念解释)

---

## 1. 路径规划失败：GridBased failed to create plan

### 错误信息

```
[planner_server]: GridBased: failed to create plan with tolerance 0.50
```

### 原因分析

`inflation_radius` 设置过大（如 0.22-0.30m），导致障碍物膨胀区域过大，狭窄通道被完全"堵死"。

### 解决方案

减小 `inflation_radius`，让规划器能找到路径：

```yaml
global_costmap:
  inflation_layer:
    inflation_radius: 0.05      # 极小膨胀
    cost_scaling_factor: 80.0   # 超快衰减，5cm 外代价接近 0
```

---

## 2. AMCL 初始位置不生效

### 问题现象

配置了 `initial_pose_x: 1.0` 等参数，但 AMCL 启动后机器人始终在 (0, 0, 0)。

### 原因分析

Nav2 的 AMCL 参数命名格式与 ROS1 不同，需要使用嵌套格式。

### 解决方案

```yaml
# 错误写法
initial_pose_x: 1.0
initial_pose_y: 1.7
initial_pose_a: 3.138

# 正确写法
initial_pose:
  x: 1.0
  y: 1.7
  yaw: 3.138
  z: 0.0
```

同时确保 `set_initial_pose: True`。

---

## 3. DWB 控制器报错：No valid trajectories

### 错误信息

```
[controller_server]: Critic ObstacleFootprint failed to score trajectory
[controller_server]: No valid trajectories out of X
```

### 原因分析

1. **Local costmap 检测到障碍物**：可能是真实障碍物，也可能是激光雷达噪声
2. **Inflation 过大**：膨胀区域阻挡了所有候选轨迹
3. **机器人位置不准**：AMCL 定位偏差导致 costmap 中机器人"撞墙"

### 解决方案

1. 减小 local_costmap 的 inflation_radius
2. 调整 obstacle_layer 的高度过滤参数：
   ```yaml
   voxel_layer:
     min_obstacle_height: 0.15  # 过滤地面噪声
     max_obstacle_height: 1.5
   ```
3. 增大 `obstacle_min_range` 过滤近距离噪声：
   ```yaml
   scan:
     obstacle_min_range: 0.35  # 忽略 35cm 内的检测
   ```

---

## 4. 里程计漂移：机器人直行但 RViz 显示偏移

### 问题现象

真实机器人直行，但 RViz 中机器人模型逐渐偏向一侧，最终"撞进墙里"。

### 原因分析

**物理原因**：前轮转向机构存在机械偏差（舵机安装角度、连杆长度不对称等），导致舵机角度=0 时，前轮实际并非指向正前方。

本车偏差约 **28.67°**（向左偏）。

### 解决方案

通过固件的 `steering_offset` 参数进行补偿：

```bash
# 设置转向偏移（负值补偿左偏）
ros2 service call /debug_cmd_srv tianbot_core_ros2/srv/DebugCmd "{cmd: 'param set steering_offset -28.67'}"

# 保存到固件 Flash（断电不丢失）
ros2 service call /debug_cmd_srv tianbot_core_ros2/srv/DebugCmd "{cmd: 'param save'}"

# 查看当前参数
ros2 service call /debug_cmd_srv tianbot_core_ros2/srv/DebugCmd "{cmd: 'param list'}"
```

### 重要说明

- `steering_offset` 保存在**固件 Flash** 中，不是上位机
- 断电重启后补偿仍然生效
- 设置后，上位机的 `yaw_offset_deg` 和 `steering_offset_degrees` 应设为 0，避免重复补偿

---

## 5. 狭窄通道无法通过

### 问题场景

走道宽度 0.6m，机器人宽度 0.4m，两侧间隙仅 0.1m。

### 原因分析

如果 `inflation_radius > 0.10m`，两侧膨胀区会重叠，整个走道变成"禁区"。

### 解决方案

**策略：极小膨胀 + 超快衰减**

```yaml
inflation_layer:
  inflation_radius: 0.05      # 膨胀半径 5cm
  cost_scaling_factor: 80.0   # 超快衰减
```

代价衰减效果：
| 距离障碍物 | cost 值 |
|-----------|---------|
| 0.00m | 252 (致命) |
| 0.01m | ~54 |
| 0.02m | ~12 |
| 0.03m | ~2.5 |
| 0.05m | ~0 |

**安全性保证**：依赖 `footprint` 碰撞检测，而非膨胀区。

### 分层策略（可选）

```yaml
# Global: 激进，让路径能规划出来
global_costmap:
  inflation_radius: 0.05
  cost_scaling_factor: 80.0

# Local: 稍保守，给动态避障留余量
local_costmap:
  inflation_radius: 0.12
  cost_scaling_factor: 15.0
```

---

## 6. 动态参数调整方法

### 运行时修改参数

```bash
# 修改 inflation_radius
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.10

# 修改 cost_scaling_factor
ros2 param set /local_costmap/local_costmap inflation_layer.cost_scaling_factor 15.0

# 修改最大速度
ros2 param set /controller_server FollowPath.max_vel_x 0.4
```

### 查看参数

```bash
# 列出所有参数
ros2 param list /local_costmap/local_costmap

# 查看具体值
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
```

### 清除 Costmap（让新参数立即生效）

```bash
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
```

### 可动态修改 vs 需要重启

| 参数类型 | 能否动态修改 |
|---------|-------------|
| inflation_radius, cost_scaling_factor | ✅ 可以 |
| max_vel_x, max_vel_theta | ✅ 可以 |
| footprint | ✅ 可以 |
| plugins 列表 | ❌ 需要重启 |
| resolution | ❌ 需要重启 |

---

## 7. 常用运维命令

### 7.1 清除 Costmap

当 costmap 中有残留的错误障碍物，或修改参数后需要立即生效：

```bash
# 清除 local costmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap

# 清除 global costmap
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap

# 两个都清除
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap && \
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
```

### 7.2 设置初始位置（命令行版 2D Pose Estimate）

当机器人位置不准或需要重新定位时：

```bash
# 设置初始位姿：位置 (x, y) + 朝向四元数 (qz, qw)
# 示例：位置 (1.0, 1.7)，朝向 yaw=180° (朝 -X 方向)
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 1.0, y: 1.7, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
    }
  }
}" --once
```

**四元数快速参考**（yaw 角度 → qz, qw）：

| 朝向 | 角度 | qz | qw |
|------|------|----|----|
| 朝 +X | 0° | 0.0 | 1.0 |
| 朝 +Y | 90° | 0.707 | 0.707 |
| 朝 -X | 180° | 1.0 | 0.0 |
| 朝 -Y | -90° | -0.707 | 0.707 |
| 朝 -Y | 270° | 0.707 | -0.707 |

**通用公式**：`qz = sin(yaw/2)`, `qw = cos(yaw/2)`

### 7.3 矫正 odom 和 map 偏移

当 AMCL 定位漂移，机器人在 RViz 中位置与实际不符时：

```bash
# 方法1：重新设置初始位姿（推荐，立即生效）
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 实际X坐标, y: 实际Y坐标, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 对应qz, w: 对应qw}
    }
  }
}" --once

# 方法2：重新初始化全局定位（粒子重新分布，需要移动才能收敛）
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

**建议**：优先使用方法1，方法2 会让粒子在整个地图重新分布，收敛较慢。

### 7.4 发送导航目标点

```bash
# 发送目标点：位置 (x, y) + 朝向
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 2.0, y: 3.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}" --once
```

### 7.5 取消当前导航

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" --cancel
```

---

## 8. 核心概念解释

### 8.1 Costmap 图层

| 图层 | 作用 | 依赖 |
|------|------|------|
| **static_layer** | 加载静态地图 | map_server 发布的地图 |
| **obstacle_layer** | 标记动态障碍物 | 激光雷达 /scan |
| **voxel_layer** | 3D 体素障碍物检测 | 激光雷达 /scan |
| **inflation_layer** | 在障碍物周围添加缓冲代价 | 前面图层的输出 |

执行顺序：`static_layer → obstacle_layer → inflation_layer`

### 8.2 Cost 值含义

| 值 | 含义 |
|---|------|
| 0 | 自由空间 |
| 1-252 | 有代价，越高越"危险" |
| 253 | 内切圆会碰撞 |
| 254 | 致命障碍物 |

### 8.3 Inflation 衰减公式

```
cost = 252 × e^(-cost_scaling_factor × distance)
```

- `cost_scaling_factor` 越大，衰减越快
- 要让 5cm 处衰减到接近 0，需要 `cost_scaling_factor ≈ 80`

### 8.4 Global vs Local Costmap

| | Global Costmap | Local Costmap |
|---|----------------|---------------|
| 范围 | 整张地图 | 机器人周围 3-5m |
| 坐标系 | map | odom |
| 用途 | 全局路径规划 | 局部避障和轨迹生成 |
| 更新频率 | 低 (1Hz) | 高 (5-10Hz) |

### 8.5 AMCL 定位校准

AMCL 通过粒子滤波进行定位：

1. 初始位置不准 → 粒子分布在错误位置
2. 机器人移动 → 激光扫描环境
3. AMCL 对比激光数据与地图
4. 调整粒子权重，淘汰错误粒子
5. 粒子收敛到正确位置
6. 发布新的 `map→odom` 变换

**注意**：odom 坐标系本身不跳跃，AMCL 通过调整 `map→odom` 变换来"纠正"机器人在 map 中的位置。

---

## 相关文件

- 配置文件：`src/tianracer_navigation_ros2/config/nav2_params_real.yaml`
- 启动文件：`src/tianracer_navigation_ros2/launch/navigation_sim.launch.py`
- Bringup：`src/tianracer_bringup_ros2/launch/tianracer_bringup.launch.py`
- 启动脚本：`boot/nav2.sh`, `boot/cmdvel2ackermann.sh`
