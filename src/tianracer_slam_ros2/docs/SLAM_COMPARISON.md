# SLAM 算法对比：GMapping vs Cartographer vs RTAB-Map

## 核心差异

| 特性 | GMapping (slam_toolbox) | Cartographer | RTAB-Map |
|------|------------------------|--------------|----------|
| 算法类型 | 粒子滤波 | 图优化 | 图优化 + 视觉词袋 |
| 回环检测 | 基本支持 | 强 | 非常强 |
| 计算资源 | 低 | 中 | 高 |
| 适用场景 | 小型室内 | 大型室内/室外 | 复杂环境 |
| 传感器 | 仅激光雷达 | 激光雷达+IMU | 激光雷达+相机+IMU |

## 算法原理

### GMapping (slam_toolbox)
- 使用 Rao-Blackwellized 粒子滤波
- 每个粒子维护一个独立的地图
- 优点：计算简单，实时性好
- 缺点：长走廊容易漂移，大环境粒子退化

### Cartographer
- 基于子图(submap)的图优化
- 局部 SLAM：扫描匹配构建子图
- 全局 SLAM：子图间约束优化
- 优点：大环境精度高，回环检测强
- 缺点：配置复杂，需要调参

### RTAB-Map
- 基于外观的回环检测（视觉词袋）
- 增量式图优化
- 支持多传感器融合
- 优点：回环检测最强，支持 3D
- 缺点：资源消耗大，纯激光效果一般

## 建图质量对比

### 小房间 (<50m²)
推荐：GMapping
- 简单快速，足够使用

### 中型环境 (50-500m²)
推荐：Cartographer
- 回环检测好，地图一致性高

### 大型/复杂环境 (>500m²)
推荐：RTAB-Map（配合相机）或 Cartographer
- 需要强回环检测防止漂移

## 使用方法

### 启动建图

```bash
# GMapping (slam_toolbox)
ros2 launch tianracer_slam_ros2 tianracer_gmapping.launch.py

# Cartographer
ros2 launch tianracer_slam_ros2 tianracer_cartographer.launch.py

# RTAB-Map
ros2 launch tianracer_slam_ros2 tianracer_rtabmap.launch.py
```

### 保存地图

```bash
# 通用方法（适用于所有 SLAM）
ros2 launch tianracer_slam_ros2 map_save.launch.py slam_methods:=gmapping

# Cartographer 专用（保存 pbstream + pgm）
ros2 launch tianracer_slam_ros2 map_save_cartographer.launch.py map_name:=my_map

# RTAB-Map 专用
ros2 launch tianracer_slam_ros2 map_save_rtabmap.launch.py map_name:=my_map
```

地图默认保存到 `~/tianracer_ros2_ws/maps/` 目录。

## 建图质量提升建议

1. 控制移动速度：慢速移动，避免快速旋转
2. 多次经过同一区域：帮助回环检测
3. 避免特征稀疏区域：长走廊、空旷区域容易漂移
4. 使用 IMU：Cartographer 配合 IMU 效果更好

## 你的情况

如果 GMapping 建图质量不好，可能原因：
1. 环境特征不足（长走廊、对称结构）
2. 里程计漂移
3. 移动速度过快

建议先尝试 Cartographer，它的回环检测更强，对漂移的修正能力更好。

## Topic 和 TF 需求

### 各算法需要的 Topic

| Topic | GMapping | Cartographer | RTAB-Map |
|-------|----------|--------------|----------|
| `/scan` | 必需 | 必需 | 必需 |
| `/odom` 或 `/odometry/filtered` | 必需 | 可选(推荐) | 必需 |
| `/imu` | 不需要 | 可选(推荐) | 可选 |
| 深度相机 | 不需要 | 不需要 | 可选(推荐) |

### 各算法需要的 TF

所有算法都需要完整的 TF 链：

```
map -> odom -> base_link -> laser
                        -> base_footprint
                        -> camera_link (RTAB-Map 可选)
```

| TF 变换 | 发布者 | 说明 |
|---------|--------|------|
| map -> odom | SLAM 算法 | 自动发布 |
| odom -> base_link | 里程计/底盘 | 机器人底盘发布 |
| base_link -> laser | 静态 TF | 激光雷达位置 |
| base_link -> base_footprint | 静态 TF | 通常重合 |
| base_link -> camera_link | 静态 TF | 相机位置(可选) |

### 检测 TF 是否存在

```bash
# 查看完整 TF 树
ros2 run tf2_tools view_frames

# 检测特定 TF 是否存在
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo odom base_link

# 查看所有 TF
ros2 topic echo /tf_static
ros2 topic echo /tf
```

### 补充缺失的 TF

如果某些静态 TF 缺失，启动补充脚本：

```bash
ros2 launch tianracer_slam_ros2 static_tf.launch.py
```

或手动发布单个 TF：

```bash
# base_link -> laser (x=0.15m 前, z=0.1m 高)
ros2 run tf2_ros static_transform_publisher 0.15 0 0.1 0 0 0 base_link laser

# base_link -> base_footprint (重合)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_footprint

# base_link -> camera_link (在 laser 前 3cm)
ros2 run tf2_ros static_transform_publisher 0.18 0 0.1 0 0 0 base_link camera_link
```

### RViz2 可视化配置

所有 SLAM 算法都发布 `/map` topic，可视化方法相同：

1. Fixed Frame: `map`
2. 添加显示:
   - Map: `/map`
   - LaserScan: `/scan`
   - TF: 查看坐标系
   - RobotModel: 机器人模型(可选)
