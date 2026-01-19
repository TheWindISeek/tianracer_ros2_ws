# URDF 基础概念说明

本文档简要说明 URDF 文件中的关键概念和参数含义。

## 目录

1. [Link 和 Frame 的关系](#link-和-frame-的关系)
2. [Joint 参数说明](#joint-参数说明)
3. [Origin 和 Axis 的区别](#origin-和-axis-的区别)
4. [Joint Limit 参数](#joint-limit-参数)
5. [Gazebo 插件说明](#gazebo-插件说明)
6. [机器人架构对比](#机器人架构对比)

---

## Link 和 Frame 的关系

### 核心概念

- **一个 Link = 一个 Frame（坐标系）**
- Joint 不创建 frame，只定义 frame 之间的变换关系
- 在代码中引用 frame 时，使用 link 名称

### 示例

```xml
<link name="base_link"/>
<joint name="base_joint" type="fixed">
  <parent link="base_footprint"/>
  <child link="base_link"/>
</joint>
```

- `base_link` link → `base_link` frame
- `base_joint` 定义从 `base_footprint` frame 到 `base_link` frame 的变换

---

## Joint 参数说明

### Parent 和 Child

- **Parent**: 父节点，坐标系参考点
- **Child**: 子节点，相对于父节点的位置
- 决定坐标系继承关系和运动关系

### Origin（位置偏移）

```xml
<origin xyz="0 0 0.065" rpy="0 0 0"/>
```

- **xyz**: 位置偏移（米）
  - x: 前后（前为正）
  - y: 左右（左为正）
  - z: 上下（上为正）
- **rpy**: 旋转角度（弧度）
  - r (roll): 绕 X 轴旋转
  - p (pitch): 绕 Y 轴旋转
  - y (yaw): 绕 Z 轴旋转

### Axis（旋转轴方向）

```xml
<axis xyz="0 1 0"/>
```

- 定义关节旋转的轴线方向（单位向量）
- `(0, 1, 0)` = Y 轴方向（轮子滚动）
- `(0, 0, 1)` = Z 轴方向（转向）

---

## Origin 和 Axis 的区别

### Origin = 位置偏移

- 定义 child link 在 parent link 坐标系中的**位置**
- 不是朝向，是空间位置

### Axis = 旋转轴方向

- 定义关节绕哪个轴**旋转**
- 不是位置，是旋转方向

### 为什么前轮轮子 origin 是 (0,0,0)？

前轮轮子关节的 parent 是转向连杆，位置已经在转向关节定义，所以轮子关节的 origin 为 (0,0,0)。

后轮直接连接底盘，需要在关节中定义位置：`origin xyz="${rear_wheel_x} ${wheel_y} 0"`

---

## Joint Limit 参数

```xml
<limit lower="-0.5" upper="0.5" effort="100" velocity="2.0"/>
```

### 参数说明

| 参数 | 单位 | 说明 |
|------|------|------|
| `lower` | 弧度 | 最小角度（负值=负方向） |
| `upper` | 弧度 | 最大角度（正值=正方向） |
| `effort` | N·m | 最大力矩（负值=无限制） |
| `velocity` | rad/s | 最大角速度（负值=无限制） |

### 示例

- `lower="-0.5" upper="0.5"`: 转向角度范围 ±28.6°
- `effort="100"`: 最大力矩 100 N·m
- `velocity="2.0"`: 最大角速度 2.0 rad/s

---

## Gazebo 插件说明

### 1. gazebo_ros_joint_state_publisher

**作用**: 发布关节状态到 `/joint_states` 话题

```xml
<plugin name="gazebo_ros_joint_state_publisher">
  <update_rate>100</update_rate>
  <joint_name>left_front_steer_joint</joint_name>
  <!-- 只列出可动关节 -->
</plugin>
```

- **为什么只发布部分关节**: 只发布可动关节，固定关节状态不变
- **update_rate**: 发布频率（Hz），100Hz 保证实时性
- **必需性**: 必需，用于生成 TF 变换

### 2. gazebo_ros_joint_pose_trajectory

**作用**: 接收 `/set_joint_trajectory` 话题，控制关节运动

```xml
<plugin name="gazebo_ros_joint_pose_trajectory">
  <update_rate>2</update_rate>
</plugin>
```

- **为什么只有一个参数**: 处理所有关节的轨迹命令，不需要指定关节
- **update_rate**: 处理频率（Hz），2Hz 足够
- **必需性**: 可选，主要用于调试

---

## 机器人架构对比

### tianbot_ackermann.urdf.xacro

- **结构**: 2层（base_footprint → base_link）
- **特点**: 单一文件，base_link 直接作为底盘
- **命名**: left_front, right_front, left_rear, right_rear

### tianracer.xacro

- **结构**: 3层（base_footprint → base_link → chassis_link）
- **特点**: 模块化设计，base_link 为空链接
- **命名**: fr_left, fr_right, re_left, re_right
- **传感器**: 包含摄像头插件，激光雷达使用 GPU 加速

### 主要差异

| 特性 | tianbot_ackermann | tianracer |
|------|-------------------|-----------|
| 底盘高度 | 0.40m | 0.20m |
| 轮子厚度 | 0.04m | 0.14m |
| 摄像头 | 无插件 | 有插件 |
| 激光雷达 | CPU (ray) | GPU (gpu_ray) |
| 模块化 | 单一文件 | 模块化设计 |

---

## 常见问题

### Q: 为什么 z = wheel_radius？

A: base_footprint 在地面（z=0），base_link 的底部应该在轮子中心高度，所以 z 偏移设为轮子半径。

### Q: 实际机器人需要这些插件吗？

A: 不需要。这些插件仅用于 Gazebo 仿真。真实机器人由硬件驱动或 ros2_control 提供相应功能。

### Q: 为什么前轮轮子 origin 是 (0,0,0)？

A: 前轮位置已在转向关节定义，轮子关节的 parent 是转向连杆，所以无偏移。

### Q: 为什么缺少 `<limit>` 标签会导致轮子重叠到原点？

#### 问题现象

当 wheel_joint（continuous 类型）缺少 `<limit>` 标签时：
1. 模型加载后，四个轮子会"折叠"成一个
2. 整个机器人瞬移到世界原点 (0,0,0)

#### 解决方案

为所有 wheel_joint 添加 `<limit>` 标签：

```xml
<joint name="wheel_joint" type="continuous">
  ...
  <limit lower="-3.14159" upper="3.14159" effort="200" velocity="-1"/>
</joint>
```

- `effort="200"`: 允许最大 200 N·m 的力矩（足够驱动轮子）
- `velocity="-1"`: **-1 是特殊值，表示无速度限制**
- `lower/upper`: 对 continuous 类型意义不大，但建议设为 ±π

#### 为什么 tianracer 能正常工作

tianracer 的所有 wheel_joint 都有完整的 `<limit>` 定义：

```xml
<limit lower="-3.14159" upper="3.14159" effort="200" velocity="-1"/>
```

具体原因尚不明确，可能与 Ackermann 插件和 Gazebo 物理引擎的交互有关。

### Q: 为什么在自定义 Gazebo 世界中机器人会解体？

#### 问题现象

使用自定义世界文件时：
1. 机器人持续向 y 轴正方向翻滚
2. 部件脱落（左前轮、激光雷达等）
3. 使用默认世界文件时正常

#### 三个根本原因

**1. 地面平面位置偏移**

```xml
<!-- 错误：地面不在原点 -->
<model name='ground_plane'>
  <pose>3.3701 -5.03256 0 0 -0 0</pose>
</model>

<!-- 正确：地面在原点 -->
<model name='ground_plane'>
  <pose>0 0 0 0 -0 0</pose>
</model>
```

**2. 物理引擎刚度过高**

```xml
<!-- 错误：刚度过高导致数值不稳定 -->
<kp>1e+13</kp>

<!-- 正确：使用标准刚度 -->
<kp>1e+06</kp>
```

**3. 生成位置与世界物体碰撞（主要原因）**

机器人生成位置 (0, 5, 0) 与世界中的物体重叠：

```xml
<!-- 检查 launch 文件中的生成位置 -->
y_arg = DeclareLaunchArgument('y', default_value='5.0')

<!-- 检查世界文件中该位置的物体 -->
<model name='labseat_clone_0'>
  <pose>0.25 5.02745 0 0 -0 0</pose>  <!-- 与生成位置冲突！ -->
</model>
```

#### 诊断方法

1. 检查 launch 文件中的机器人生成位置 (x, y, z)
2. 在世界文件中搜索相近的 `<pose>` 坐标
3. 移动冲突物体或修改生成位置

#### 解决方案

```xml
<!-- 移动冲突物体到安全位置 -->
<model name='labseat_clone_0'>
  <pose>0.291949 7.5 0 0 -0 0</pose>  <!-- 从 y=5.02745 移到 y=7.5 -->
</model>
```

**注意**: SDF 文件中 `<state>` 部分和模型定义部分的 `<pose>` 需要同步修改。

---

## 参考

- [URDF 官方文档](http://wiki.ros.org/urdf)
- [Gazebo 插件文档](http://gazebosim.org/tutorials?cat=connect_ros)

