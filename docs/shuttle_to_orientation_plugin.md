# ShuttleToOrientation 行为树插件

## 1. 概述

`ShuttleToOrientation` 是一个专为阿克曼底盘设计的 Nav2 行为树动作节点。由于阿克曼底盘无法原地旋转，当目标点在机器人后方时，传统的导航方式会导致大幅度绕行或无法规划路径。

该插件通过 **Shuttle（穿梭）机动** 来解决这个问题：交替执行"前进+转向"和"后退+反向转向"，逐步改变机器人朝向，使目标点转到前方后再进行正常导航。

## 2. 工作原理

### 2.1 Shuttle 机动原理

```
  俯视图（一个 Shuttle 周期）
  
  ①前进+左转          ②后退+右转
       B                    B
      /|                   /|
     / |                  / |
    /  |   →   左转后   /  | → 朝向继续左转
   /   |      倒车+右转/   |
  A----+               A---C
  
  A: 起始位置          C: 一周期后位置
  结果：位置基本不变，朝向左转约 40-50°
```

### 2.2 状态机流程

```
┌─────────────────────────────────────────────────────────────────┐
│                    ShuttleToOrientation 状态机                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────────┐                                            │
│  │ DETECT_CLEAR_SIDE │ ← 检测左右哪边更空旷，决定转向方向          │
│  └────────┬─────────┘                                            │
│           │                                                       │
│           ▼ 空间足够?                                             │
│     ┌─────┴─────┐                                                │
│     │ 是        │ 否                                              │
│     ▼           ▼                                                │
│  ┌──────┐  ┌────────────────┐                                    │
│  │前进   │  │PRE_FORWARD     │ ← 先直线前进利用前方空间            │
│  │转向   │  │STRAIGHT        │                                    │
│  └──┬───┘  └───────┬────────┘                                    │
│     │              │                                              │
│     │              ▼                                              │
│     │         ┌────────────┐                                      │
│     │         │INITIAL_    │ ← 空间不足时先后退                    │
│     │         │BACKUP      │                                      │
│     │         └─────┬──────┘                                      │
│     │               │                                             │
│     └───────┬───────┘                                             │
│             ▼                                                     │
│      ┌─────────────┐                                              │
│      │FORWARD_TURN │ ← 前进 + 转向                                │
│      └──────┬──────┘                                              │
│             │                                                     │
│             ▼                                                     │
│      ┌─────────────┐                                              │
│      │BACKWARD_TURN│ ← 后退 + 同向转向（累积角度）                 │
│      └──────┬──────┘                                              │
│             │                                                     │
│             ▼                                                     │
│      ┌─────────────────┐                                          │
│      │CHECK_ORIENTATION│ ← 检查目标是否在前方                      │
│      └────────┬────────┘                                          │
│               │                                                   │
│        目标在前方? ─── 是 ──→ SUCCESS (发布最终位置)               │
│               │                                                   │
│               │ 否                                                │
│               │                                                   │
│               └─────────→ 继续下一个 Shuttle 周期                  │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

### 2.3 核心算法

1. **转弯半径计算**：`R = L / tan(δ)` （L=轴距，δ=转向角）
2. **角度变化**：`θ = s / R`（s=行驶距离）
3. **一周期总变化**：`Δψ = 2θ`（前进+后退各转 θ）

使用转向角 28°、行驶距离 0.3m 时，每个周期可转约 35°。

## 3. 配套的条件节点

### 3.1 IsGoalInFront

判断目标点是否在机器人前方的条件节点。

```xml
<IsGoalInFront goal="{goal}" angle_threshold="90.0"/>
```

**参数**：
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| goal | PoseStamped | 必需 | 目标位姿 |
| angle_threshold | double | 90.0 | 角度阈值（度），目标与机器人朝向的夹角在此范围内返回 SUCCESS |

**返回值**：
- `SUCCESS`：目标在前方（角度差 ≤ 阈值）
- `FAILURE`：目标在后方（需要调整朝向）

## 4. ShuttleToOrientation 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| goal | PoseStamped | 必需 | 目标位姿（用于计算角度差） |
| angle_threshold | double | 90.0 | 完成阈值（度），目标在此角度范围内时 Shuttle 完成 |
| clearance_threshold | double | 0.8 | 空旷判断距离阈值（米） |
| forward_distance | double | 0.3 | 每次最大前进距离（米） |
| backward_distance | double | 0.3 | 每次最大后退距离（米） |
| speed | double | 0.1 | 运动速度（米/秒） |
| steering_angle | double | 50.0 | 转向角度（度） |
| timeout | double | 120.0 | 超时时间（秒） |

### 4.1 参数调优建议

- **steering_angle**：转向角越大，每周期转的角度越多，但需要更大空间。建议 25-35°。
- **forward_distance / backward_distance**：通常设为相等，0.2-0.3m 较安全。
- **speed**：建议 0.1-0.2 m/s，太快会影响定位精度。
- **angle_threshold**：建议 5-15°，太大会导致调整不充分。

## 5. 配套行为树 XML

### 5.1 完整示例：ackermann_nav_shuttle.xml

```xml
<!--
  Tianracer 阿克曼底盘导航行为树 - 带 Shuttle 朝向调整

  特性：
  1. 目标在后方时，先 Shuttle 调整朝向再导航
  2. Shuttle 只在导航开始时执行一次，不会因为规划失败而重复
  3. 恢复行为只针对路径规划和跟踪
-->
<root main_tree_to_execute="MainTree">

  <BehaviorTree ID="MainTree">
    <Sequence name="NavigateWithShuttle">

      <!-- ========== 阶段1: 朝向预处理（只执行一次，在 RecoveryNode 外面） ========== -->
      <!-- 如果目标在前方（±90°以内），跳过 Shuttle -->
      <Fallback name="OrientationCheck">
        <!-- 目标已在前方，跳过 -->
        <IsGoalInFront goal="{goal}" angle_threshold="90.0"/>
        <!-- 目标在后方，执行 Shuttle 调整，直到目标在±10°以内 -->
        <Sequence name="ShuttleAndClear">
          <ShuttleToOrientation goal="{goal}"
                                angle_threshold="10.0"
                                clearance_threshold="0.8"
                                forward_distance="0.3"
                                backward_distance="0.3"
                                speed="0.2"
                                steering_angle="28.0"
                                timeout="120.0"/>
          <!-- Shuttle 完成后等待 AMCL 收敛 -->
          <Wait wait_duration="2"/>
          <!-- 清除 costmap，避免因位置偏移导致规划失败 -->
          <ClearEntireCostmap name="ClearLocalAfterShuttle"
                              service_name="local_costmap/clear_entirely_local_costmap"/>
          <ClearEntireCostmap name="ClearGlobalAfterShuttle"
                              service_name="global_costmap/clear_entirely_global_costmap"/>
          <!-- 等待 costmap 重建 -->
          <Wait wait_duration="2"/>
        </Sequence>
      </Fallback>

      <!-- ========== 阶段2: 路径规划和跟踪（带恢复机制） ========== -->
      <RecoveryNode number_of_retries="6" name="NavigateRecovery">

        <PipelineSequence name="PlanAndFollow">
          <RateController hz="1.0">
            <RecoveryNode number_of_retries="1" name="ComputePath">
              <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
              <ClearEntireCostmap name="ClearGlobalCostmap"
                                  service_name="global_costmap/clear_entirely_global_costmap"/>
            </RecoveryNode>
          </RateController>

          <RecoveryNode number_of_retries="1" name="FollowPath">
            <FollowPath path="{path}" controller_id="FollowPath"/>
            <ClearEntireCostmap name="ClearLocalCostmap"
                                service_name="local_costmap/clear_entirely_local_costmap"/>
          </RecoveryNode>
        </PipelineSequence>

        <!-- 恢复行为：只针对路径规划和跟踪失败 -->
        <ReactiveFallback name="RecoveryFallback">
          <GoalUpdated/>
          <RoundRobin name="RecoveryActions">
            <Sequence name="ClearingActions">
              <ClearEntireCostmap name="ClearLocalCostmap-Subtree"
                                  service_name="local_costmap/clear_entirely_local_costmap"/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Subtree"
                                  service_name="global_costmap/clear_entirely_global_costmap"/>
            </Sequence>
            <Wait wait_duration="3"/>
            <BackUp backup_dist="0.15" backup_speed="0.10"/>
          </RoundRobin>
        </ReactiveFallback>

      </RecoveryNode>

    </Sequence>
  </BehaviorTree>

</root>
```

### 5.2 行为树逻辑说明

```
MainTree
└── Sequence: NavigateWithShuttle
    │
    ├── 阶段1: Fallback: OrientationCheck
    │   ├── IsGoalInFront (angle=90°)  → 目标在前方则 SUCCESS，跳过 Shuttle
    │   └── Sequence: ShuttleAndClear
    │       ├── ShuttleToOrientation   → 调整朝向直到目标在 ±10° 内
    │       ├── Wait (2s)              → 等待 AMCL 收敛
    │       ├── ClearLocalCostmap      → 清除 costmap
    │       ├── ClearGlobalCostmap
    │       └── Wait (2s)              → 等待 costmap 重建
    │
    └── 阶段2: RecoveryNode: NavigateRecovery
        ├── PipelineSequence: PlanAndFollow
        │   ├── ComputePathToPose      → 规划路径
        │   └── FollowPath             → 跟踪路径
        └── RecoveryFallback           → 恢复行为
```

## 6. 特性与限制

### 6.1 特性

1. **激光雷达避障**：使用激光雷达实时检测障碍物，动态调整前进/后退距离
2. **自动选择转向方向**：检测左右空间，选择更空旷的一侧进行转向
3. **AMCL 跳变检测**：检测定位跳变并用理论位置重置，避免位置漂移
4. **位置限制**：最终发布的位置被限制在初始位置 0.6m 范围内，防止定位偏差过大
5. **朝向保证**：最终朝向设置为初始朝向的相反方向（180°翻转）

### 6.2 限制

1. **需要一定空间**：至少需要约 0.5m × 0.5m 的空间才能执行 Shuttle
2. **依赖激光雷达**：需要激光雷达提供障碍物检测数据
3. **执行时间较长**：调头 180° 通常需要 4-6 个周期，约 20-40 秒

## 7. 车辆几何参数

插件内置的 Tianracer 车辆参数：

| 参数 | 值 | 说明 |
|------|-----|------|
| wheelbase_ | 0.40m | 轴距 L |
| track_width_ | 0.27m | 轮距 W |
| front_overhang_ | 0.13m | 前悬 |
| rear_overhang_ | 0.07m | 后悬 |
| lidar_x_offset_ | 0.20m | 激光雷达在 base_link 前方的偏移 |

## 8. 话题与服务

### 8.1 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| /cmd_vel | geometry_msgs/Twist | 速度命令 |
| /initialpose | geometry_msgs/PoseWithCovarianceStamped | 位姿初始化（完成时发布） |

### 8.2 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| /scan | sensor_msgs/LaserScan | 激光雷达数据 |

### 8.3 TF 依赖

| 变换 | 说明 |
|------|------|
| map → base_link | 机器人在地图中的位姿（AMCL） |
| odom → base_link | 里程计位姿（用于检测 AMCL 跳变） |

## 9. 使用示例

### 9.1 在 nav2_params.yaml 中配置

```yaml
bt_navigator:
  ros__parameters:
    default_bt_xml_filename: "ackermann_nav_shuttle.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_rate_controller_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - tianracer_shuttle_to_orientation_bt_node      # ShuttleToOrientation
      - tianracer_is_goal_in_front_bt_node            # IsGoalInFront
```

### 9.2 编译插件

```bash
cd ~/tianracer_ros2_ws
colcon build --packages-select tianracer_navigation_ros2
source install/setup.bash
```

## 10. 相关文档

- [阿克曼底盘 Shuttle 操作数学推导](./ackermann_shuttle_math.md)：详细的数学公式和几何分析
- [Nav2 配置说明](./nav2_config_notes.md)：Nav2 参数配置指南
- [Nav2 真实机器人调试](./nav2_real_robot_debugging.md)：调试技巧

## 11. 更新日志

- **2026-01-28**：添加位置限制功能，防止最终位置偏离初始位置超过 0.6m
- **2026-01-22**：添加 AMCL 跳变检测和理论位置重置功能
- **2026-01-20**：初始版本，实现基本 Shuttle 机动功能
