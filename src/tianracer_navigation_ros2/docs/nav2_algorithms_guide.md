# Nav2 导航算法配置指南

## 目录
1. [概述](#概述)
2. [全局路径规划器](#全局路径规划器)
3. [局部控制器](#局部控制器)
4. [代价地图配置](#代价地图配置)
5. [行为树配置](#行为树配置)
6. [阿克曼底盘特殊配置](#阿克曼底盘特殊配置)
7. [常见问题与解决方案](#常见问题与解决方案)

---

## 概述

Nav2 导航栈主要由以下组件组成：

```
┌─────────────────────────────────────────────────────────────┐
│                      bt_navigator                           │
│                    (行为树导航器)                            │
└─────────────────────────────────────────────────────────────┘
                            │
            ┌───────────────┼───────────────┐
            ▼               ▼               ▼
    ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
    │planner_server│ │controller    │ │behavior      │
    │ (全局规划器)  │ │_server       │ │_server       │
    │              │ │ (局部控制器)  │ │ (恢复行为)    │
    └──────────────┘ └──────────────┘ └──────────────┘
            │               │
            ▼               ▼
    ┌──────────────┐ ┌──────────────┐
    │global_costmap│ │local_costmap │
    │ (全局代价图)  │ │ (局部代价图)  │
    └──────────────┘ └──────────────┘
```

---

## 全局路径规划器

### 1. NavFn Planner (nav2_navfn_planner)

**原理**：基于 Dijkstra 或 A* 算法的栅格搜索规划器。

**特点**：
- 简单、快速、稳定
- 不考虑机器人运动学约束
- 生成的路径可能包含急转弯

**适用机器人**：
- ✅ 差速驱动 (Differential Drive)
- ✅ 全向轮 (Omnidirectional)
- ⚠️ 阿克曼底盘 (需要配合能处理运动学的控制器)

**配置参数**：
```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5          # 目标容差（米）
      use_astar: true         # true=A*, false=Dijkstra
      allow_unknown: true     # 是否允许穿过未知区域
```

---

### 2. Smac Planner 2D (nav2_smac_planner/SmacPlanner2D)

**原理**：基于 A* 的 2D 栅格规划器，比 NavFn 更现代化。

**特点**：
- 支持 4 连通或 8 连通搜索
- 更好的代价评估
- 不考虑运动学约束

**适用机器人**：
- ✅ 差速驱动
- ✅ 全向轮
- ⚠️ 阿克曼底盘

**配置参数**：
```yaml
GridBased:
  plugin: "nav2_smac_planner/SmacPlanner2D"
  tolerance: 0.5
  allow_unknown: true
  max_iterations: 1000000
  max_planning_time: 5.0
  cost_travel_multiplier: 2.0
  use_final_approach_orientation: false
```

---

### 3. Smac Planner Hybrid (nav2_smac_planner/SmacPlannerHybrid) ⭐推荐

**原理**：Hybrid-A* 算法，结合连续状态空间和离散搜索。

**特点**：
- 考虑机器人运动学约束（最小转弯半径）
- 支持 Reeds-Shepp 曲线（前进+倒车）
- 支持 Dubin 曲线（仅前进）
- 生成平滑、可执行的路径

**适用机器人**：
- ✅ 阿克曼底盘 (Ackermann) - **最佳选择**
- ✅ 差速驱动
- ❌ 全向轮（不需要这么复杂的规划器）

**配置参数**：
```yaml
GridBased:
  plugin: "nav2_smac_planner/SmacPlannerHybrid"
  tolerance: 0.5
  allow_unknown: true
  max_iterations: 1000000
  max_planning_time: 5.0

  # 运动模型
  motion_model: "REEDS_SHEPP"  # 或 "DUBIN"
  # REEDS_SHEPP: 支持前进和倒车
  # DUBIN: 仅支持前进

  # 关键参数：最小转弯半径
  minimum_turning_radius: 0.45  # 根据实际机器人测量

  # 角度分辨率
  angle_quantization_bins: 72   # 72 bins = 5度/bin

  # 惩罚系数
  reverse_penalty: 1.0          # 倒车惩罚（1.0=无惩罚）
  change_penalty: 0.0           # 方向改变惩罚
  non_straight_penalty: 1.0     # 非直线惩罚
  cost_penalty: 2.0             # 代价惩罚

  # 分析扩展
  analytic_expansion_ratio: 3.5
  analytic_expansion_max_length: 3.0

  smooth_path: true
```

**最小转弯半径计算**：
```
R = L / tan(θ_max)

其中：
- R = 最小转弯半径
- L = 轴距 (wheelbase)
- θ_max = 最大转向角
```

---

### 4. Smac Planner Lattice (nav2_smac_planner/SmacPlannerLattice)

**原理**：基于预计算的运动原语（Motion Primitives）进行搜索。

**特点**：
- 使用离线生成的运动原语
- 可以精确建模复杂的运动学
- 需要额外的运动原语文件

**适用机器人**：
- ✅ 任意运动学模型（需要对应的运动原语）

---

### 5. Theta Star Planner (nav2_theta_star_planner)

**原理**：Theta* 算法，A* 的变体，允许任意角度的路径。

**特点**：
- 生成更平滑的路径
- 减少不必要的转弯
- 不考虑运动学约束

**适用机器人**：
- ✅ 全向轮
- ⚠️ 差速驱动
- ❌ 阿克曼底盘

---

## 局部控制器

### 1. DWB Controller (nav2_dwb_controller)

**原理**：Dynamic Window Approach，在速度空间中采样并评估轨迹。

**特点**：
- 经典算法，稳定可靠
- 基于评分函数选择最优轨迹
- 适合差速驱动

**适用机器人**：
- ✅ 差速驱动
- ⚠️ 阿克曼底盘（需要特殊配置）

**配置参数**：
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

      # 速度限制
      min_vel_x: -0.3
      max_vel_x: 0.5
      min_vel_y: 0.0      # 非全向设为0
      max_vel_y: 0.0
      max_vel_theta: 1.0

      # 加速度限制
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2

      # 轨迹生成
      sim_time: 1.7
      vx_samples: 20
      vy_samples: 1
      vtheta_samples: 20

      # 评分器
      critics: ["RotateToGoal", "Oscillation", "ObstacleFootprint", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

---

### 2. MPPI Controller (nav2_mppi_controller) ⭐推荐

**原理**：Model Predictive Path Integral，基于采样的模型预测控制。

**特点**：
- 现代化的控制算法
- 原生支持阿克曼运动学
- 可配置多种评估器（Critics）
- 预测性控制，轨迹更平滑

**适用机器人**：
- ✅ 阿克曼底盘 - **最佳选择**
- ✅ 差速驱动
- ✅ 全向轮

**配置参数**：
```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"

      # 预测参数
      time_steps: 56          # 预测步数
      model_dt: 0.1           # 每步时间间隔
      batch_size: 2000        # 采样轨迹数量

      # 速度限制
      vx_max: 0.5
      vx_min: -0.3            # 负值允许倒车
      vy_max: 0.0
      wz_max: 1.0

      # 运动模型
      motion_model: "Ackermann"  # 或 "DiffDrive", "Omni"

      # 阿克曼约束
      AckermannConstraints:
        min_turning_r: 0.45   # 最小转弯半径

      # 评估器
      critics: [
        "ConstraintCritic",
        "CostCritic",
        "GoalCritic",
        "GoalAngleCritic",
        "PathAlignCritic",
        "PathFollowCritic",
        "PathAngleCritic",
        "PreferForwardCritic"
      ]

      # 各评估器权重
      GoalCritic:
        cost_weight: 5.0
        threshold_to_consider: 1.0

      PathFollowCritic:
        cost_weight: 5.0
        offset_from_furthest: 5
        threshold_to_consider: 1.0

      PreferForwardCritic:
        cost_weight: 5.0
        threshold_to_consider: 0.4
```

---

### 3. Regulated Pure Pursuit (nav2_regulated_pure_pursuit_controller)

**原理**：改进的纯追踪算法，根据曲率和障碍物调节速度。

**特点**：
- 简单直观
- 自动减速过弯
- 适合高速场景

**适用机器人**：
- ✅ 阿克曼底盘
- ✅ 差速驱动

---

## 代价地图配置

### 代价地图层

```yaml
local_costmap:
  ros__parameters:
    plugins: ["voxel_layer", "inflation_layer"]

    # 体素层：处理3D障碍物
    voxel_layer:
      plugin: "nav2_costmap_2d::VoxelLayer"
      enabled: true
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        raytrace_max_range: 3.0
        obstacle_max_range: 2.5

    # 膨胀层：障碍物周围的安全区域
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 10.0   # 代价衰减速度
      inflation_radius: 0.08      # 膨胀半径
```

### 关键参数说明

| 参数 | 作用 | 调优建议 |
|------|------|----------|
| `inflation_radius` | 障碍物膨胀半径 | 太大会导致障碍物合并，太小会贴近障碍物 |
| `cost_scaling_factor` | 代价衰减速度 | 值越大，代价衰减越快 |
| `robot_radius` | 机器人半径 | 用于碰撞检测 |
| `resolution` | 地图分辨率 | 一般 0.05m |

---

## 行为树配置

### 阿克曼底盘专用行为树

阿克曼底盘无法原地旋转，需要移除 `Spin` 行为：

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <!-- 导航逻辑 -->
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <Sequence name="ClearingActions">
            <ClearEntireCostmap service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>
          <!-- 注意：没有 Spin -->
          <Wait wait_duration="3"/>
          <BackUp backup_dist="0.30" backup_speed="0.10"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

### 配置使用自定义行为树

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: /path/to/ackermann_nav.xml
    default_nav_through_poses_bt_xml: /path/to/ackermann_nav_through_poses.xml

behavior_server:
  ros__parameters:
    behavior_plugins: ["backup", "wait"]  # 移除 "spin"
```

---

## 阿克曼底盘特殊配置

### 运动学约束

阿克曼底盘的关键约束：
1. **最小转弯半径**：由轴距和最大转向角决定
2. **无法原地旋转**：必须移除 Spin 行为
3. **倒车能力**：需要 REEDS_SHEPP 运动模型

### 推荐配置组合

```
全局规划器: SmacPlannerHybrid (REEDS_SHEPP)
局部控制器: MPPI (Ackermann motion model)
行为树: 自定义（无 Spin）
```

### 最小转弯半径测量方法

1. 让机器人以最大转向角行驶
2. 记录 `cmd_vel` 中的 `linear.x` 和 `angular.z`
3. 计算：`R = linear.x / angular.z`

---

## 常见问题与解决方案

### 1. "Starting point in lethal space" 错误

**原因**：机器人位置被标记为障碍物

**解决方案**：
- 检查 `inflation_radius` 是否过大
- 检查机器人定位是否准确
- 清除代价地图：`ros2 service call /local_costmap/clear_entirely_local_costmap`

---

### 2. Spin 行为超时循环

**原因**：阿克曼底盘无法原地旋转

**解决方案**：
- 使用自定义行为树，移除 Spin
- 从 `behavior_plugins` 中移除 "spin"

---

### 3. 路径规划绕大圈

**原因**：
- `minimum_turning_radius` 设置过大
- 惩罚系数不合理

**解决方案**：
```yaml
minimum_turning_radius: 0.45  # 根据实际测量
reverse_penalty: 1.0          # 允许倒车
change_penalty: 0.0           # 允许换向
```

---

### 4. 障碍物在代价地图中合并

**原因**：`inflation_radius` 过大

**解决方案**：
```yaml
inflation_radius: 0.08        # 减小膨胀半径
cost_scaling_factor: 10.0     # 增大衰减速度
```

---

### 5. 倒车时提前转向

**原因**：MPPI 控制器的预测性控制提前规划转向

**解决方案**：
```yaml
GoalAngleCritic:
  threshold_to_consider: 0.4  # 减小阈值
PathAngleCritic:
  threshold_to_consider: 0.4
PreferForwardCritic:
  threshold_to_consider: 0.4
```

---

### 6. 激光雷达后方盲区

**原因**：270° FOV 的激光雷达后方有 90° 盲区

**解决方案**：
- 依赖静态地图中的墙壁信息
- 限制倒车距离
- 添加后方传感器（超声波等）

---

## 配置文件备份

当前项目中的配置文件备份：
- `nav2_params_real.yaml.smac_backup` - SmacPlannerHybrid 配置
- `nav2_params_real.yaml.dwb_backup` - DWB Controller 配置

切换配置：
```bash
# 切换到 SmacPlanner
cp nav2_params_real.yaml.smac_backup nav2_params_real.yaml

# 切换到 NavFn
cp nav2_params_real.yaml.navfn_backup nav2_params_real.yaml
```

---

## 参考资料

- [Nav2 官方文档](https://docs.nav2.org/)
- [SmacPlanner 配置指南](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html)
- [MPPI Controller 配置指南](https://docs.nav2.org/configuration/packages/configuring-mppi-controller.html)
