# Nav2 错误分析与解决方案

## 问题总结

根据运行日志，主要存在以下问题：

### 1. ⚠️ **TF 变换错误（最严重）**

#### 错误信息：
```
[ERROR] Timed out waiting for transform from base_link to odom to become available
tf error: Invalid frame ID "odom" passed to canTransform argument target_frame - frame does not exist

[ERROR] Timed out waiting for transform from base_link to map to become available
tf error: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist
```

#### 原因：
- **缺少 `odom` 帧**：`tianbot_core` 节点未启动，无法发布 `odom` -> `base_link` 的变换
- **缺少 `map` 帧**：AMCL 需要先有 `odom` 帧才能发布 `map` -> `odom` 的变换

#### 解决方案：
**必须按正确顺序启动节点：**

```bash
# 终端 1: 启动机器人驱动（必须先启动！）
source boot/bringup.sh
# 或者
ros2 launch tianbot_core_ros2 tianbot_core.launch.py

# 终端 2: 启动激光雷达（如果需要）
source boot/osight_lidar.sh
# 或者
ros2 launch osight_lidar_ros2 osight_iexxx.launch.py

# 终端 3: 启动导航（在驱动启动后再启动）
source boot/nav2.sh
# 或者
ros2 launch tianracer_navigation_ros2 navigation_sim.launch.py \
    params_file:=/home/lucifer/tianracer_ros2_ws/install/tianracer_navigation_ros2/share/tianracer_navigation_ros2/config/nav2_params_real.yaml
```

**验证 TF 树：**
```bash
# 检查 TF 树是否完整
ros2 run tf2_tools view_frames

# 检查 odom 帧是否存在
ros2 run tf2_ros tf2_echo odom base_link

# 检查 map 帧是否存在
ros2 run tf2_ros tf2_echo map odom
```

---

### 2. ⚠️ **膨胀半径配置过小**

#### 错误信息：
```
[ERROR] The configured inflation radius (0.050) is smaller than the computed inscribed radius (0.186) of your footprint
[ERROR] The configured inflation radius (0.050) is smaller than the computed inscribed radius (0.225) of your footprint
```

#### 原因：
- 配置的 `inflation_radius: 0.05` 小于机器人的内切半径
- 局部代价地图：内切半径 0.186m
- 全局代价地图：内切半径 0.225m

#### 解决方案：
**已修复配置文件** `nav2_params_real.yaml`：
- 局部代价地图：`inflation_radius: 0.25` (≥ 0.186m)
- 全局代价地图：`inflation_radius: 0.30` (≥ 0.225m)

**重新编译并运行：**
```bash
cd ~/tianracer_ros2_ws
colcon build --packages-select tianracer_navigation_ros2
source install/setup.bash
```

---

### 3. ⚠️ **路径规划失败**

#### 错误信息：
```
[WARN] GridBased: failed to create plan with tolerance 0.50
[WARN] Planning algorithm GridBased failed to generate a valid path to (-1.52, 0.12)
```

#### 可能原因：
1. 目标点位于障碍物上
2. 目标点超出地图范围
3. 目标点与起点之间被障碍物完全阻挡
4. 地图数据有问题

#### 解决方案：
1. **检查目标点位置**：确保目标点在地图范围内且不在障碍物上
2. **在 RViz2 中设置初始位置**：
   - 点击 "2D Pose Estimate" 按钮
   - 在地图上点击机器人实际位置
   - 拖拽设置机器人朝向
3. **检查地图质量**：
   ```bash
   # 查看地图信息
   ros2 topic echo /map --once
   ```
4. **尝试不同的目标点**：选择明显可达的位置

---

### 4. ⚠️ **轨迹碰撞错误**

#### 错误信息：
```
[ERROR] No valid trajectories out of 440!
[ERROR] BaseObstacle/Trajectory Hits Obstacle
[ERROR] ObstacleFootprint/Trajectory Hits Obstacle
```

#### 原因：
- 所有生成的轨迹都碰撞障碍物
- 可能与膨胀半径过小、传感器数据异常、地图问题有关

#### 解决方案：
1. **已修复膨胀半径**（见问题 2）
2. **检查激光雷达数据**：
   ```bash
   ros2 topic echo /scan --once
   ```
3. **检查代价地图**：在 RViz2 中查看 `local_costmap` 和 `global_costmap`
4. **调整控制器参数**（如果问题持续）：
   - 减小 `max_vel_x` 和 `max_vel_theta`
   - 增加 `sim_time`（预测时间）
   - 调整 critic 权重

---

### 5. ⚠️ **行为树超时警告**

#### 错误信息：
```
[WARN] Behavior Tree tick rate 100.00 was exceeded!
[WARN] Exceeded time allowance before reaching the Spin goal - Exiting Spin
```

#### 原因：
- 行为树处理速度不够快
- 旋转行为超时

#### 解决方案：
1. **增加超时时间**（在 `nav2_params_real.yaml` 中）：
   ```yaml
   behavior_server:
     spin:
       plugin: "nav2_behaviors/Spin"
       spin_max_rotational_vel: 1.0  # 减小旋转速度
       spin_min_rotational_vel: 0.4
       simulated_award_time: 10.0    # 增加超时时间
   ```
2. **检查系统性能**：确保 CPU 和内存充足

---

## 正确的启动流程

### 真机环境启动顺序：

```bash
# ===== 终端 1: 机器人驱动（必须先启动！）=====
cd ~/tianracer_ros2_ws
source install/setup.bash
source boot/bringup.sh

# ===== 终端 2: 激光雷达（如果需要）=====
cd ~/tianracer_ros2_ws
source install/setup.bash
source boot/osight_lidar.sh

# ===== 终端 3: 导航系统 =====
cd ~/tianracer_ros2_ws
source install/setup.bash
source boot/nav2.sh

# ===== 终端 4: RViz2 可视化 =====
cd ~/tianracer_ros2_ws
source install/setup.bash
rviz2 -d config/lab.rviz
```

### 在 RViz2 中的操作：

1. **设置初始位置（必须！）**：
   - 点击工具栏 "2D Pose Estimate" 按钮
   - 在地图上点击机器人实际位置
   - 拖拽设置机器人朝向

2. **发送导航目标**：
   - 点击工具栏 "2D Nav Goal" 按钮
   - 在地图上点击目标位置
   - 拖拽设置目标朝向

---

## 验证系统是否正常

### 1. 检查节点：
```bash
ros2 node list | grep -E "(tianbot|nav2|amcl|map_server)"
```

应该看到：
- `/tianbot_chasis` 或类似节点（发布 odom）
- `/map_server`
- `/amcl`
- `/controller_server`
- `/planner_server`
- `/bt_navigator`

### 2. 检查话题：
```bash
ros2 topic list | grep -E "(odom|scan|map|amcl)"
```

应该看到：
- `/odom` - 里程计数据
- `/scan` - 激光雷达数据
- `/map` - 地图数据
- `/amcl_pose` - AMCL 定位结果

### 3. 检查 TF 树：
```bash
ros2 run tf2_tools view_frames
# 会生成 frames.pdf 文件，检查是否包含：
# map -> odom -> base_link -> base_footprint
```

### 4. 检查 TF 变换：
```bash
# 检查 odom -> base_link
ros2 run tf2_ros tf2_echo odom base_link

# 检查 map -> odom
ros2 run tf2_ros tf2_echo map odom
```

---

## 常见问题排查

### Q: 为什么看不到地图？
A: 
1. 检查 Fixed Frame 是否设置为 `map`
2. 在 RViz2 中添加 Map 显示
3. 检查 `/map` 话题是否有数据：`ros2 topic echo /map --once`

### Q: AMCL 定位失败？
A:
1. **必须先设置初始位置**（在 RViz2 中使用 "2D Pose Estimate"）
2. 确保 `odom` 帧存在：`ros2 run tf2_ros tf2_echo odom base_link`
3. 确保激光雷达数据正常：`ros2 topic echo /scan --once`

### Q: 路径规划一直失败？
A:
1. 确保目标点不在障碍物上
2. 确保目标点在地图范围内
3. 检查地图质量（是否有太多未知区域）
4. 尝试不同的目标点

### Q: 机器人不移动？
A:
1. 检查速度命令是否发布：`ros2 topic echo /cmd_vel`
2. 检查控制器是否激活：查看日志中是否有 `controller_server` 的错误
3. 检查是否有有效的轨迹：查看日志中是否有 "No valid trajectories" 错误

---

## 配置文件修改记录

### nav2_params_real.yaml

**修改内容：**
- 局部代价地图 `inflation_radius`: `0.05` → `0.25`
- 全局代价地图 `inflation_radius`: `0.05` → `0.30`

**原因：**
- 原配置小于机器人内切半径，会导致碰撞风险
- 新配置符合 Nav2 推荐值（≥ 内切半径）

---

## 参考文档

- [Nav2 官方文档](https://docs.nav2.org/)
- [AMCL 配置指南](https://docs.nav2.org/configuration/packages/configuring-amcl.html)

