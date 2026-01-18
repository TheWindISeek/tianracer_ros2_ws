# Nav2 导航配置说明

## 配置文件结构

所有 Nav2 导航参数都集中在 `config/nav2_params.yaml` 文件中，便于管理和修改。

## 如何修改参数

### 方法 1: 直接编辑 YAML 文件（推荐）

编辑 `config/nav2_params.yaml` 文件，修改对应节点的参数：

```yaml
# 例如：修改控制器速度
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.5    # 修改最大线速度
      max_vel_theta: 1.0  # 修改最大角速度
```

修改后，重新编译并启动：

```bash
cd ~/tianracer_ros2_ws
colcon build --packages-select tianracer_navigation_ros2
source install/setup.bash
ros2 launch tianracer_navigation_ros2 navigation.launch.py
```

### 方法 2: 使用自定义参数文件

```bash
ros2 launch tianracer_navigation_ros2 navigation.launch.py \
    params_file:=/path/to/your/custom_params.yaml
```

## 主要参数说明

### 1. AMCL 定位参数

位置：`amcl.ros__parameters`

- `min_particles` / `max_particles`: 粒子数量（影响定位精度和计算量）
- `laser_max_beams`: 使用的激光束数量
- `initial_pose_x/y/a`: 初始位姿（通常在 RViz2 中设置）

### 2. 控制器参数

位置：`controller_server.ros__parameters.FollowPath`

- `max_vel_x`: 最大线速度（m/s）
- `max_vel_theta`: 最大角速度（rad/s）
- `acc_lim_x`: 线加速度限制（m/s²）
- `acc_lim_theta`: 角加速度限制（rad/s²）
- `xy_goal_tolerance`: 位置容差（米）
- `yaw_goal_tolerance`: 角度容差（弧度）

### 3. 规划器参数

位置：`planner_server.ros__parameters.GridBased`

- `tolerance`: 目标容差（米）
- `allow_unknown`: 是否允许在未知区域规划

### 4. 代价地图参数

位置：`local_costmap.local_costmap.ros__parameters` 和 `global_costmap.global_costmap.ros__parameters`

- `robot_radius`: 机器人半径（米）
- `inflation_radius`: 膨胀半径（米）
- `resolution`: 分辨率（米/像素）

## 常用参数调整示例

### 提高导航速度

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.8        # 从 0.5 增加到 0.8
      max_vel_theta: 1.5    # 从 1.0 增加到 1.5
      acc_lim_x: 2.0        # 从 1.0 增加到 2.0
```

### 提高定位精度

```yaml
amcl:
  ros__parameters:
    min_particles: 1000     # 从 500 增加到 1000
    max_particles: 5000    # 从 2000 增加到 5000
    laser_max_beams: 120   # 从 60 增加到 120
```

### 调整目标容差

```yaml
controller_server:
  ros__parameters:
    general_goal_checker:
      xy_goal_tolerance: 0.15   # 从 0.25 减小到 0.15（更精确）
      yaw_goal_tolerance: 0.15  # 从 0.25 减小到 0.15
```

### 调整机器人尺寸

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.25    # 根据实际机器人尺寸调整

global_costmap:
  global_costmap:
    ros__parameters:
      robot_radius: 0.25    # 根据实际机器人尺寸调整
```

## 参数文件格式说明

Nav2 参数文件使用以下格式：

```yaml
节点名称:
  ros__parameters:
    参数名: 参数值
    嵌套参数:
      子参数: 值
```

**重要**：
- 每个节点必须有 `ros__parameters:` 键
- 参数名使用下划线命名（如 `use_sim_time`）
- 列表使用 YAML 列表格式（`-` 开头）
- 字符串值建议加引号

## 验证参数

启动导航后，可以检查参数是否正确加载：

```bash
# 检查 AMCL 参数
ros2 param get /amcl min_particles

# 检查控制器参数
ros2 param get /controller_server FollowPath.max_vel_x

# 列出所有参数
ros2 param list /controller_server
```

## 常见问题

### Q: 修改参数后没有生效？

A: 确保：
1. 重新编译了包：`colcon build --packages-select tianracer_navigation_ros2`
2. Source 了工作空间：`source install/setup.bash`
3. 重新启动了导航节点

### Q: 参数格式错误？

A: 检查 YAML 语法：
- 缩进使用空格（不是 Tab）
- 冒号后要有空格
- 列表项使用 `-` 开头

### Q: 如何查看当前使用的参数？

A: 使用 `ros2 param list` 和 `ros2 param get` 命令查看。

