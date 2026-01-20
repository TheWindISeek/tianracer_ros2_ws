# Tianracer Nav2 配置说明

## 1. FastDDS XML 配置

### 问题
Nav2 lifecycle 节点启动时出现超时错误，节点无法正常激活。

### 原因
ROS2 默认使用 FastDDS 作为 DDS 中间件。默认的 lease duration 设置较短，在节点启动较慢时会导致发现协议超时。

### 解决方案
创建 `config/fastdds.xml`：
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <leaseDuration>
                        <sec>DURATION_INFINITY</sec>
                    </leaseDuration>
                    <leaseAnnouncement>
                        <sec>5</sec>
                    </leaseAnnouncement>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

### 使用方法
启动前设置环境变量：
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/lucifer/tianracer_ros2_ws/config/fastdds.xml
```

或在 `.bashrc` 中添加。

---

## 2. 倒车配置

### 问题
小车在狭窄空间无法倒车脱困，导致 "Failed to make progress" 错误。

### 原因
`nav2_params_sim.yaml` 中 `min_vel_x` 默认为 `0.0`，禁止了负速度（倒车）。

### 解决方案
修改 `config/nav2_params_sim.yaml` 中 controller_server 的 DWB 参数：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      # 允许倒车
      min_vel_x: -0.1

      # 放宽进度检查（避免误判卡住）
      required_movement_radius: 0.1  # 原 0.5
      movement_time_allowance: 20.0  # 原 10.0
```

### 参数说明
| 参数 | 说明 | 建议值 |
|------|------|--------|
| `min_vel_x` | 最小线速度，负值允许倒车 | `-0.1` |
| `required_movement_radius` | 判定"有进展"的最小移动距离 | `0.1` |
| `movement_time_allowance` | 允许无进展的最大时间(秒) | `20.0` |

---

## 3. 自定义 RViz 配置

### 使用本地 RViz 配置文件
```bash
# 方法1: 直接启动 rviz2
rviz2 -d /home/lucifer/tianracer_ros2_ws/config/lab.rviz

# 方法2: 在 launch 文件中添加 rviz_config_file 参数
```

### 在 launch 文件中添加 RViz 节点
```python
from launch_ros.actions import Node

rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', '/home/lucifer/tianracer_ros2_ws/config/lab.rviz'],
    parameters=[{'use_sim_time': True}]
)
```

---

## 4. 转向补偿配置（真车）

### 问题
真车转向存在机械偏差，直行时实际会偏转。

### 解决方案
在 `tianbot_core_ros2` 中添加了 `yaw_offset_deg` 参数，用于补偿 odom 的 yaw 值。

### 使用方法
```bash
# 使用默认补偿值 (-28.67°)
ros2 launch tianracer_bringup_ros2 tianracer_bringup.launch.py

# 自定义补偿值
ros2 launch tianracer_bringup_ros2 tianracer_bringup.launch.py yaw_offset_deg:=-30.0

# 禁用补偿
ros2 launch tianracer_bringup_ros2 tianracer_bringup.launch.py yaw_offset_deg:=0.0
```

---

## 5. 常用启动命令

### 仿真环境
```bash
# 终端1: Gazebo
ros2 launch tianbot_description_ros2 gazebo_robot.launch.py x:=0.5 y:=0.5 world:=/home/lucifer/tianracer_ros2_ws/world/lab.sdf

# 终端2: Nav2
ros2 launch tianracer_navigation_ros2 navigation_sim.launch.py

# 终端3: RViz (可选)
rviz2 -d /home/lucifer/tianracer_ros2_ws/config/lab.rviz
```

### 真车环境
```bash
# 启动硬件
ros2 launch tianracer_bringup_ros2 tianracer_bringup.launch.py

# 启动导航 (使用真车参数)
ros2 launch tianracer_navigation_ros2 navigation.launch.py
```
