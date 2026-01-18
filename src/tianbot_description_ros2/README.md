# Launch 文件使用指南

## Launch文件说明

### `description.launch.py` - 发布机器人模型
发布机器人URDF模型，用于RViz2可视化或实际硬件运行。

```bash
ros2 launch tianbot_description_ros2 description.launch.py
```

### `includes/tianbot_ackermann.launch.py` - 发布传感器TF
发布传感器TF变换，用于实际硬件运行时调整传感器位置（当URDF位置与实际不匹配时）。

```bash
export TIANBOT_LIDAR=osight  # 可选: rplidar/osight/rslidar
ros2 launch tianbot_description_ros2 includes/tianbot_ackermann.launch.py
```

### `gazebo.launch.py` - 仅启动Gazebo
只启动Gazebo仿真环境，不加载机器人。

```bash
ros2 launch tianbot_description_ros2 gazebo.launch.py
ros2 launch tianbot_description_ros2 gazebo.launch.py world:=/path/to/world.world gui:=false
```

### `gazebo_robot.launch.py` - Gazebo + 机器人
同时启动Gazebo和机器人模型（最常用）。

```bash
# 默认空world
ros2 launch tianbot_description_ros2 gazebo_robot.launch.py

# 指定world文件
ros2 launch tianbot_description_ros2 gazebo_robot.launch.py world:=/path/to/world.world

# 指定world和机器人初始位置
ros2 launch tianbot_description_ros2 gazebo_robot.launch.py \
    world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world \
    x:=1.0 y:=2.0 yaw:=1.57

# 无GUI模式
ros2 launch tianbot_description_ros2 gazebo_robot.launch.py world:=/path/to/world.world gui:=false
```

## World文件指定方法

### 方法1: 完整路径
```bash
world:=/home/lucifer/tianracer_ros2_ws/worlds/my_world.world
```

### 方法2: 相对路径
```bash
cd /home/lucifer/tianracer_ros2_ws
world:=./worlds/my_world.world
```

### 方法3: 系统Gazebo world
```bash
world:=/usr/share/gazebo-11/worlds/empty.world
```

### 方法4: ROS2包中的world
```bash
world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world
```

## 常用World文件路径

```bash
# Turtlebot3 world
/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world

# Gazebo默认world
/usr/share/gazebo-11/worlds/empty.world
/usr/share/gazebo-11/worlds/willowgarage.world
```

## 查找World文件

```bash
# 查找所有.world文件
find /opt/ros/humble -name "*.world" 2>/dev/null
find /usr/share/gazebo* -name "*.world" 2>/dev/null

# 查找特定包的路径
ros2 pkg prefix turtlebot3_gazebo
```

## 完整示例

```bash
# 1. 仅可视化机器人（RViz2）
ros2 launch tianbot_description_ros2 description.launch.py
# 然后另开终端: ros2 run rviz2 rviz2

# 2. Gazebo仿真（空world）
ros2 launch tianbot_description_ros2 gazebo_robot.launch.py

# 3. Gazebo仿真（指定world）
ros2 launch tianbot_description_ros2 gazebo_robot.launch.py \
    world:=/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world

# 4. 实际硬件运行（需要调整传感器位置）
ros2 launch tianbot_description_ros2 description.launch.py use_sim_time:=false
export TIANBOT_LIDAR=osight
ros2 launch tianbot_description_ros2 includes/tianbot_ackermann.launch.py
```

## 参数说明

### gazebo_robot.launch.py 参数
- `world`: World文件路径（默认：空）
- `x`, `y`, `z`: 机器人初始位置（默认：0.0, 0.0, 0.065）
- `yaw`: 机器人初始朝向弧度（默认：0.0）
- `gui`: 是否显示GUI（默认：true）
- `use_sim_time`: 使用仿真时间（默认：true）
- `urdf_file`: URDF文件路径（默认：包内tianbot_ackermann.urdf.xacro）
- `robot_name`: 机器人实体名称（默认：tianbot_ackermann）
