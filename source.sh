#!/usr/bin/bash

# 根据当前用户判断使用哪个配置
if [ "$USER" = "lucifer" ]; then
    # lucifer 用户的配置
    #source /opt/ros/humble/local_setup.bash
    #source ~/ros2_humble/install/local_setup.bash
    #source ~/nav2_ws/install/local_setup.bash
    source /opt/ros/humble/local_setup.bash
    source ~/ros2_humble/install/local_setup.bash
    source ~/nav2_ws/install/local_setup.bash
else
    # 其他用户的配置
    source ~/ros2_humble/install/local_setup.bash 
    source ~/nav2_ws/install/local_setup.bash 
    source ~/cv_ws/install/local_setup.bash 
    source ~/bt_ws/install/local_setup.bash 
    source ~/diagnostics_ws/install/local_setup.bash 
    source ~/bond_ws/install/local_setup.bash 
    source ~/angles_ws/install/local_setup.bash 
fi

# 添加 Gazebo 模型路径
# export GAZEBO_MODEL_PATH=/home/lucifer/.gazebo/models:/home/lucifer/Codes/gazebo_models:/home/lucifer/Codes/ros2_warehouse_amr/src/ros2_warehouse_amr/models:/home/lucifer/Codes/ros2_warehouse_amr/src/ros2_warehouse_amr/worlds:$GAZEBO_MODEL_PATH
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=/home/lucifer/.gazebo/models:/home/lucifer/Codes/gazebo_models:/home/lucifer/Codes/ros2_warehouse_amr/src/ros2_warehouse_amr/models:/home/lucifer/Codes/ros2_warehouse_amr/src/ros2_warehouse_amr/worlds::/opt/ros/humble/share/turtlebot3_gazebo/models:/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
source install/local_setup.bash
