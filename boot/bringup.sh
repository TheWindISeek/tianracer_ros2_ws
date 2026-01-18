# 使用默认参数启动
ros2 launch tianracer_bringup_ros2 tianracer_bringup.launch.py

# 自定义参数启动
# ros2 launch tianracer_bringup_ros2 tianracer_bringup.launch.py \
    # lidar_ip:=192.168.1.10 \
    # tianbot_serial_port:=/dev/tianbot_racecar \
    # tianbot_type:=ackermann