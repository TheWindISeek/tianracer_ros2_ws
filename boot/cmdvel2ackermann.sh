# 固件已设置 steering_offset，上位机不再需要补偿
ros2 launch tianracer_teleop_ros2 cmd_vel_to_ackermann.launch.py output_topic:=/ackermann_cmd steering_offset_degrees:=0.0