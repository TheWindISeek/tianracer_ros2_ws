from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/cmd_vel',
            description='Input topic for cmd_vel (Twist messages)'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/tianracer/ackermann_cmd',
            description='Output topic for ackermann_cmd (AckermannDrive messages)'
        ),
        DeclareLaunchArgument(
            'output_type',
            default_value='auto',
            description='Output type: auto, twist, or ackermann'
        ),
        DeclareLaunchArgument(
            'wheelbase',
            default_value='0.40',
            description='Wheelbase in meters'
        ),
        DeclareLaunchArgument(
            'steering_offset_degrees',
            default_value='0.0',
            description='Steering offset in degrees to compensate for misaligned front wheels'
        ),
        Node(
            package='tianracer_teleop_ros2',
            executable='cmd_vel_to_ackermann.py',
            name='cmd_vel_to_ackermann',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'output_type': LaunchConfiguration('output_type'),
                'wheelbase': LaunchConfiguration('wheelbase'),
                'steering_offset_degrees': LaunchConfiguration('steering_offset_degrees'),
            }]
        ),
    ])

