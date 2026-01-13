from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os


def generate_launch_description():
    # 获取环境变量，默认为 'dji_dt7'
    joy_mode_default = os.environ.get('TIANRACER_JOY_MODE', 'dji_dt7')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_mode',
            default_value=joy_mode_default,
            description='Joystick mode: dji_dt7, X, D, etc.'
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/tianbot_joystick',
            description='Joystick device path'
        ),
        DeclareLaunchArgument(
            'throttle_scale',
            default_value='1.0',
            description='Throttle scale factor'
        ),
        DeclareLaunchArgument(
            'servo_scale',
            default_value='1.0',
            description='Servo scale factor'
        ),
        
        # 只有在非dji模式下才启动joy_node和tianracer_joy
        GroupAction(
            condition=UnlessCondition(
                PythonExpression(["'dji' in '", LaunchConfiguration('joy_mode'), "'"])
            ),
            actions=[
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joystick',
                    parameters=[{
                        'dev': LaunchConfiguration('joy_dev'),
                    }]
                ),
                Node(
                    package='tianracer_teleop_ros2',
                    executable='tianracer_joy.py',
                    name='tianracer_joy',
                    parameters=[{
                        'joy_mode': LaunchConfiguration('joy_mode'),
                        'throttle_scale': LaunchConfiguration('throttle_scale'),
                        'servo_scale': LaunchConfiguration('servo_scale'),
                    }]
                ),
            ]
        ),
    ])

