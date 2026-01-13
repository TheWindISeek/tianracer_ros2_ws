from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('tianracer_slam_ros2')
    config_dir = os.path.join(pkg_share, 'param')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulation time'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', '3d_points.lua'
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('odom', 'odometry/filtered'),
            ('imu', 'tianracer/imu'),
            ('points2', 'lidar_points'),
        ]
    )
    
    # Cartographer occupancy grid node
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', '0.05'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # RViz node (optional)
    # Note: Check if tianracer_rviz package exists before using it
    try:
        rviz_config_path = os.path.join(
            get_package_share_directory('tianracer_rviz'),
            'rviz_cfg', 'demo_3d.rviz'
        )
        rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []
    except:
        rviz_args = []
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=rviz_args
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
    ])

