from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    
    configuration_basename_arg = DeclareLaunchArgument(
        'configuration_basename',
        default_value='2d_scan.lua',
        description='Cartographer configuration basename'
    )
    
    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', LaunchConfiguration('configuration_basename'),
        ],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/odom', '/odometry/filtered'),
            ('/imu', '/tianracer/imu'),
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
    
    return LaunchDescription([
        use_sim_time_arg,
        configuration_basename_arg,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])

