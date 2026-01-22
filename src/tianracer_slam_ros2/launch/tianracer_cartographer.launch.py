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

    # Static TF: base_link -> base_footprint
    base_link_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'base_footprint'],
    )

    # Static TF: base_link -> lidar (激光雷达的 frame_id)
    base_link_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_lidar_tf',
        arguments=['--x', '0.15', '--y', '0', '--z', '0.1', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'lidar'],
    )

    # Static TF: base_link -> camera_link (相机在 lidar 前面约 3cm)
    base_link_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_tf',
        arguments=['--x', '0.18', '--y', '0', '--z', '0.1', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
    )

    return LaunchDescription([
        use_sim_time_arg,
        configuration_basename_arg,
        base_link_to_base_footprint,
        base_link_to_lidar,
        base_link_to_camera,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])

