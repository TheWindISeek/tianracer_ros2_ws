from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('tianracer_slam_ros2')
    
    # Get lidar type from environment variable or use default
    lidar_env = EnvironmentVariable('TIANBOT_LIDAR', default_value='osight')
    
    # Declare launch arguments
    lidar_arg = DeclareLaunchArgument(
        'lidar',
        default_value=lidar_env,
        description='Lidar type: osight, richbeam, or rplidar'
    )
    
    # Determine max range based on lidar type
    max_urange = '20'  # default for osight and richbeam
    # Note: In ROS2, we need to handle conditional parameters differently
    # For now, we'll use a default value. User can override via parameter file
    
    # SLAM Toolbox node (ROS2 replacement for gmapping)
    # Note: In ROS2, gmapping is typically replaced by slam_toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'map_frame': 'map',
            'max_laser_range': 20.0,  # Adjust based on lidar
            'map_update_interval': 0.5,
            'resolution': 0.05,
            'minimum_travel_distance': 0.1,
            'minimum_travel_heading': 0.1,
            'scan_buffer_maximum_scan_distance': 20.0,
            'scan_buffer_size': 10,
            'link_scan_maximum_distance': 1.5,
            'loop_search_maximum_distance': 3.0,
            'do_loop_closing': True,
            'loop_match_minimum_chain_size': 10,
            'loop_match_maximum_variance_coarse': 3.0,
            'loop_match_minimum_response_coarse': 0.1,
            'loop_match_minimum_response_fine': 0.1,
            # Message filter parameters to prevent queue overflow
            'message_filter_queue_size': 50,  # Increase queue size (default is usually 10)
            'transform_tolerance': 0.5,  # Time tolerance for TF lookups (seconds)
            'tf_buffer_duration': 10.0,  # Duration of TF buffer (seconds)
        }],
        remappings=[
            ('/scan', '/scan'),
        ]
    )
    
    # Static TF publishers for complete TF chain
    # Required TF chain: odom -> base_footprint -> base_link -> laser
    # Note: If you already have odom -> base_link, we need:
    #   1. base_link -> base_footprint (or base_footprint -> base_link, depending on your setup)
    #   2. base_link -> laser (or base_footprint -> laser)
    
    # Option 1: If your /scan frame_id is "laser" and you have base_link
    # Publish base_link -> base_footprint (typically base_footprint is at ground level, base_link is slightly above)
    base_link_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        output='screen'
    )
    
    # Publish base_link -> laser (adjust x, y, z based on your actual laser position)
    # Common values: x=0.15-0.22m forward, z=0.1m up
    # Check your actual laser mounting position and adjust accordingly
    base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.15', '0.0', '0.1', '0.0', '0.0', '0.0', 'base_link', 'laser'],
        output='screen'
    )
    
    # Alternative: If your /scan frame_id is "base_laser_link" instead of "laser"
    # Uncomment and use this instead:
    # base_link_to_base_laser_link = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_link_to_base_laser_link',
    #     arguments=['0.15', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'base_laser_link'],
    #     output='screen'
    # )
    
    return LaunchDescription([
        lidar_arg,
        base_link_to_base_footprint,
        base_link_to_laser,
        slam_toolbox_node,
    ])

