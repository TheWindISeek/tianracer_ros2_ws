from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory


def generate_map_saver_process(context):
    """Generate map saver CLI process based on slam_methods argument"""
    slam_methods = context.launch_configurations.get('slam_methods', 'gmapping')
    map_path = context.launch_configurations.get('map_path', '')
    map_file = context.launch_configurations.get('map_file', 'tianbot_office')
    
    map_file_path = os.path.join(map_path, map_file)
    
    if slam_methods == 'cartographer':
        occ_threshold = '51'
        free_threshold = '49'
    else:
        occ_threshold = '65'
        free_threshold = '19'
    
    return [ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '--occ', occ_threshold,
             '--free', free_threshold,
             '-f', map_file_path],
        output='screen'
    )]


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('tianracer_slam_ros2')
    maps_dir = os.path.join(pkg_share, 'maps')
    
    # Get map file from environment variable or use default
    map_file_env = EnvironmentVariable('TIANRACER_MAP_FILE', default_value='tianbot_office')
    
    # Declare launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=map_file_env,
        description='Map file name (without extension)'
    )
    
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=maps_dir,
        description='Path to maps directory'
    )
    
    slam_methods_arg = DeclareLaunchArgument(
        'slam_methods',
        default_value='gmapping',
        description='SLAM method: gmapping, cartographer, hector, karto, frontier_exploration'
    )
    
    # Map saver CLI process (one-time save, then exits)
    # This is similar to running: ros2 run nav2_map_server map_saver_cli -f <file>
    map_saver_action = OpaqueFunction(function=generate_map_saver_process)
    
    return LaunchDescription([
        map_file_arg,
        map_path_arg,
        slam_methods_arg,
        map_saver_action,
    ])

