from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('tianracer_slam_ros2')
    # Default to workspace root maps directory instead of install directory
    # This is more user-friendly and accessible
    # pkg_share is typically: <workspace>/install/<pkg>/share/<pkg>
    # So we need to go up 4 levels to get to workspace root
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    maps_dir = os.path.join(workspace_root, 'maps')
    # Create maps directory if it doesn't exist
    os.makedirs(maps_dir, exist_ok=True)
    
    # Get map file from environment variable or use default
    map_file_env = EnvironmentVariable('TIANRACER_MAP_FILE', default_value='tianracer_ros2_ws')
    
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
    
    # Determine thresholds based on SLAM method
    # Note: We'll use parameters to pass thresholds, but map_saver_server
    # will use these when saving via service call
    
    # Map saver server node (lifecycle node)
    # This is a service-based node that can be called multiple times
    # Similar to ROS1's map_saver node behavior
    map_saver_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        namespace='',
        output='screen',
        parameters=[{
            'save_map_timeout': 10000.0,  # Timeout in milliseconds (must be double type)
            'free_thresh_default': 0.19,  # Default free threshold for non-cartographer
            'occupied_thresh_default': 0.65,  # Default occupied threshold for non-cartographer
        }]
    )
    
    # Lifecycle manager to automatically activate map_saver
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_saver']
        }]
    )
    
    # Function to generate service call command
    def generate_save_map_service_call(context):
        """Generate service call to save map after node is activated"""
        slam_methods = context.launch_configurations.get('slam_methods', 'gmapping')
        map_path = context.launch_configurations.get('map_path', maps_dir)
        # Get map_file from launch configuration, with fallback to default
        # Default should match the default_value in map_file_arg (tianracer_ros2_ws)
        map_file = context.launch_configurations.get('map_file', 'tianracer_ros2_ws')
        
        # Build full file path (without extension - map_saver will add .pgm and .yaml)
        # Important: map_url must be a FILE PATH, not a directory!
        map_file_path = os.path.join(os.path.expanduser('~'), 'tianracer_ros2_ws', 'my_map')
        
        # Determine thresholds based on SLAM method
        if slam_methods == 'cartographer':
            free_thresh = '0.49'
            occ_thresh = '0.51'
        else:
            free_thresh = '0.19'
            occ_thresh = '0.65'
        
        # Build service call command
        # Note: map_url should be a file path (without extension), not a directory
        # map_saver will automatically create <map_url>.pgm and <map_url>.yaml
        service_call_cmd = [
            'ros2', 'service', 'call', '/map_saver/save_map', 'nav2_msgs/srv/SaveMap',
            f'{{map_url: "{map_file_path}", image_format: "pgm", map_mode: "trinary", free_thresh: {free_thresh}, occupied_thresh: {occ_thresh}}}'
        ]
        
        return [ExecuteProcess(
            cmd=service_call_cmd,
            output='screen'
        )]
    
    # Auto-save map after a delay (wait for node to be fully activated)
    # Wait 3 seconds for lifecycle manager to activate the node
    auto_save_action = TimerAction(
        period=3.0,
        actions=[
            OpaqueFunction(function=generate_save_map_service_call)
        ]
    )
    
    # Note: The map will be automatically saved 3 seconds after launch.
    # 
    # Important: map_url must be a FILE PATH (without extension), not a directory!
    # map_saver will automatically create <map_url>.pgm and <map_url>.yaml
    # 
    # Example manual service call:
    # ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
    #   "{map_url: '/path/to/maps/my_map', image_format: 'pgm', map_mode: 'trinary', free_thresh: 0.19, occupied_thresh: 0.65}"
    # This will create: /path/to/maps/my_map.pgm and /path/to/maps/my_map.yaml
    # 
    # For cartographer: free_thresh: 0.49, occupied_thresh: 0.51
    
    return LaunchDescription([
        map_file_arg,
        map_path_arg,
        slam_methods_arg,
        map_saver_server_node,
        lifecycle_manager,
        auto_save_action,
    ])
