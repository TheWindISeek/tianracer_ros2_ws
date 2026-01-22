from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os
from datetime import datetime


def generate_launch_description():
    # Default map save path
    default_map_path = os.path.join(os.path.expanduser('~'), 'tianracer_ros2_ws', 'maps')
    os.makedirs(default_map_path, exist_ok=True)

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='rtabmap_map',
        description='Map file name (without extension)'
    )

    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=default_map_path,
        description='Path to save map'
    )

    def save_rtabmap_map(context):
        map_name = context.launch_configurations.get('map_name', 'rtabmap_map')
        map_path = context.launch_configurations.get('map_path', default_map_path)

        # Add timestamp to avoid overwriting
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        full_path = os.path.join(map_path, f'{map_name}_{timestamp}')

        # RTAB-Map saves its database automatically to ~/.ros/rtabmap.db
        # We need to export the 2D grid map for nav2

        # Save occupancy grid map using nav2 map_saver
        save_map = ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', full_path,
                 '--free', '0.19',
                 '--occ', '0.65'],
            output='screen'
        )

        return [save_map]

    save_action = OpaqueFunction(function=save_rtabmap_map)

    return LaunchDescription([
        map_name_arg,
        map_path_arg,
        save_action,
    ])
