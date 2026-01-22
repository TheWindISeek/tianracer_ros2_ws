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
        default_value='cartographer_map',
        description='Map file name (without extension)'
    )

    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=default_map_path,
        description='Path to save map'
    )

    def save_cartographer_map(context):
        map_name = context.launch_configurations.get('map_name', 'cartographer_map')
        map_path = context.launch_configurations.get('map_path', default_map_path)

        # Add timestamp to avoid overwriting
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        full_path = os.path.join(map_path, f'{map_name}_{timestamp}')

        # Cartographer uses pbstream format, then convert to occupancy grid
        pbstream_path = f'{full_path}.pbstream'

        # Step 1: Finish trajectory and save pbstream
        finish_trajectory = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/finish_trajectory',
                 'cartographer_ros_msgs/srv/FinishTrajectory',
                 '{trajectory_id: 0}'],
            output='screen'
        )

        # Step 2: Write state to pbstream file
        write_state = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/write_state',
                 'cartographer_ros_msgs/srv/WriteState',
                 f'{{filename: "{pbstream_path}"}}'],
            output='screen'
        )

        # Step 3: Save occupancy grid map using nav2 map_saver
        save_map = ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', full_path,
                 '--free', '0.49',
                 '--occ', '0.51'],
            output='screen'
        )

        return [finish_trajectory, write_state, save_map]

    save_action = OpaqueFunction(function=save_cartographer_map)

    return LaunchDescription([
        map_name_arg,
        map_path_arg,
        save_action,
    ])
