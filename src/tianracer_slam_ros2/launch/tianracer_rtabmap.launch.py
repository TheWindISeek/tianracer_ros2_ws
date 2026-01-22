from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Whether to use simulation time'
    )

    # RTAB-Map node for 2D SLAM (using only lidar)
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': True,
            'approx_sync': True,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            # SLAM parameters
            'RGBD/ProximityBySpace': 'false',
            'Reg/Strategy': '1',  # 1=ICP (for lidar)
            'Icp/CorrespondenceRatio': '0.2',
            'Icp/MaxCorrespondenceDistance': '0.3',
            'RGBD/NeighborLinkRefining': 'true',
            'Grid/FromDepth': 'false',  # Use scan for grid
            'Grid/RangeMax': '20.0',
            'Grid/CellSize': '0.05',
            # Memory management
            'Mem/STMSize': '30',
            'Mem/RehearsalSimilarity': '0.45',
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odometry/filtered'),
        ],
        arguments=['--delete_db_on_start']
    )

    return LaunchDescription([
        use_sim_time_arg,
        rtabmap_node,
    ])
