"""
动态障碍物过滤器启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('scan_filter').find('scan_filter')
    default_params = PathJoinSubstitution([pkg_share, 'config', 'filter_params.yaml'])

    # 参数声明
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the filter parameters file'
    )

    enable_filter_arg = DeclareLaunchArgument(
        'enable_filter',
        default_value='true',
        description='Enable or disable the filter'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug output'
    )

    # 过滤器节点
    filter_node = Node(
        package='scan_filter',
        executable='dynamic_obstacle_filter.py',
        name='dynamic_obstacle_filter',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'enable_filter': LaunchConfiguration('enable_filter'),
                'debug': LaunchConfiguration('debug'),
            }
        ]
    )

    return LaunchDescription([
        params_file_arg,
        enable_filter_arg,
        debug_arg,
        filter_node,
    ])
