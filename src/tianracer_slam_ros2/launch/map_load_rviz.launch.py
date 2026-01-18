from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('tianracer_slam_ros2').find('tianracer_slam_ros2')
    
    # 默认地图路径（从install目录读取）
    default_map_path = PathJoinSubstitution([
        pkg_share, 'maps', 'my_map.yaml'
    ])
    
    # 声明启动参数
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_path,
        description='地图文件路径（.yaml 文件）'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动 RViz2'
    )
    
    # Map Server 节点（生命周期节点）
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map_file'),
        }]
    )
    
    # Lifecycle Manager 用于自动激活 map_server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # 可选：发布 map -> base_footprint 变换（用于 RViz2 显示）
    static_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint'],
        output='screen'
    )
    
    # RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    
    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        use_rviz_arg,
        map_server_node,
        lifecycle_manager,
        static_tf_base,
        rviz_node,
    ])

