from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('tianracer_navigation_ros2').find('tianracer_navigation_ros2')
    
    # 获取slam包路径（地图文件在slam包中）
    slam_pkg_share = FindPackageShare('tianracer_slam_ros2').find('tianracer_slam_ros2')
    
    # 默认地图路径（从install目录读取）
    default_map_path = PathJoinSubstitution([
        slam_pkg_share, 'maps', 'my_map.yaml'
    ])
    
    # 默认参数文件路径
    default_params_file = PathJoinSubstitution([
        pkg_share, 'config', 'nav2_params.yaml'
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
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Nav2 参数文件路径'
    )
    
    # 读取参数文件
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    
    # Map Server 节点（生命周期节点）
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file,
        }]
    )
    
    # AMCL 定位节点（生命周期节点）
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Nav2 控制器节点
    controller_node = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Nav2 规划器节点
    planner_node = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Nav2 恢复行为节点
    recovery_node = LifecycleNode(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Nav2 行为树节点（BT Navigator）
    bt_navigator_node = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Nav2 路径规划器节点（Waypoint Follower）
    waypoint_follower_node = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Nav2 速度平滑器节点
    velocity_smoother_node = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Lifecycle Manager 用于管理所有 Nav2 节点
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'recoveries_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ]
        }]
    )
    
    return LaunchDescription([
        map_file_arg,
        use_sim_time_arg,
        params_file_arg,
        map_server_node,
        amcl_node,
        controller_node,
        planner_node,
        recovery_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager,
    ])
