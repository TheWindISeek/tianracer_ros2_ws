"""
================================================================================
Tianracer Nav2 仿真导航启动脚本
================================================================================

使用方法:
    # 1. 先启动 Gazebo 仿真（终端1）
    ros2 launch tianbot_description_ros2 gazebo_robot.launch.py x:=0.5 y:=0.5 world:=/home/lucifer/tianracer_ros2_ws/world/lab.sdf

    # 2. 再启动导航（终端2），初始位姿要和 Gazebo 一致
    ros2 launch tianracer_navigation_ros2 navigation_sim.launch.py

    # 自定义初始位置（需要和 Gazebo 启动参数一致）
    ros2 launch tianracer_navigation_ros2 navigation_sim.launch.py initial_x:=1.0 initial_y:=2.0 initial_yaw:=-0.5

================================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('tianracer_navigation_ros2').find('tianracer_navigation_ros2')

    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'nav2_params_real.yaml'])
    default_map_path = '/home/lucifer/tianracer_ros2_ws/maps/newfull_map.yaml'

    # Launch 参数声明
    map_file_arg = DeclareLaunchArgument('map_file', default_value=default_map_path)
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params_file)
    # 默认值和 gazebo_robot.launch.py 保持一致
    initial_x_arg = DeclareLaunchArgument('initial_x', default_value='1')
    initial_y_arg = DeclareLaunchArgument('initial_y', default_value='1.7')
    initial_yaw_arg = DeclareLaunchArgument('initial_yaw', default_value='3.138')  # 和 Gazebo 默认 yaw 一致

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')

    # 0s: Map Server
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_file}]
    )

    # 3s: AMCL
    amcl_node = TimerAction(
        period=3.0,
        actions=[
            LifecycleNode(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace='',
                output='screen',
                parameters=[params_file, {
                    'initial_pose.x': PythonExpression(['float(', initial_x, ')']),
                    'initial_pose.y': PythonExpression(['float(', initial_y, ')']),
                    'initial_pose.yaw': PythonExpression(['float(', initial_yaw, ')']),
                    'set_initial_pose': True,
                }]
            )
        ]
    )

    # 6s: Nav2 核心节点
    controller_node = TimerAction(
        period=6.0,
        actions=[
            LifecycleNode(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                namespace='',
                output='screen',
                parameters=[params_file]
            )
        ]
    )

    planner_node = TimerAction(
        period=6.0,
        actions=[
            LifecycleNode(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace='',
                output='screen',
                parameters=[params_file]
            )
        ]
    )

    behavior_node = TimerAction(
        period=6.0,
        actions=[
            LifecycleNode(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                namespace='',
                output='screen',
                parameters=[params_file]
            )
        ]
    )

    bt_navigator_node = TimerAction(
        period=6.0,
        actions=[
            LifecycleNode(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace='',
                output='screen',
                parameters=[params_file]
            )
        ]
    )

    # 静态TF发布器：base_footprint -> base_link (根据URDF定义，z偏移为轮子半径0.065m)
    static_tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint_to_base_link',
        arguments=['--x', '0', '--y', '0', '--z', '0.065', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
        parameters=[{'use_sim_time': True}]
    )

    # 9s: 静态TF发布器 (在AMCL之后启动)
    static_tf_publisher = TimerAction(
        period=9.0,
        actions=[static_tf_base_footprint_to_base_link]
    )

    # 10s: Lifecycle Manager - 等待所有节点完全启动后再激活
    lifecycle_manager = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'bond_timeout': 10.0,
                    'node_names': [
                        'map_server',
                        'amcl',
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                    ]
                }]
            )
        ]
    )

    return LaunchDescription([
        map_file_arg,
        params_file_arg,
        initial_x_arg,
        initial_y_arg,
        initial_yaw_arg,
        map_server_node,
        amcl_node,
        controller_node,
        planner_node,
        behavior_node,
        bt_navigator_node,
        static_tf_publisher,
        lifecycle_manager,
    ])
