from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Nav2 参数文件路径（可选，使用默认配置如果为空）'
    )
    
    # Map Server 节点（生命周期节点）
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map_file'),
        }]
    )
    
    # AMCL 定位节点（生命周期节点）
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # AMCL 基本参数
            'min_particles': 500,
            'max_particles': 2000,
            'kld_err': 0.05,
            'kld_z': 0.99,
            'update_min_d': 0.2,
            'update_min_a': 0.5,
            'resample_interval': 1,
            'transform_tolerance': 1.0,
            'recovery_alpha_slow': 0.0,
            'recovery_alpha_fast': 0.0,
            'initial_pose_x': 0.0,
            'initial_pose_y': 0.0,
            'initial_pose_a': 0.0,
            'initial_cov_xx': 0.5,
            'initial_cov_yy': 0.5,
            'initial_cov_aa': 0.5,
            # 激光参数
            'laser_min_range': -1.0,
            'laser_max_range': -1.0,
            'laser_max_beams': 60,
            'laser_z_hit': 0.95,
            'laser_z_short': 0.1,
            'laser_z_max': 0.05,
            'laser_z_rand': 0.05,
            'laser_sigma_hit': 0.2,
            'laser_lambda_short': 0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_model_type': 'likelihood_field',
            # 里程计参数
            'odom_model_type': 'diff',
            'odom_alpha1': 0.2,
            'odom_alpha2': 0.2,
            'odom_alpha3': 0.8,
            'odom_alpha4': 0.2,
            'odom_alpha5': 0.1,
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint',
            'global_frame_id': 'map',
            'scan_topic': '/scan',
        }]
    )
    
    # Nav2 控制器节点
    controller_node = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'controller_frequency': 20.0,
            'min_x_velocity_threshold': 0.001,
            'min_y_velocity_threshold': 0.5,
            'min_theta_velocity_threshold': 0.001,
            'failure_tolerance': 0.3,
            'progress_checker_plugin': 'progress_checker',
            'goal_checker_plugins': ['general_goal_checker'],
            'controller_plugins': ['FollowPath'],
            'FollowPath': {
                'plugin': 'nav2_controller::FollowPath',
                'desired_linear_vel': 0.5,
                'base_desired_linear_vel': 0.5,
                'desired_angular_vel': 1.0,
                'base_desired_angular_vel': 1.0,
                'max_linear_acc': 2.5,
                'max_linear_decel': 2.5,
                'max_angular_acc': 3.2,
                'inflation_radius': 0.55,
                'cost_scaling_factor': 5.0,
                'transform_tolerance': 0.1,
                'use_rotate_to_heading': True,
                'min_rotate_to_heading_vel': 0.4,
                'allow_backward': True,
                'max_allowed_time_to_collision_up_to_carrot': 1.0,
                'use_regulated_linear_velocity_scaling': True,
                'use_constrained_linear_velocity_scaling': False,
                'use_cost_based_linear_velocity_scaling': True,
                'regulated_linear_scaling_min_radius': 0.9,
                'regulated_linear_scaling_min_speed': 0.25,
                'cost_scaling_gain': 1.0,
                'inflation_cost_scaling_factor': 3.0,
                'regulated_angular_scaling_min_radius': 0.9,
                'lookahead_dist': 0.6,
                'use_approach_velocity_scaling': True,
                'approach_velocity_scaling_dist': 0.6,
                'max_approach_linear_velocity': 0.3,
                'control_duration': 0.05,
                'use_collision_detection': True,
            },
            'general_goal_checker': {
                'plugin': 'nav2_controller::SimpleGoalChecker',
                'stateful': True,
                'xy_goal_tolerance': 0.25,
                'yaw_goal_tolerance': 0.25,
            },
            'progress_checker': {
                'plugin': 'nav2_controller::SimpleProgressChecker',
                'required_movement_radius': 0.5,
                'movement_time_allowance': 10.0,
            },
        }]
    )
    
    # Nav2 规划器节点
    planner_node = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'expected_planner_frequency': 20.0,
            'planner_plugins': ['GridBased'],
            'GridBased': {
                'plugin': 'nav2_navfn_planner/NavfnPlanner',
                'tolerance': 0.5,
                'use_astar': False,
                'allow_unknown': True,
            },
        }]
    )
    
    # Nav2 恢复行为节点
    recovery_node = LifecycleNode(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'costmap_topic': 'local_costmap/costmap_raw',
            'footprint_topic': 'local_costmap/published_footprint',
            'cycle_frequency': 10.0,
            'recovery_plugins': ['spin', 'backup', 'wait'],
            'spin': {
                'plugin': 'nav2_recoveries/Spin',
                'simulate_ahead_time': 2.0,
                'max_rotational_vel': 1.0,
                'min_rotational_vel': 0.4,
                'rotational_acc_lim': 3.2,
            },
            'backup': {
                'plugin': 'nav2_recoveries/BackUp',
                'simulate_ahead_time': 2.0,
                'max_speed': -0.025,
                'min_speed': -0.025,
                'acc_lim': 0.25,
            },
            'wait': {
                'plugin': 'nav2_recoveries/Wait',
                'wait_duration': 5.0,
            },
        }]
    )
    
    # Nav2 行为树节点
    bt_navigator_node = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'global_frame': 'map',
            'robot_base_frame': 'base_footprint',
            'odom_topic': '/odom',
            'bt_loop_duration': 10,
            'default_server_timeout': 20,
            'enable_groot_monitoring': True,
            'groot_zmq_publisher_port': 1666,
            'groot_zmq_server_port': 1667,
        }]
    )
    
    # Nav2 路径规划器节点（Waypoint Follower）
    waypoint_follower_node = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'loop_rate': 20,
            'stop_on_failure': False,
            'required_movement_radius': 0.5,
        }]
    )
    
    # Nav2 速度平滑器节点
    velocity_smoother_node = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'smoothing_frequency': 20.0,
            'scale_velocities': False,
            'feedback': 'OPEN_LOOP',
            'odom_topic': '/odom',
            'odom_duration': 0.1,
            'deadband_velocity': [0.0, 0.0, 0.0],
            'velocity_timeout': 1.0,
            'max_velocity': [0.5, 0.0, 1.0],
            'min_velocity': [-0.5, 0.0, -1.0],
            'max_accel': [2.5, 0.0, 3.2],
            'max_decel': [-2.5, 0.0, -3.2],
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_footprint',
            'odom_duration': 0.1,
        }]
    )
    
    # Lifecycle Manager 用于管理所有 Nav2 节点
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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

