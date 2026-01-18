# src/ros2_warehouse_amr/launch/robot_factory.launch.py
"""
机器人工厂：支持多机器人载入
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
import os
import yaml
import tempfile

def generate_nav2_params_file(robot_name, namespace, pkg_share):
    """为每个机器人动态生成 Nav2 参数文件"""
    # 读取原始参数文件
    nav2_params_path = os.path.join(pkg_share, 'navigation', 'params', 'nav2_params.yaml')
    with open(nav2_params_path, 'r') as f:
        params = yaml.safe_load(f)
    
    # 修改 local_costmap 参数
    if 'local_costmap' in params and 'local_costmap' in params['local_costmap']:
        lc = params['local_costmap']['local_costmap']
        if 'ros__parameters' in lc:
            lc['ros__parameters']['global_frame'] = f'{robot_name}/odom'
            lc['ros__parameters']['robot_base_frame'] = f'{robot_name}/robot_footprint'
            # 修改 voxel_layer 的 scan 话题
            if 'voxel_layer' in lc['ros__parameters']:
                if 'scan' in lc['ros__parameters']['voxel_layer']:
                    lc['ros__parameters']['voxel_layer']['scan']['topic'] = f'{namespace}/scan'
    
    # 修改 global_costmap 参数
    if 'global_costmap' in params and 'global_costmap' in params['global_costmap']:
        gc = params['global_costmap']['global_costmap']
        if 'ros__parameters' in gc:
            gc['ros__parameters']['global_frame'] = 'map'
            gc['ros__parameters']['robot_base_frame'] = f'{robot_name}/robot_footprint'
            # 修改 obstacle_layer 的 scan 话题
            if 'obstacle_layer' in gc['ros__parameters']:
                if 'scan' in gc['ros__parameters']['obstacle_layer']:
                    gc['ros__parameters']['obstacle_layer']['scan']['topic'] = f'{namespace}/scan'
            # 修改 static_layer 的 map_topic，确保订阅全局 /map 话题
            if 'static_layer' in gc['ros__parameters']:
                gc['ros__parameters']['static_layer']['map_topic'] = '/map'
            else:
                gc['ros__parameters']['static_layer'] = {'map_topic': '/map'}
    
    # 修改 bt_navigator 参数
    if 'bt_navigator' in params:
        if 'ros__parameters' in params['bt_navigator']:
            params['bt_navigator']['ros__parameters']['global_frame'] = 'map'
            params['bt_navigator']['ros__parameters']['robot_base_frame'] = f'{robot_name}/robot_footprint'
            params['bt_navigator']['ros__parameters']['odom_topic'] = f'{namespace}/odom'
    
    # 修改 behavior_server 参数
    if 'behavior_server' in params:
        if 'ros__parameters' in params['behavior_server']:
            params['behavior_server']['ros__parameters']['global_frame'] = f'{robot_name}/odom'
            params['behavior_server']['ros__parameters']['robot_base_frame'] = f'{robot_name}/robot_footprint'
    
    # 创建临时文件保存修改后的参数
    temp_dir = tempfile.gettempdir()
    temp_file = os.path.join(temp_dir, f'nav2_params_{robot_name}.yaml')
    with open(temp_file, 'w') as f:
        yaml.dump(params, f, default_flow_style=False)
    
    return temp_file


def generate_robot_nav2_nodes(robot_name, namespace, pkg_share, use_sim_time,
                              initial_x, initial_y, initial_yaw):
    """生成单个机器人的Nav2节点"""
    # nav2参数文件
    amcl_params_file = PathJoinSubstitution([
        pkg_share, 'navigation', 'params', 'amcl_params.yaml'
    ])

    # AMCL 参数 - RewrittenYaml 对顶层参数有效
    amcl_params = RewrittenYaml(
        source_file=amcl_params_file,
        root_key=namespace,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'initial_x': str(initial_x),
            'initial_y': str(initial_y),
            'initial_yaw': str(initial_yaw),
            'odom_frame_id': f'{robot_name}/odom',
            'base_frame_id': f'{robot_name}/robot_footprint',
            'global_frame_id': 'map',
            'scan_topic': f'{namespace}/scan',
            'map_topic': '/map',
            'use_map_topic': 'True',
        },
        convert_types=True
    )

    # ========================================================================
    # 动态生成 Nav2 参数文件
    # ========================================================================
    # 由于 RewrittenYaml 对嵌套路径无效，且嵌套字典也无法传递给子节点，
    # 我们需要在 launch 时动态生成一个包含正确参数的 YAML 文件
    nav2_params_file = generate_nav2_params_file(robot_name, namespace, pkg_share)
    
    # 使用动态生成的参数文件（通过 RewrittenYaml 添加命名空间）
    nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key=namespace,
        param_rewrites={
            'use_sim_time': use_sim_time,
        },
        convert_types=True
    )

    # ========================================================================
    # 启动节点
    # ========================================================================
    
    # AMCL 节点
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=namespace,
        output='screen',
        parameters=[amcl_params],
    )

    # Nav2 Controller Server
    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=namespace,
        output='screen',
        parameters=[nav2_params],
    )
    
    # Nav2 Planner Server
    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=namespace,
        output='screen',
        parameters=[nav2_params],
    )
    
    # Nav2 Smoother Server
    smoother_server = LifecycleNode(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        namespace=namespace,
        output='screen',
        parameters=[nav2_params],
    )
    
    # Nav2 Behavior Server
    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=namespace,
        output='screen',
        parameters=[nav2_params],
    )
    
    # Nav2 BT Navigator
    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=namespace,
        output='screen',
        parameters=[nav2_params],
    )
    
    # Nav2 生命周期管理器（每个机器人一个）
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {
                'node_names': [
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'smoother_server',
                    'behavior_server',
                    'bt_navigator',
                ]
            },
            {'autostart': True}
        ]
    )
    
    return [
        # AMCL 延迟启动（等待地图服务器激活）
        # 地图服务器在0秒启动，lifecycle_manager_map在0秒启动并激活地图服务器
        # 所以AMCL应该在2秒后启动，确保地图已经发布
        TimerAction(
            period=2.0,  # 延迟 2 秒，等待地图服务器激活并发布地图
            actions=[amcl_node]
        ),
        # Nav2 其他节点延迟启动（等待 AMCL）
        TimerAction(
            period=4.0,  # 延迟 4 秒，等待 AMCL 启动
            actions=[
                controller_server,
                planner_server,
                smoother_server,
                behavior_server,
                bt_navigator,
            ]
        ),
        # 生命周期管理器启动（激活所有节点）
        # 延迟时间要确保地图服务器已经激活，AMCL节点已经启动
        TimerAction(
            period=5.0,  # 延迟 5 秒，等待地图服务器激活和AMCL节点启动
            actions=[nav2_lifecycle_manager]
        ),
    ] 


def generate_robot_group(robot_id, x, y, yaw, material, use_sim_time):
    """生成单个机器人的完整配置组"""
    robot_name = f"robot{robot_id}"
    namespace = f"/{robot_name}"
    
    # 获取包路径
    pkg_share = get_package_share_directory('ros2_warehouse_amr')
    
    # URDF文件路径
    urdf_file = PathJoinSubstitution([
        pkg_share, 'models', 'urdf', 'delivery_robot.xacro'
    ])
    
    # 处理URDF（可以传入robot_name参数给xacro）
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_file,
        ' robot_name:=', robot_name,  # 传递机器人名称给xacro
        ' namespace:=', namespace,     # 传递命名空间给xacro
        ' material:=', material     # 传递材质给xacro
    ])
    
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    
    # Robot State Publisher（在命名空间下）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
            {'frame_prefix': f'{robot_name}/'}  # TF前缀
        ],
        output='screen'
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 在Gazebo中生成机器人实体
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', robot_name,  # 实体名称必须唯一
            '-topic', f'{namespace}/robot_description',  # 使用命名空间下的话题
            '-x', str(x),
            '-y', str(y),
            '-z', '0.13',
            '-Y', str(yaw)
        ],
        output='screen'
    )
    
    # Nav2节点（每个机器人需要独立的Nav2实例）
    nav2_nodes = generate_robot_nav2_nodes(robot_name, namespace, pkg_share, use_sim_time,
                                           x, y, yaw)

    # 设置初始位置（延迟启动，等待AMCL被激活）
    # 时间线：
    # T=0s: 地图服务器启动，lifecycle_manager_map启动并激活地图服务器
    # T=2s: AMCL节点启动
    # T=5s: lifecycle_manager启动并激活AMCL（以及其他Nav2节点）
    # T=10s: initial_pose_setter启动并发布初始位姿（确保AMCL已经完全激活）
    # T=12s: 行为树执行器启动（确保Nav2完全就绪）
    # 注意：多机器人系统中，激活过程可能需要更长时间
    initial_pose_setter_node = TimerAction(
        period=10.0,  # 延迟10秒，确保AMCL已经被lifecycle_manager完全激活
        actions=[
            Node(
                package='ros2_warehouse_amr',
                executable='initial_pose_setter_node',
                name='initial_pose_setter_node',
                namespace=namespace,  # 显式设置命名空间，确保发布到正确的话题
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'initial_x': x},
                    {'initial_y': y},
                    {'initial_yaw': yaw},
                    {'wait_time': 2.0},  # 节点启动后等待2秒再发布（给AMCL更多时间）
                ],
            )
        ]
    )
    
    # 行为树执行器节点（延迟启动，确保Nav2完全就绪）
    # 行为树执行器需要：
    # 1. Nav2 导航服务可用（navigate_to_pose action）
    # 2. TF 树正确建立（map -> robot_footprint）
    # 3. 货物控制器可用
    behavior_tree_executor_node = TimerAction(
        period=12.0,  # 延迟12秒，确保Nav2完全启动并可用
        actions=[
            Node(
                package='ros2_warehouse_amr',
                executable='robot_behavior_tree_executor_node',
                name='robot_behavior_tree_executor',
                namespace=namespace,  # 使用机器人命名空间
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    # 机器人初始位置（用于任务完成后返回）
                    {'initial_x': x},
                    {'initial_y': y},
                    {'initial_yaw': yaw},
                    # 可选：自定义参数
                    # {'loading_duration': 3.0},
                    # {'unloading_duration': 2.0},
                    # {'cargo_spawn_height': 0.3},
                ],
            )
        ]
    )
    
    # 使用GroupAction将所有节点组织在同一个命名空间下
    return GroupAction([
        PushRosNamespace(namespace),
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        *nav2_nodes,
        initial_pose_setter_node,
        behavior_tree_executor_node,  # 添加行为树执行器节点
    ])

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 获取包路径
    pkg_share = get_package_share_directory('ros2_warehouse_amr')
    
    # 地图服务器创建
    map_file = os.path.join(pkg_share, "maps", "warehouse_slam_map.yaml")

    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time}
        ],
    )

    map_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {'use_sim_time': use_sim_time},
            {'node_names': ['map_server']},
            {'autostart': True}
        ],
    )


    # 读取机器人配置文件
    config_file = os.path.join(pkg_share, 'configs', 'robot_factory.yaml')
    with open(config_file, 'r', encoding='utf-8') as f:
        config_data = yaml.safe_load(f)
    
    # 从配置文件中获取机器人配置列表
    robot_configs = config_data.get('robots', [])

    # 为每个机器人生成配置组
    robot_groups = []
    for config in robot_configs:
        robot_groups.append(
            generate_robot_group(
                config['id'],
                config['x'],
                config['y'],
                config['yaw'],
                config['material'],
                use_sim_time
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time'
        ),
        # 启动全局地图
        map_server_node,
        map_lifecycle_manager,
        # 启动所有机器人
        *robot_groups  # 展开所有机器人组
    ])