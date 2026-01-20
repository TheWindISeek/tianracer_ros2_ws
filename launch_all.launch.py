"""
================================================================================
ROS2 Warehouse AMR - 世界与机器人启动脚本
================================================================================

功能说明:
    这是一个综合性的 launch 脚本，用于启动 Gazebo 仿真环境和机器人模型。
    参考 TurtleBot3 的启动方式设计，具有良好的扩展性。

使用方法:
    # 默认启动（warehouse 世界 + delivery_robot）
    ros2 launch ros2_warehouse_amr launch_all.launch.py

    # 自定义世界文件
    ros2 launch ros2_warehouse_amr launch_all.launch.py world:=/path/to/your.world

    # 使用空世界进行测试
    ros2 launch ros2_warehouse_amr launch_all.launch.py world:=/usr/share/gazebo-11/worlds/empty.world

    # 自定义机器人初始位置
    ros2 launch ros2_warehouse_amr launch_all.launch.py x:=1.0 y:=2.0 yaw:=1.57

    # 无头模式（不启动 GUI，用于 CI/CD 或远程服务器）
    ros2 launch ros2_warehouse_amr launch_all.launch.py headless:=True gui:=False

    # 自定义机器人模型
    ros2 launch ros2_warehouse_amr launch_all.launch.py robot_model:=custom_robot.xacro

    # 同时启动 RViz2 可视化（推荐）
    ros2 launch ros2_warehouse_amr launch_all.launch.py use_rviz:=True

作者: lucifer
创建日期: 2025.12.8
================================================================================
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():
    """
    生成 Launch 描述。

    整体流程:
    1. 获取包路径和配置参数
    2. 设置 Gazebo 环境变量（关键步骤！）
    3. 启动 Gazebo 仿真器（gzserver + gzclient）
    4. 启动机器人状态发布节点
    5. 在仿真中生成机器人实体
    """

    # ============================================================================
    # 第一部分：包路径和默认配置
    # ============================================================================

    # 获取本包的 share 目录（安装后的路径）
    # 注意：这里获取的是 install/ros2_warehouse_amr/share/ros2_warehouse_amr/
    # 而不是源码目录 src/ros2_warehouse_amr/
    pkg_share = get_package_share_directory('ros2_warehouse_amr')

    # ============================================================================
    # 第二部分：Launch 参数声明
    # ============================================================================
    # 这些参数可以在命令行通过 param:=value 的方式覆盖

    # --- 仿真控制参数 ---
    use_sim_time = LaunchConfiguration('use_sim_time')  # 是否使用仿真时间
    gui = LaunchConfiguration('gui')                     # 是否启动 Gazebo GUI
    headless = LaunchConfiguration('headless')           # 无头模式（无渲染）
    use_rviz = LaunchConfiguration('use_rviz')          # 是否启动 RViz2
    use_composition = LaunchConfiguration('use_composition')  # 是否使用组件方式加载

    # --- 世界和机器人选择 ---
    world = LaunchConfiguration('world')                 # 世界文件路径
    robot_model = LaunchConfiguration('robot_model')     # 机器人模型文件名

    # --- 机器人初始位姿 ---
    x = LaunchConfiguration('x')                         # X 坐标
    y = LaunchConfiguration('y')                         # Y 坐标
    z = LaunchConfiguration('z')                         # Z 坐标（高度）
    roll = LaunchConfiguration('roll')                   # 横滚角
    pitch = LaunchConfiguration('pitch')                 # 俯仰角
    yaw = LaunchConfiguration('yaw')                     # 偏航角

    # --- 默认文件路径 ---
    world_default = os.path.join(pkg_share, 'worlds', 'warehouse.world')

    # 使用 PathJoinSubstitution 支持运行时替换机器人模型
    urdf_file = PathJoinSubstitution([pkg_share, 'models', 'urdf', robot_model])

    # ============================================================================
    # 第三部分：环境变量设置（非常重要！）
    # ============================================================================
    """
    为什么需要手动设置这些环境变量？

    问题背景:
    Gazebo 需要知道在哪里查找模型、插件和资源文件。这些路径通过环境变量指定：
    - GAZEBO_MODEL_PATH: 模型文件搜索路径
    - GAZEBO_PLUGIN_PATH: 插件库搜索路径
    - GAZEBO_RESOURCE_PATH: 资源文件搜索路径

    为什么没有被自动设置？
    1. ROS2 的设计理念是"松耦合"，不自动污染用户环境变量
    2. 不同项目可能需要不同的模型路径，统一设置会造成冲突
    3. 安装位置可能因系统而异（/opt/ros vs 本地 workspace）
    4. 用户可能有多个 Gazebo 版本或自定义模型库

    TurtleBot3 的做法:
    TurtleBot3 也需要在 launch 文件中设置这些变量，参见：
    /opt/ros/humble/share/turtlebot3_gazebo/launch/turtlebot3_world.launch.py

    使用 AppendEnvironmentVariable 而非 SetEnvironmentVariable 的原因:
    - Append 会保留现有值并追加新路径
    - Set 会覆盖现有值，可能导致系统默认模型找不到

    路径说明:
    - /usr/share/gazebo-11/models: Gazebo 自带模型（ground_plane, sun 等）
    - pkg_share/models: 我们自定义的机器人和环境模型
    - /opt/ros/humble/lib: ROS2 Gazebo 插件（libgazebo_ros_*.so）
    """

    # 设置 Gazebo 模型搜索路径
    set_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        ':'.join([
            '/usr/share/gazebo-11/models',      # Gazebo 默认模型
            os.path.join(pkg_share, 'models'),  # 我们的自定义模型
        ])
    )

    # 设置 Gazebo 插件搜索路径
    # 插件是 Gazebo 与 ROS2 通信的桥梁，如：
    # - libgazebo_ros_init.so: 初始化 ROS2 节点
    # - libgazebo_ros_factory.so: 支持动态生成模型
    # - libgazebo_ros_diff_drive.so: 差速驱动控制器
    # - libgazebo_ros_ray_sensor.so: 激光雷达传感器
    set_plugin_path = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        ':'.join([
            '/opt/ros/humble/lib',  # ROS2 Gazebo 插件目录
        ])
    )

    # 设置 Gazebo 资源搜索路径（材质、纹理、媒体文件等）
    set_resource_path = AppendEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        ':'.join([
            '/usr/share/gazebo-11',             # Gazebo 默认资源
            os.path.join(pkg_share, 'worlds'),  # 我们的世界文件目录
        ])
    )

    # ============================================================================
    # 第四部分：Gazebo 仿真器启动
    # ============================================================================
    """
    Gazebo 采用客户端-服务器架构:
    - gzserver: 物理仿真引擎，负责所有计算（可在无 GUI 环境运行）
    - gzclient: 3D 可视化界面，负责渲染（需要显示器或虚拟显示）

    关键参数:
    - -s libgazebo_ros_init.so: 加载 ROS2 初始化插件
    - -s libgazebo_ros_factory.so: 加载模型工厂插件（支持 spawn_entity）

    为什么分开启动？
    - 在远程服务器或 CI/CD 中，可以只运行 gzserver（headless 模式）
    - gzclient 消耗大量 GPU 资源，不是必须的
    """

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',     # ROS2 通信支持
            '-s', 'libgazebo_ros_factory.so',  # 模型生成支持
            world                               # 世界文件
        ],
        cwd=[pkg_share],
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[pkg_share],
        output='screen',
        # 条件：gui=True 且 headless=False 时才启动
        condition=IfCondition(PythonExpression([gui, ' and not ', headless]))
    )

    # ============================================================================
    # 第五部分：机器人描述处理
    # ============================================================================
    """
    机器人模型加载流程:

    1. xacro 处理:
       .xacro 文件是 XML 宏语言，支持参数化和模块化
       xacro 命令将其展开为标准 URDF XML

    2. robot_state_publisher:
       - 读取 URDF，构建机器人运动学模型
       - 发布 /robot_description 话题
       - 根据 /joint_states 计算并发布 TF 变换

    3. joint_state_publisher:
       - 发布 /joint_states 话题
       - 在仿真中，Gazebo 插件会覆盖此发布

    4. spawn_entity.py:
       - 从 /robot_description 话题读取 URDF
       - 调用 Gazebo 服务在仿真中生成实体
    """

    # 执行 xacro 命令，将 .xacro 转换为 URDF 字符串
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_file
    ])

    # print(robot_description_content)

    # 将 URDF 内容包装为参数
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # 机器人状态发布节点
    # 功能：发布机器人各部件之间的 TF 变换
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ],
    )

    # 关节状态发布节点
    # 注意：在 Gazebo 仿真中，diff_drive 插件会发布轮子的关节状态
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ============================================================================
    # 第六部分：机器人生成
    # ============================================================================

    # 使用 TimerAction 延迟生成，确保 Gazebo 完全启动
    # 如果 Gazebo 还没准备好就尝试生成，会失败
    spawn_entity = TimerAction(
        period=3.0,  # 等待 3 秒
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='urdf_spawner',
                output='screen',
                arguments=[
                    '-entity', 'delivery_robot',  # 实体名称（Gazebo 中显示的名字）
                    '-topic', 'robot_description', # 从此话题获取 URDF
                    '-x', x, '-y', y, '-z', z,     # 位置
                    '-R', roll, '-P', pitch, '-Y', yaw,  # 姿态
                ]
            )
        ]
    )

    # ============================================================================
    # 第六部分（续）：仓库对象生成（Storage 和 Dispatch）
    # ============================================================================
    """
    仓库对象生成节点说明：
    
    功能：
    - 从配置文件（configs/storages 和 configs/dispatches）读取位置信息
    - 在 Gazebo 中生成 Storage（货物柜）和 Dispatch（分发点）模型
    - 这些模型会在机器人启动后自动加载到地图中
    
    启动时机：
    - 在机器人生成后启动（延迟 4 秒）
    - 节点内部还会再等待 5 秒确保 Gazebo 服务可用
    - 总共约 9 秒后开始生成仓库对象
    
    注意：
    - 这些是静态模型，不会影响 SLAM 建图
    - 但如果这些模型会影响激光雷达扫描，可能需要重新建图
    """
    
    warehouse_spawner_node = TimerAction(
        period=4.0,  # 在机器人生成后 1 秒启动（机器人生成延迟 3 秒）
        actions=[
            Node(
                package='ros2_warehouse_amr',
                executable='warehouse_spawner_node',
                name='warehouse_spawner',
                output='screen',
            )
        ]
    )

    # ============================================================================
    # 第七部分：地图服务器和定位系统
    # ============================================================================
    """
    地图服务器和定位系统说明：
    
    【1. 地图服务器 (map_server)】
    
    功能：
    - 读取地图文件（YAML + PGM）
    - 发布 /map 话题（nav_msgs/OccupancyGrid）
    - 为 Nav2 提供静态地图数据
    
    启动时机：
    - 在机器人生成后启动（确保 TF 树已建立）
    - 不需要过早启动，因为地图是静态的
    
    【2. AMCL (自适应蒙特卡洛定位)】
    
    功能：
    - 使用激光雷达数据和地图进行定位
    - 动态发布 odom → map 的 TF 变换
    - 修正里程计的累积误差
    
    为什么需要 AMCL？
    - 静态 TF 无法修正里程计误差
    - AMCL 通过传感器数据与地图匹配，实时更新定位
    - 提供准确的 map → odom → robot_footprint 变换链
    
    依赖关系：
    - 需要 map_server 提供地图
    - 需要 /scan 话题（激光雷达数据）
    - 需要 /odom 话题（里程计数据）
    
    【3. 生命周期节点依赖关系】
    
    Nav2 节点的启动顺序：
    1. map_server → 提供地图
    2. amcl → 提供定位（需要地图）
    3. planner_server, controller_server → 需要地图和定位
    4. bt_navigator → 需要 planner 和 controller
    5. recoveries_server → 可以独立启动
    
    注意：所有 Nav2 节点都是生命周期节点，需要 lifecycle_manager 激活
    """
    
    # 地图文件路径（使用 SLAM 生成的新地图）
    map_file = os.path.join(pkg_share, 'maps', 'warehouse_slam_map.yaml')

    # AMCL 参数文件路径
    amcl_params_file = os.path.join(pkg_share, 'navigation', 'params', 'amcl_params.yaml')

    # 地图服务器节点 - 生命周期节点
    # 延迟启动：等待机器人实体生成完成（3秒后）
    map_server_node = TimerAction(
        period=4.0,  # 等待 spawn_entity 完成（3秒）后再启动
        actions=[
            LifecycleNode(
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
        ]
    )

    # AMCL 定位节点 - 生命周期节点
    # 延迟启动：等待地图服务器启动（4秒后）
    amcl_node = TimerAction(
        period=5.0,  # 等待 map_server 启动（4秒）后再启动
        actions=[
            LifecycleNode(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace='',
                output='screen',
                parameters=[
                    RewrittenYaml(
                        source_file=amcl_params_file,
                        root_key='',
                        param_rewrites={},
                        convert_types=True
                    ),
                    {'use_sim_time': use_sim_time}
                ],
            )
        ]
    )

    # ============================================================================
    # 第八部分：Nav2 导航系统
    # ============================================================================
    """
    Nav2 导航系统启动说明：
    
    Nav2 是 ROS2 的导航栈，提供自主导航功能。主要组件包括：
    
    1. **controller_server**: 局部路径跟踪控制器
       - 接收全局路径和局部代价地图
       - 发布速度命令到 /cmd_vel
       - 使用 DWB (Dynamic Window Behavior) 算法
    
    2. **planner_server**: 全局路径规划器
       - 接收目标点
       - 使用地图和全局代价地图规划路径
       - 使用 NavFn 或 A* 算法
    
    3. **bt_navigator**: 行为树导航器
       - 协调导航行为
       - 处理导航任务的状态机
       - 管理恢复行为
    
    4. **recoveries_server**: 恢复行为服务器
       - 当机器人卡住时执行恢复动作
       - 包括旋转、后退、等待等
    
    【生命周期节点依赖关系】
    
    Nav2 节点之间存在依赖关系，需要按顺序激活：
    
    1. map_server → 必须先激活（提供地图）
    2. amcl → 需要地图，可以在地图激活后启动
    3. planner_server, controller_server → 需要地图和定位
    4. bt_navigator → 需要 planner 和 controller
    5. recoveries_server → 可以独立启动
    
    注意：所有节点都是生命周期节点，不会自动激活。
    lifecycle_manager 会按顺序激活所有节点。
    
    【为什么使用一个 lifecycle_manager？】
    
    - Nav2 的 lifecycle_manager 支持管理多个节点
    - 它会按顺序激活节点（先激活 map_server，再激活其他节点）
    - 使用一个管理器更简洁，避免冲突
    - 所有节点在同一个管理器中，便于统一控制
    
    数据流：
    ┌─────────────┐      ┌──────────────┐      ┌─────────────┐
    │  用户目标    │  →   │ bt_navigator │  →   │ planner     │
    │ (RViz点击)   │      │              │      │ (全局路径)   │
    └─────────────┘      └──────────────┘      └─────────────┘
                                ↓
                         ┌──────────────┐
                         │ controller   │  →  /cmd_vel → 机器人移动
                         │ (局部控制)    │
                         └──────────────┘
    
    关键话题：
    - /cmd_vel: 速度命令（geometry_msgs/Twist）
    - /plan: 全局路径（nav_msgs/Path）
    - /local_plan: 局部路径
    - /navigate_to_pose: 导航目标（action）
    """
    
    # Nav2 参数文件路径
    nav2_params_file = os.path.join(pkg_share, 'navigation', 'params', 'nav2_params.yaml')
    
    # 使用 RewrittenYaml 处理参数文件中的路径替换
    nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={},
        convert_types=True
    )
    # print(nav2_params) 
    # Nav2 Controller Server - 局部路径跟踪
    # 延迟启动：等待地图服务器和 AMCL 启动（5秒后）
    controller_server = TimerAction(
        period=6.0,  # 等待 map_server 和 amcl 启动
        actions=[
            LifecycleNode(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                namespace='',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            )
        ]
    )
    
    # Nav2 Planner Server - 全局路径规划
    # 延迟启动：等待地图服务器和 AMCL 启动（5秒后）
    planner_server = TimerAction(
        period=6.0,  # 等待 map_server 和 amcl 启动
        actions=[
            LifecycleNode(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace='',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            )
        ]
    )
    
    # Nav2 BT Navigator - 行为树导航器
    # 延迟启动：等待 planner 和 controller 启动（6秒后）
    bt_navigator = TimerAction(
        period=6.0,  # 与 planner 和 controller 同时启动
        actions=[
            LifecycleNode(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace='',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            )
        ]
    )
    
    # Nav2 Behavior Server - 恢复行为服务器
    # 功能：提供 spin, backup, wait 等恢复行为
    # 注意：bt_navigator 需要 behavior_server 提供的 spin action server
    # 延迟启动：与其他 Nav2 节点同时启动（6秒后）
    behavior_server = TimerAction(
        period=6.0,  # 与其他 Nav2 节点同时启动
        actions=[
            LifecycleNode(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                namespace='',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            )
        ]
    )
    
    # Nav2 Smoother Server - 路径平滑服务器（可选）
    # 功能：平滑全局路径，使路径更平滑
    # 延迟启动：与其他 Nav2 节点同时启动（6秒后）
    smoother_server = TimerAction(
        period=6.0,  # 与其他 Nav2 节点同时启动
        actions=[
            LifecycleNode(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                namespace='',
                output='screen',
                parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            )
        ]
    )
    
    # Nav2 Lifecycle Manager - 统一管理所有 Nav2 生命周期节点
    # 延迟启动：等待所有节点启动完成（6秒后）
    # 注意：lifecycle_manager 会按顺序激活节点
    # 先激活 map_server 和 amcl，再激活其他节点
    nav2_lifecycle_manager = TimerAction(
        period=7.0,  # 等待所有节点启动完成
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',  # 统一名称，管理所有节点
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {
                        'node_names': [
                            'map_server',        # 先激活（提供地图）
                            'amcl',              # 再激活（提供定位）
                            'controller_server', # 然后激活导航节点
                            'planner_server',
                            'smoother_server',   # 路径平滑服务器
                            'behavior_server',   # 恢复行为服务器（bt_navigator 需要）
                            'bt_navigator',      # 最后激活（需要其他节点都准备好）
                        ]
                    },
                    {'autostart': True}  # 自动按顺序激活所有节点
                ]
            )
        ]
    )

    # ============================================================================
    # 第九部分：RViz2 可视化（可选）
    # ============================================================================
    # RViz2 配置文件路径
    rviz_config_file = os.path.join(pkg_share, 'rvizConfig', 'robot.rviz')

    # RViz2 节点（延迟启动，等待系统完全初始化）
    rviz2_node = TimerAction(
        period=8.0,  # 等待 8 秒，确保所有 Nav2 节点已被 lifecycle_manager 激活
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(use_rviz)  # 仅在 use_rviz=True 时启动
            )
        ]
    )

    # ============================================================================
    # 第十部分：业务逻辑节点（导航接口等）
    # ============================================================================
    """
    业务逻辑节点说明：

    这里启动的是与导航系统交互的高层业务节点，例如：
    - nav_interface_node: 对外提供简单的话题接口（/warehouse/nav_goal 等），
      内部通过 Nav2 的 /navigate_to_pose action 驱动机器人导航。
    - initial_pose_setter_node: 自动设置 AMCL 初始位姿，避免每次手动设置。
      默认将机器人初始位置设置为地图原点 (0, 0, 0)。
    - robot_state_machine_node: 机器人状态机，接收任务后自动完成取货、送货流程。
      订阅：/warehouse/robot/task (std_msgs/String, 格式: "storage_name dispatch_name product_name")

    启动时机：
    - initial_pose_setter_node: 在 AMCL 启动后启动（6 秒后），等待 AMCL 就绪后发布初始位姿
    - nav_interface_node: 在 Nav2 的生命周期管理器启动之后再启动（8 秒后）
    - robot_state_machine_node: 在 Nav2 的生命周期管理器启动之后再启动（8 秒后）
    - 这里通过 TimerAction 进行简单的时间延迟，确保 Nav2 已完成激活流程

    支持两种启动方式：
    1. 组件方式（use_composition=True）：使用组件管理器动态加载组件（nav_interface_node, initial_pose_setter_node）
    2. 可执行文件方式（use_composition=False）：直接运行可执行文件（向后兼容）
    注意：robot_state_machine_node 和 warehouse_spawner_node 总是以可执行文件方式启动
    """

    # 方式1：使用组件方式加载（推荐）
    composable_node_container = TimerAction(
        period=8.0,  # 等待 Nav2 全部激活后再启动业务节点
        actions=[
            ComposableNodeContainer(
                name='business_nodes_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='ros2_warehouse_amr',
                        plugin='ros2_warehouse_amr::NavInterface',
                        name='nav_interface_node',
                        parameters=[{'use_sim_time': use_sim_time}],
                    ),
                    ComposableNode(
                        package='ros2_warehouse_amr',
                        plugin='ros2_warehouse_amr::InitialPoseSetter',
                        name='initial_pose_setter_node',
                        parameters=[
                            {'use_sim_time': use_sim_time},
                            {'initial_x': 0.0},
                            {'initial_y': 0.0},
                            {'initial_yaw': 0.0},
                            {'wait_time': 5.0},  # 等待 AMCL 启动的时间（秒）
                        ],
                    ),
                ],
                output='screen',
                condition=IfCondition(PythonExpression(["'", use_composition, "' == 'True'"])),
            )
        ]
    )

    # 方式2：使用可执行文件方式（向后兼容）
    initial_pose_setter_node = TimerAction(
        period=6.0,  # 等待 AMCL 启动（5秒后）再启动
        actions=[
            Node(
                package='ros2_warehouse_amr',
                executable='initial_pose_setter_node',
                name='initial_pose_setter_node',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'initial_x': 0.0},      # 地图原点 X
                    {'initial_y': 0.0},       # 地图原点 Y
                    {'initial_yaw': 0.0},     # 地图原点偏航角
                    {'wait_time': 5.0},       # 等待 AMCL 启动的时间（秒）
                ],
                condition=UnlessCondition(PythonExpression(["'", use_composition, "' == 'True'"])),
            )
        ]
    )

    nav_interface_node = TimerAction(
        period=8.0,  # 等待 Nav2 全部激活后再启动业务节点
        actions=[
            Node(
                package='ros2_warehouse_amr',
                executable='nav_interface_node',
                name='nav_interface_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                condition=UnlessCondition(PythonExpression(["'", use_composition, "' == 'True'"])),
            )
        ]
    )

    # 机器人状态机节点
    # 功能：接收任务后自动完成取货、送货流程
    # 订阅：/warehouse/robot/task (std_msgs/String, 格式: "storage_name dispatch_name product_name")
    robot_state_machine_node = TimerAction(
        period=8.0,  # 等待 Nav2 全部激活后再启动业务节点
        actions=[
            Node(
                package='ros2_warehouse_amr',
                executable='robot_state_machine_node',
                name='robot_state_machine_node',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'robot_name': 'amr'},
                    {'loading_duration': 3.0},      # 装载等待时间（秒）
                    {'unloading_duration': 2.0},    # 卸载等待时间（秒）
                    {'cargo_spawn_height': 0.3},    # 货物生成高度（相对于机器人）
                ],
            )
        ]
    )

    # ============================================================================
    # 第八部分：组装 Launch 描述
    # ============================================================================

    return LaunchDescription([
        # --- Launch 参数声明 ---
        # 仿真控制
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='使用仿真时间（Gazebo 时钟）'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='True',
            description='是否启动 Gazebo 图形界面'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='无头模式（用于远程服务器/CI）'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='是否启动 RViz2 可视化（默认 True，可设置为 False）'
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='是否使用组件方式加载（True=组件方式，False=可执行文件方式）'
        ),

        # 世界和机器人选择
        DeclareLaunchArgument(
            'world',
            default_value=world_default,
            description='Gazebo 世界文件路径'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='delivery_robot.xacro',
            description='机器人模型文件名（位于 models/urdf/ 目录）'
        ),

        # 机器人初始位姿
        DeclareLaunchArgument('x', default_value='0.0', description='初始 X 坐标'),
        DeclareLaunchArgument('y', default_value='0.0', description='初始 Y 坐标'),
        DeclareLaunchArgument('z', default_value='0.13', description='初始 Z 坐标（高度）'),
        DeclareLaunchArgument('roll', default_value='0.0', description='初始横滚角（弧度）'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='初始俯仰角（弧度）'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='初始偏航角（弧度）'),

        # --- 启动信息 ---
        LogInfo(msg=['Launching Gazebo with world: ', world]),
        LogInfo(msg=['Robot model: ', robot_model]),

        # --- 环境变量设置（必须在 Gazebo 启动前执行）---
        set_model_path,
        set_plugin_path,
        set_resource_path,

        # --- Gazebo 仿真器 ---
        gzserver,
        gzclient,

        # --- 机器人节点 ---
        robot_state_publisher,
        joint_state_publisher,

        # --- 机器人生成（延迟 3 秒）---
        spawn_entity,

        # --- 仓库对象生成（Storage 和 Dispatch，延迟 4 秒）---
        warehouse_spawner_node,

        # --- 地图服务器和定位系统 ---
        map_server_node,  # 延迟 4 秒启动
        amcl_node,        # 延迟 5 秒启动（需要地图）

        # --- Nav2 导航系统 ---
        controller_server,  # 延迟 6 秒启动（需要地图和定位）
        planner_server,     # 延迟 6 秒启动（需要地图和定位）
        smoother_server,   # 延迟 6 秒启动（路径平滑服务器）
        behavior_server,   # 延迟 6 秒启动（恢复行为服务器，bt_navigator 需要）
        bt_navigator,       # 延迟 6 秒启动（需要 planner, controller 和 behavior_server）
        
        # --- Nav2 生命周期管理器（统一管理所有节点）---
        nav2_lifecycle_manager,  # 延迟 7 秒启动，按顺序激活所有节点

        # --- RViz2 可视化（可选，延迟启动）---
        rviz2_node,

        # --- 业务逻辑节点（延迟启动，依赖 Nav2 已经就绪）---
        composable_node_container,  # 组件容器（组件方式）
        initial_pose_setter_node,  # 初始位姿设置节点（可执行文件方式）
        nav_interface_node,         # 导航接口节点（可执行文件方式）
        robot_state_machine_node,  # 机器人状态机节点（总是启动）
    ])
