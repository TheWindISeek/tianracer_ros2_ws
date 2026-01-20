from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def resolve_world_path(context, *args, **kwargs):
    """解析world文件路径，支持package://格式和相对路径"""
    world_path = context.launch_configurations.get('world', '')
    
    if not world_path:
        return []  # 返回空，使用默认空world
    
    # 处理package://格式
    if world_path.startswith('package://'):
        # 格式: package://package_name/path/to/world.world
        parts = world_path.replace('package://', '').split('/', 1)
        if len(parts) == 2:
            package_name, relative_path = parts
            try:
                pkg_share = get_package_share_directory(package_name)
                world_path = os.path.join(pkg_share, relative_path)
            except:
                print(f"Warning: Package '{package_name}' not found, using original path")
    
    # 处理相对路径（相对于当前工作目录）
    elif not os.path.isabs(world_path):
        world_path = os.path.abspath(world_path)
    
    # 检查文件是否存在
    if world_path and not os.path.exists(world_path):
        print(f"Warning: World file not found: {world_path}")
    
    # 更新launch配置
    context.launch_configurations['world'] = world_path
    return []


def log_urdf_path(context, *args, **kwargs):
    """在launch时打印解析后的URDF/Xacro路径"""
    urdf_path = LaunchConfiguration('urdf_file').perform(context)
    print(f"[gazebo_robot.launch] Using URDF file: {urdf_path}")
    return []


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare(package='tianbot_description_ros2').find('tianbot_description_ros2')
    
    # 获取工作空间路径
    # pkg_share 通常是: <workspace>/install/<pkg>/share/<pkg>
    # 向上4级到工作空间根目录
    workspace_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    
    # 设置默认world文件路径（map.sdf）
    default_world_path = os.path.join(workspace_path, 'src', 'map2gazebo', 'worlds', 'map.sdf')
    
    # 设置GAZEBO_MODEL_PATH环境变量
    model_path = os.path.join(workspace_path, 'src', 'map2gazebo', 'models')
    # 如果已经存在GAZEBO_MODEL_PATH，则追加；否则新建
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing_model_path:
        gazebo_model_path = f"{model_path}:{existing_model_path}"
    else:
        gazebo_model_path = model_path

    # 声明launch参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation(Gazebo) clock if true'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run headless'
    )

    # World文件参数，默认使用map.sdf世界
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='World file path. Default: map.sdf from map2gazebo',
    )
    
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        # default_value=os.path.join(pkg_share, 'urdf', 'qingzhou.xacro'),
        default_value=os.path.join(pkg_share, 'urdf', 'tianracer.xacro'),
        # default_value=os.path.join(pkg_share, 'urdf', 'tianbot_ackermann.urdf.xacro'),
        description='URDF/Xacro文件路径'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='tianbot_ackermann',
        description='机器人名称（用于Gazebo中的实体名称）'
    )
    
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.5',
        description='机器人初始X坐标（世界原点）'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.5',
        description='机器人初始Y坐标（世界原点）'
    )
    
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.0',  # base_footprint在地面
        description='机器人初始Z坐标（世界原点）'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',  # 默认朝向 0，和 Nav2 初始位姿一致
        description='机器人初始Yaw角度（弧度）'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    urdf_file = LaunchConfiguration('urdf_file')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    
    # 启动gazebo服务器(gzserver)
    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
        output='screen'
    )

    # 启动Gazebo客户端(gzclient)
    gzclient = ExecuteProcess(
        condition=IfCondition(gui),
        cmd=['gzclient'],
        output='screen'
    )


    # 处理URDF文件
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Joint State Publisher (用于Gazebo仿真)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # 在Gazebo中生成机器人实体
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', robot_name,
            '-topic', '/robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw
        ],
        output='screen'
    )

    # 解析world路径（在launch时执行）
    resolve_world = OpaqueFunction(function=resolve_world_path)
    log_urdf = OpaqueFunction(function=log_urdf_path)
    
    # 设置GAZEBO_MODEL_PATH环境变量
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        gazebo_model_path
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        world_arg,
        urdf_file_arg,
        robot_name_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        set_gazebo_model_path,  # 设置环境变量
        resolve_world,  # 先解析world路径
        log_urdf,
        gzserver,
        gzclient,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
    ])
