from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare(package='tianbot_description_ros2').find('tianbot_description_ros2')
    
    # URDF文件路径
    default_urdf_path = os.path.join(pkg_share, 'urdf', 'tianbot_ackermann.urdf.xacro')
    
    # 声明启动参数
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf_path,
        description='URDF/Xacro文件路径'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='',
        description='机器人名称（用于多机器人场景）'
    )
    
    # 处理URDF文件
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_file')]),
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # Joint State Publisher (用于可视化，实际运行时由硬件驱动提供)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    return LaunchDescription([
        urdf_file_arg,
        use_sim_time_arg,
        robot_name_arg,
        robot_state_publisher,
        joint_state_publisher,
    ])

