from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os


def generate_launch_description():
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

    # 默认world文件路径（空world）
    # 可以通过以下方式指定world文件：
    # 1. 完整路径: world:=/path/to/world.world
    # 2. 相对路径: world:=./worlds/my_world.world
    # 3. 从ROS包中: world:=package://package_name/worlds/world.world
    # 4. 留空使用默认空world
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='World file path. Options: full path, relative path, package://package_name/path, or empty for default empty world',
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    
    # 启动gazebo服务器(gzserver)
    # 构建命令：如果world为空，只启动gzserver，否则加载world文件
    gzserver_cmd = ['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so']
    
    # 使用ExecuteProcess，world参数会自动处理（如果为空字符串，gzserver会使用默认空world）
    gzserver = ExecuteProcess(
        cmd=gzserver_cmd + [world],
        output='screen'
    )

    # 启动Gazebo客户端(gzclient) 根据gui参数决定
    gzclient = ExecuteProcess(
        condition=IfCondition(gui),
        cmd=['gzclient'],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        world_arg,
        gzserver,
        gzclient,
    ])

