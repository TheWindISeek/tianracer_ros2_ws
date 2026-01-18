from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch_ros.actions import Node
import os


def generate_launch_description():
    # 获取环境变量或使用默认值
    lidar_type = os.environ.get('TIANBOT_LIDAR', 'osight_lidar')
    robot_name = os.environ.get('TIANBOT_NAME', '/')
    
    # 机器人尺寸参数（米）
    robot_length = 0.65
    robot_width = 0.35
    robot_height = 0.20
    
    # 传感器位置：从机器人前端向后21cm，即从原点向前11.5cm (32.5 - 21 = 11.5cm)
    lidar_x = 0.115  # 从原点向前11.5cm
    lidar_y = 0.0
    lidar_z = 0.10   # 雷达高度10cm
    
    camera_x = 0.115
    camera_y = 0.0
    camera_z = 0.15  # 摄像头高度15cm
    
    imu_x = 0.0
    imu_y = 0.0
    imu_z = 0.10
    
    # 根据雷达类型设置不同的位置和角度
    lidar_configs = {
        'rplidar': {
            'x': lidar_x,
            'y': lidar_y,
            'z': lidar_z,
            'roll': 3.1415926535,  # 180度翻转
            'pitch': 0.0,
            'yaw': 0.0
        },
        'osight': {
            'x': lidar_x,
            'y': lidar_y,
            'z': lidar_z,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        },
        'rslidar': {
            'x': lidar_x + 0.1,  # 稍微前移
            'y': lidar_y,
            'z': lidar_z + 0.1,  # 稍微抬高
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }
    }
    
    # 选择雷达配置
    lidar_config = lidar_configs.get('osight', lidar_configs['osight'])  # 默认osight
    if 'rplidar' in lidar_type:
        lidar_config = lidar_configs['rplidar']
    elif 'osight' in lidar_type:
        lidar_config = lidar_configs['osight']
    elif 'rslidar' in lidar_type:
        lidar_config = lidar_configs['rslidar']
    
    # Base footprint to base_link (轮子半径高度)
    base_footprint_z = 0.13 / 2  # 轮子直径13cm，半径6.5cm
    
    nodes = []
    
    if robot_name == '/':
        # 无命名空间的情况
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint2base_link',
            arguments=[
                '0', '0', str(base_footprint_z),
                '0', '0', '0',
                'base_footprint', 'base_link'
            ],
            output='screen'
        ))
        
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2laser',
            arguments=[
                str(lidar_config['x']), str(lidar_config['y']), str(lidar_config['z']),
                str(lidar_config['roll']), str(lidar_config['pitch']), str(lidar_config['yaw']),
                'base_link', 'laser'
            ],
            output='screen'
        ))
        
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2imu',
            arguments=[
                str(imu_x), str(imu_y), str(imu_z),
                '0', '0', '0',
                'base_link', 'imu_link'
            ],
            output='screen'
        ))
        
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2camera',
            arguments=[
                str(camera_x), str(camera_y), str(camera_z),
                '0', '0', '0',
                'base_link', 'camera_link'
            ],
            output='screen'
        ))
    else:
        # 有命名空间的情况
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint2base_link',
            arguments=[
                '0', '0', str(base_footprint_z),
                '0', '0', '0',
                f'{robot_name}/base_footprint', f'{robot_name}/base_link'
            ],
            output='screen'
        ))
        
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2laser',
            arguments=[
                str(lidar_config['x']), str(lidar_config['y']), str(lidar_config['z']),
                str(lidar_config['roll']), str(lidar_config['pitch']), str(lidar_config['yaw']),
                f'{robot_name}/base_link', f'{robot_name}/laser'
            ],
            output='screen'
        ))
        
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2imu',
            arguments=[
                str(imu_x), str(imu_y), str(imu_z),
                '0', '0', '0',
                f'{robot_name}/base_link', f'{robot_name}/imu_link'
            ],
            output='screen'
        ))
        
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2camera',
            arguments=[
                str(camera_x), str(camera_y), str(camera_z),
                '0', '0', '0',
                f'{robot_name}/base_link', f'{robot_name}/camera_link'
            ],
            output='screen'
        ))
    
    return LaunchDescription(nodes)

