from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    发布 SLAM 所需的静态 TF 变换。

    TF 树结构：
    map -> odom -> base_link -> laser
                            -> base_footprint
                            -> camera_link (可选)

    注意：
    - map -> odom: 由 SLAM 算法发布
    - odom -> base_link: 由里程计/机器人底盘发布
    - 其他静态变换由本脚本发布

    检测现有 TF：
    ros2 run tf2_tools view_frames
    ros2 run tf2_ros tf2_echo base_link laser
    """

    # base_link -> base_footprint (通常重合，base_footprint 在地面)
    base_link_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # base_link -> laser
    # 根据你的机器人调整 x, y, z 位置 (单位: 米)
    # x: 前后, y: 左右, z: 上下
    base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        arguments=['0.15', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
    )

    # base_link -> camera_link
    # 摄像头在 laser 前面约 3cm，所以 x = 0.15 + 0.03 = 0.18
    base_link_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_tf',
        arguments=['0.18', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
    )

    return LaunchDescription([
        base_link_to_base_footprint,
        base_link_to_laser,
        base_link_to_camera,
    ])
