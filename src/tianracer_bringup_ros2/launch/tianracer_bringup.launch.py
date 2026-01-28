from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ========== Lidar 参数 ==========
    lidar_ip_arg = DeclareLaunchArgument(
        'lidar_ip',
        default_value='192.168.1.10',
        description='IP address of the lidar'
    )
    lidar_name_arg = DeclareLaunchArgument(
        'lidar_name',
        default_value='osight_lidar',
        description='Name of the lidar node'
    )
    lidar_model_arg = DeclareLaunchArgument(
        'lidar_model',
        default_value='iexxx',
        description='Lidar model type'
    )
    lidar_frame_id_arg = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='lidar',
        description='Frame ID for the lidar scan'
    )
    lidar_angle_max_arg = DeclareLaunchArgument(
        'lidar_angle_max',
        default_value='2.3562',
        description='Maximum angle for the scan'
    )
    lidar_angle_min_arg = DeclareLaunchArgument(
        'lidar_angle_min',
        default_value='-2.3562',
        description='Minimum angle for the scan'
    )

    # ========== Realsense 参数 ==========
    # 只包含最常用的参数，其他参数可以通过config_file传递
    realsense_camera_name_arg = DeclareLaunchArgument(
        'realsense_camera_name',
        default_value='camera',
        description='Camera unique name'
    )
    realsense_camera_namespace_arg = DeclareLaunchArgument(
        'realsense_camera_namespace',
        default_value='camera',
        description='Namespace for camera'
    )
    realsense_serial_no_arg = DeclareLaunchArgument(
        'realsense_serial_no',
        default_value="''",
        description='Choose device by serial number'
    )
    realsense_config_file_arg = DeclareLaunchArgument(
        'realsense_config_file',
        default_value="''",
        description='YAML config file for realsense'
    )
    realsense_enable_color_arg = DeclareLaunchArgument(
        'realsense_enable_color',
        default_value='true',
        description='Enable color stream'
    )
    realsense_enable_depth_arg = DeclareLaunchArgument(
        'realsense_enable_depth',
        default_value='true',
        description='Enable depth stream'
    )

    # ========== Tianbot Core 参数 ==========
    tianbot_serial_port_arg = DeclareLaunchArgument(
        'tianbot_serial_port',
        default_value='/dev/tianbot_racecar',
        description='Serial port device'
    )
    tianbot_serial_baudrate_arg = DeclareLaunchArgument(
        'tianbot_serial_baudrate',
        default_value='460800',
        description='Serial baudrate'
    )
    tianbot_type_arg = DeclareLaunchArgument(
        'tianbot_type',
        default_value='ackermann',
        description='Robot type: omni, diff, ackermann'
    )
    tianbot_type_verify_arg = DeclareLaunchArgument(
        'tianbot_type_verify',
        default_value='true',
        description='Verify device type'
    )
    tianbot_publish_tf_arg = DeclareLaunchArgument(
        'tianbot_publish_tf',
        default_value='true',
        description='Publish TF transforms'
    )
    tianbot_yaw_offset_arg = DeclareLaunchArgument(
        'yaw_offset_deg',
        default_value='0.0',
        description='Odom yaw offset in degrees (固件已设置steering_offset，此处设为0)'
    )
    tianbot_yaw_scale_arg = DeclareLaunchArgument(
        'yaw_scale',
        default_value='0.66',
        description='Odom yaw scale factor to correct turning angle error (实际角度/odom角度)'
    )

    # ========== 启动 Lidar ==========
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('osight_lidar_ros2'),
                'launch',
                'osight_iexxx.launch.py'
            ])
        ]),
        launch_arguments={
            'lidar_ip': LaunchConfiguration('lidar_ip'),
            'lidar_name': LaunchConfiguration('lidar_name'),
            'lidar_model': LaunchConfiguration('lidar_model'),
            'frame_id': LaunchConfiguration('lidar_frame_id'),
            'angle_max': LaunchConfiguration('lidar_angle_max'),
            'angle_min': LaunchConfiguration('lidar_angle_min'),
        }.items()
    )

    # ========== 启动 Realsense ==========
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('realsense_camera_name'),
            'camera_namespace': LaunchConfiguration('realsense_camera_namespace'),
            'serial_no': LaunchConfiguration('realsense_serial_no'),
            'config_file': LaunchConfiguration('realsense_config_file'),
            'enable_color': LaunchConfiguration('realsense_enable_color'),
            'enable_depth': LaunchConfiguration('realsense_enable_depth'),
        }.items()
    )

    # ========== 启动 Tianbot Core ==========
    tianbot_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tianbot_core_ros2'),
                'launch',
                'tianbot_core.launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': LaunchConfiguration('tianbot_serial_port'),
            'serial_baudrate': LaunchConfiguration('tianbot_serial_baudrate'),
            'type': LaunchConfiguration('tianbot_type'),
            'type_verify': LaunchConfiguration('tianbot_type_verify'),
            'publish_tf': LaunchConfiguration('tianbot_publish_tf'),
            'yaw_offset_deg': LaunchConfiguration('yaw_offset_deg'),
            'yaw_scale': LaunchConfiguration('yaw_scale'),
        }.items()
    )

    # ========== 静态 TF 转换 ==========
    # base_link -> base_footprint (通常重合，base_footprint 在地面)
    base_link_to_base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # base_link -> camera_link
    # 摄像头位置：x=0.18m (前方), y=0m, z=0.1m (高度)
    # 参考 static_tf.launch.py 中的配置
    base_link_to_camera_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link_tf',
        arguments=['0.18', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
    )

    return LaunchDescription([
        # Lidar 参数声明
        lidar_ip_arg,
        lidar_name_arg,
        lidar_model_arg,
        lidar_frame_id_arg,
        lidar_angle_max_arg,
        lidar_angle_min_arg,
        
        # Realsense 参数声明
        realsense_camera_name_arg,
        realsense_camera_namespace_arg,
        realsense_serial_no_arg,
        realsense_config_file_arg,
        realsense_enable_color_arg,
        realsense_enable_depth_arg,
        
        # Tianbot Core 参数声明
        tianbot_serial_port_arg,
        tianbot_serial_baudrate_arg,
        tianbot_type_arg,
        tianbot_type_verify_arg,
        tianbot_publish_tf_arg,
        tianbot_yaw_offset_arg,
        tianbot_yaw_scale_arg,

        # 启动三个节点
        lidar_launch,
        realsense_launch,
        tianbot_core_launch,
        
        # 静态 TF 转换节点
        base_link_to_base_footprint_tf,
        base_link_to_camera_link_tf,
    ])

