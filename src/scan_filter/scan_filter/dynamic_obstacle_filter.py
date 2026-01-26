#!/usr/bin/env python3
"""
================================================================================
智能动态障碍物过滤器 (Dynamic Obstacle Filter)
================================================================================

功能：
  1. 过滤掉与静态地图重合的激光点（避免定位误差导致的"双层墙"效应）
  2. 保留地图上空闲区域检测到的障碍物（检测新放置的动态障碍物）

原理：
  - 订阅 /scan（原始激光数据）和 /map（静态地图）
  - 对每个激光点，计算其在地图坐标系中的位置
  - 检查该位置在地图上的状态：
    - 如果地图上是障碍物（occupied）→ 过滤掉（设为inf）
    - 如果地图上是空闲（free）→ 保留（这是动态障碍物！）
    - 如果地图上是未知（unknown）→ 保留
  - 发布过滤后的 /scan_filtered 给 local_costmap 使用

使用场景：
  - 实验室环境，地图是之前建的，但有人放了新东西
  - 需要检测动态障碍物，同时避免定位误差导致的问题

================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tf2_ros
from tf2_ros import TransformException
import math


class DynamicObstacleFilter(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_filter')

        # ========== 参数声明 ==========
        self.declare_parameter('static_obstacle_threshold', 0.20)  # 米，判断是否为静态障碍物的距离阈值
        self.declare_parameter('map_occupied_threshold', 50)       # 地图占用概率阈值（0-100）
        self.declare_parameter('map_free_threshold', 30)           # 地图空闲概率阈值（0-100）
        self.declare_parameter('input_scan_topic', '/scan')
        self.declare_parameter('output_scan_topic', '/scan_filtered')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('enable_filter', True)              # 是否启用过滤
        self.declare_parameter('debug', False)                     # 调试模式

        # 获取参数
        self.static_threshold = self.get_parameter('static_obstacle_threshold').value
        self.occupied_threshold = self.get_parameter('map_occupied_threshold').value
        self.free_threshold = self.get_parameter('map_free_threshold').value
        self.input_topic = self.get_parameter('input_scan_topic').value
        self.output_topic = self.get_parameter('output_scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.enable_filter = self.get_parameter('enable_filter').value
        self.debug = self.get_parameter('debug').value

        # ========== TF2 ==========
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ========== 地图数据 ==========
        self.map_data = None
        self.map_info = None
        self.map_received = False

        # ========== QoS 配置 ==========
        # 地图使用 transient_local 确保能收到
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        # ========== 订阅和发布 ==========
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, map_qos)
        self.scan_sub = self.create_subscription(
            LaserScan, self.input_topic, self.scan_callback, 10)
        self.scan_pub = self.create_publisher(
            LaserScan, self.output_topic, 10)

        # 统计信息
        self.stats_total = 0
        self.stats_filtered = 0
        self.stats_kept_dynamic = 0

        self.get_logger().info(f'动态障碍物过滤器已启动')
        self.get_logger().info(f'  输入话题: {self.input_topic}')
        self.get_logger().info(f'  输出话题: {self.output_topic}')
        self.get_logger().info(f'  静态障碍物距离阈值: {self.static_threshold}m')
        self.get_logger().info(f'  过滤功能: {"启用" if self.enable_filter else "禁用"}')

    def map_callback(self, msg: OccupancyGrid):
        """接收地图数据"""
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        self.map_info = msg.info
        self.map_received = True
        if self.debug:
            self.get_logger().info(
                f'收到地图: {msg.info.width}x{msg.info.height}, '
                f'分辨率: {msg.info.resolution}m')

    def scan_callback(self, msg: LaserScan):
        """处理激光扫描数据"""
        # 如果禁用过滤或没有地图，直接转发
        if not self.enable_filter or not self.map_received:
            self.scan_pub.publish(msg)
            return

        # 获取 lidar -> map 的变换
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException as e:
            if self.debug:
                self.get_logger().warn(f'TF变换失败: {e}')
            self.scan_pub.publish(msg)
            return

        # 创建过滤后的scan消息
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = list(msg.ranges)
        filtered_scan.intensities = list(msg.intensities) if msg.intensities else []

        # 激光雷达在map坐标系中的位置
        lidar_x = transform.transform.translation.x
        lidar_y = transform.transform.translation.y

        # 从四元数获取yaw角
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # 统计
        total_points = 0
        filtered_points = 0
        dynamic_points = 0

        # 处理每个激光点
        for i, r in enumerate(msg.ranges):
            # 跳过无效点
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                continue

            total_points += 1

            # 计算激光点在map坐标系中的位置
            angle = msg.angle_min + i * msg.angle_increment + yaw
            point_x = lidar_x + r * math.cos(angle)
            point_y = lidar_y + r * math.sin(angle)

            # 检查该点在地图上的状态
            map_status = self.get_map_status(point_x, point_y)

            if map_status == 'occupied':
                # 地图上是障碍物 → 这是静态障碍物，过滤掉
                filtered_scan.ranges[i] = float('inf')
                filtered_points += 1
            elif map_status == 'free':
                # 地图上是空闲区域 → 这是动态障碍物，保留！
                dynamic_points += 1
            # unknown 状态也保留

        # 更新统计
        self.stats_total += total_points
        self.stats_filtered += filtered_points
        self.stats_kept_dynamic += dynamic_points

        # 调试输出
        if self.debug and total_points > 0:
            self.get_logger().info(
                f'本帧: 总点数={total_points}, 过滤静态={filtered_points}, '
                f'保留动态={dynamic_points}')

        self.scan_pub.publish(filtered_scan)

    def get_map_status(self, world_x: float, world_y: float) -> str:
        """
        获取世界坐标点在地图上的状态
        返回: 'occupied', 'free', 'unknown', 'out_of_bounds'
        """
        if self.map_info is None or self.map_data is None:
            return 'unknown'

        # 转换为地图像素坐标
        map_x = int((world_x - self.map_info.origin.position.x) / self.map_info.resolution)
        map_y = int((world_y - self.map_info.origin.position.y) / self.map_info.resolution)

        # 检查边界
        if not (0 <= map_x < self.map_info.width and 0 <= map_y < self.map_info.height):
            return 'out_of_bounds'

        # 检查该点及其周围是否有静态障碍物（考虑定位误差）
        check_radius = int(self.static_threshold / self.map_info.resolution)

        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                nx, ny = map_x + dx, map_y + dy
                if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
                    cell_value = self.map_data[ny, nx]
                    if cell_value >= self.occupied_threshold:
                        return 'occupied'

        # 检查中心点是否为空闲
        center_value = self.map_data[map_y, map_x]
        if center_value >= 0 and center_value < self.free_threshold:
            return 'free'
        elif center_value < 0:
            return 'unknown'
        else:
            return 'unknown'


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 打印统计信息
        if node.stats_total > 0:
            node.get_logger().info(
                f'统计: 总处理点数={node.stats_total}, '
                f'过滤静态障碍物={node.stats_filtered} '
                f'({100*node.stats_filtered/node.stats_total:.1f}%), '
                f'保留动态障碍物={node.stats_kept_dynamic}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
