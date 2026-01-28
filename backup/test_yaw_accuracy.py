#!/usr/bin/env python3
"""
测试odom yaw精度的脚本

使用方法：
1. 启动tianracer
2. 运行此脚本: python3 test_yaw_accuracy.py
3. 手动控制小车转弯，观察输出
4. 按Ctrl+C结束，会显示统计信息
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


def quat_to_yaw(q):
    """从四元数提取yaw角"""
    return 2.0 * math.atan2(q.z, q.w)


def normalize_angle(a):
    """归一化角度到[-180, 180]"""
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


class YawTester(Node):
    def __init__(self):
        super().__init__('yaw_tester')
        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.initial_yaw_deg = None
        self.current_yaw_deg = 0.0
        self.prev_yaw_deg = None
        self.cumulative_change = 0.0  # 累计变化（不归一化）
        self.frame_count = 0

        self.get_logger().info('Yaw测试器已启动，等待odom数据...')
        self.get_logger().info('显示格式: 当前yaw | 累计变化')
        self.get_logger().info('按Ctrl+C结束测试')

    def odom_callback(self, msg):
        yaw_rad = quat_to_yaw(msg.pose.pose.orientation)
        yaw_deg = math.degrees(yaw_rad)
        self.current_yaw_deg = yaw_deg
        self.frame_count += 1

        if self.initial_yaw_deg is None:
            self.initial_yaw_deg = yaw_deg
            self.prev_yaw_deg = yaw_deg
            self.get_logger().info(f'初始yaw: {yaw_deg:+.1f}°')
            return

        # 计算与上一帧的差值（处理±180°跳变）
        delta = normalize_angle(yaw_deg - self.prev_yaw_deg)
        self.cumulative_change += delta
        self.prev_yaw_deg = yaw_deg

        # 每50帧打印一次
        if self.frame_count % 50 == 0:
            self.get_logger().info(
                f'当前yaw: {yaw_deg:+7.1f}° | 累计变化: {self.cumulative_change:+7.1f}°'
            )

    def print_summary(self):
        if self.initial_yaw_deg is None:
            return

        print('\n' + '='*50)
        print('测试总结')
        print('='*50)
        print(f'初始yaw:    {self.initial_yaw_deg:+.1f}°')
        print(f'最终yaw:    {self.current_yaw_deg:+.1f}°')
        print(f'累计变化:   {self.cumulative_change:+.1f}°')
        print('='*50)
        print('\n对比实际转过的角度来计算 yaw_scale:')
        print('  yaw_scale = 实际角度 / odom显示角度')
        if abs(self.cumulative_change) > 0:
            print(f'  例如实际转了90°，odom显示{abs(self.cumulative_change):.0f}°')
            print(f'  则 yaw_scale = 90 / {abs(self.cumulative_change):.0f} = {90/abs(self.cumulative_change):.2f}')


def main():
    rclpy.init()
    node = YawTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
