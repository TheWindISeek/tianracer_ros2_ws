#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive


class CmdVelToAckermann(Node):
    """
    将cmd_vel (Twist) 转换为 ackermann_cmd (AckermannDrive) 或直接转发Twist
    用于补偿小车前轮歪斜的问题
    """
    
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')
        
        # 声明参数
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/tianracer/ackermann_cmd')
        self.declare_parameter('output_type', 'auto')  # auto, twist, ackermann
        self.declare_parameter('wheelbase', 0.40)  # 轴距，单位：米（实测值）
        self.declare_parameter('steering_offset_degrees', 0.0)  # 舵机偏移，单位：度（固件已补偿 -21 度，此处设为 0）
        
        # 获取参数
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        output_type = self.get_parameter('output_type').get_parameter_value().string_value
        wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        steering_offset_degrees = self.get_parameter('steering_offset_degrees').get_parameter_value().double_value
        
        # 判断输出类型
        if output_type == 'auto':
            if 'turtle' in output_topic.lower():
                output_type = 'twist'
            else:
                output_type = 'ackermann'
        
        self._output_type = output_type
        self._wheelbase = wheelbase
        self._offset_radians = math.radians(steering_offset_degrees)
        
        # 创建发布者
        if output_type == 'twist':
            self._pub_ackermann = None
            self._pub_twist = self.create_publisher(Twist, output_topic, 1)
            self.get_logger().info("输出类型: Twist (转发模式)")
        else:
            self._pub_ackermann = self.create_publisher(AckermannDrive, output_topic, 1)
            self._pub_twist = None
            self.get_logger().info("输出类型: AckermannDrive (转换模式)")
        
        # 创建订阅者
        self._sub = self.create_subscription(
            Twist, 
            input_topic, 
            self.cmd_vel_callback, 
            1
        )
        
        self.get_logger().info("cmd_vel_to_ackermann node started")
        self.get_logger().info(f"Subscribing to: {input_topic}")
        self.get_logger().info(f"Publishing to: {output_topic}")
        self.get_logger().info(f"Wheelbase: {wheelbase:.2f} m")
        self.get_logger().info(f"Steering offset: {steering_offset_degrees:.2f} degrees")
    
    def cmd_vel_callback(self, data):
        """
        将cmd_vel (Twist) 转换为 ackermann_cmd (AckermannDrive) 或直接转发Twist
        """
        # 获取速度
        v = data.linear.x  # 线速度
        omega = data.angular.z  # 角速度
        
        # 打印收到的cmd_vel消息
        self.get_logger().info(f"收到cmd_vel: linear.x={v:.3f}, angular.z={omega:.3f}")
        
        # 如果发布到小乌龟，应用偏移后转发Twist消息
        if self._output_type == 'twist':
            # 创建新的Twist消息
            corrected_twist = Twist()
            corrected_twist.linear.x = v
            corrected_twist.linear.y = data.linear.y
            corrected_twist.linear.z = data.linear.z
            corrected_twist.angular.x = data.angular.x
            corrected_twist.angular.y = data.angular.y
            
            # 应用偏移：将偏移角度转换为角速度
            # 对于差速驱动，偏移相当于在直行时添加一个角速度
            # 偏移-10度意味着需要向右转来补偿左偏
            if v != 0:  # 只有在有速度时才应用偏移
                # 将偏移角度转换为角速度（rad/s）
                # 偏移角度越大，需要的补偿角速度越大
                # 简单模型：角速度偏移 = (偏移角度 * 线速度) / 轴距
                angular_offset = (self._offset_radians * abs(v)) / self._wheelbase
                corrected_twist.angular.z = omega + angular_offset
            else:
                corrected_twist.angular.z = omega
            
            self._pub_twist.publish(corrected_twist)
            angular_offset_val = (self._offset_radians * abs(v)) / self._wheelbase if v != 0 else 0
            self.get_logger().info(
                f"转发Twist: linear.x={v:.3f}, angular.z={omega:.3f} -> {corrected_twist.angular.z:.3f} "
                f"(偏移: {math.degrees(self._offset_radians):.3f} deg, 角速度补偿: {angular_offset_val:.3f})"
            )
            return
        
        # 否则转换为Ackermann消息
        # 计算转向角
        if omega == 0 or v == 0:
            steering = 0
        else:
            # Ackermann转向几何
            radius = v / omega  # 转弯半径
            steering = math.atan(self._wheelbase / radius)
        
        # 添加偏移量（修正舵机中位）
        steering += self._offset_radians
        
        # 创建Ackermann消息
        msg = AckermannDrive()
        msg.steering_angle = steering
        msg.speed = v
        msg.steering_angle_velocity = 0.0
        msg.acceleration = 0.0
        msg.jerk = 0.0
        
        # 发布消息
        self._pub_ackermann.publish(msg)
        
        # 打印转换后的信息
        self.get_logger().info(
            f"转换后: speed={msg.speed:.3f}, steering_angle={math.degrees(steering):.3f} deg "
            f"({steering:.3f} rad), offset={math.degrees(self._offset_radians):.3f} deg"
        )


def main(args=None):
    try:
        rclpy.init(args=args)
        node = CmdVelToAckermann()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

