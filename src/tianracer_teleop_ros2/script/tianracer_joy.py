#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy


class TianracerJoyTeleop(Node):
    """Tianracer joy teleop node. rewrite from cpp turtlebot logitech teleop"""

    def __init__(self):
        super().__init__('tianracer_joy_teleop')
        self.get_logger().info("Tianracer JoyTeleop Initializing...")
        
        self._ackermann = AckermannDrive()
        self._ackermann.speed = 0.0
        self._ackermann.steering_angle = 0.0
        self._ackermann.steering_angle_velocity = 0.0
        self._ackermann.acceleration = 0.0
        self._ackermann.jerk = 0.0
        
        self._zero_ackermann = AckermannDrive()
        self._zero_ackermann.speed = 0.0
        self._zero_ackermann.steering_angle = 0.0
        self._zero_ackermann.steering_angle_velocity = 0.0
        self._zero_ackermann.acceleration = 0.0
        self._zero_ackermann.jerk = 0.0
        
        self._deadman_pressed = False
        self._zero_ackermann_published = False

        self._ackermann_cmd_pub = self.create_publisher(
            AckermannDrive, 
            '/tianracer/ackermann_cmd', 
            5
        )
        self._joy_sub = self.create_subscription(
            Joy, 
            '/joy', 
            self.joy_callback, 
            10
        )
        self._timer = self.create_timer(0.05, self.joystick_controller)
        self._axis_throttle = 1

        # 声明参数
        self.declare_parameter('joy_mode', 'D')
        self.declare_parameter('throttle_scale', 0.5)
        self.declare_parameter('servo_scale', 1.0)
        
        _joy_mode = self.get_parameter('joy_mode').get_parameter_value().string_value.lower()
        if _joy_mode == "d":
            self._axis_servo = 2
        elif _joy_mode == "x":
            self._axis_servo = 3
        else:
            self._axis_servo = 2  # 默认值

        self._throttle_scale = self.get_parameter('throttle_scale').get_parameter_value().double_value
        self._servo_scale = self.get_parameter('servo_scale').get_parameter_value().double_value

    def joy_callback(self, joy):
        # reset the speed every cycle.
        self._ackermann.speed = 0.0
        self._ackermann.steering_angle = 0.0
        if joy.buttons[4] == 1:
            self._ackermann.speed = joy.axes[self._axis_throttle] * self._throttle_scale * 3

        if joy.buttons[5] == 1:
            self._ackermann.steering_angle = joy.axes[self._axis_servo] * self._servo_scale * 30/180*3.1415926535

        self._deadman_pressed = joy.buttons[4] or joy.buttons[5]

    def joystick_controller(self):
        if self._deadman_pressed:
            self._ackermann_cmd_pub.publish(self._ackermann)
            self._zero_ackermann_published = False
        elif not self._zero_ackermann_published:
            self._ackermann_cmd_pub.publish(self._zero_ackermann)
            self._zero_ackermann_published = True


def main(args=None):
    try:
        rclpy.init(args=args)
        node = TianracerJoyTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().warn(f"Init node tianracer_joy_teleop failed: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
