#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_publisher')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(0.1, self.publish_joy)  # 10 Hz

        # Example joystick values (axes: [left/right, forward/backward, throttle, yaw])
        self.axes = [0.0, 0.0, 0.0, 0.0]  # Can simulate inputs by updating these
        self.buttons = [0] * 12  # 12 buttons, all released by default

        self.get_logger().info("Joy publisher started.")

    def publish_joy(self):
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.axes = self.axes
        joy_msg.buttons = self.buttons

        self.publisher_.publish(joy_msg)
        self.get_logger().info(f"Published Joy message: axes={self.axes}, buttons={self.buttons}")


def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()