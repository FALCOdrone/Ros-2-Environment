#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy

class PSForward(Node):
    def __init__(self, drone_id):
        super().__init__('ps_forward')

        self.scaling = 10.0
        self.pub = self.create_publisher(Twist, f'/quadrotor_{drone_id}/cmd_vel', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.get_logger().info(f"Joystick forwarding node started for quadrotor_{drone_id}")

    def joy_callback(self, data):
        axes = data.axes
        cmd = Twist(
            linear=Vector3(x=axes[3]*self.scaling, y=axes[2]*self.scaling, z=axes[1]*self.scaling),
            angular=Vector3(x=0.0, y=0.0, z=axes[0]*self.scaling)
        )
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run <package_name> ps_forward_node.py <drone_id>")
        return
    
    drone_id = sys.argv[1]
    node = PSForward(drone_id)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()