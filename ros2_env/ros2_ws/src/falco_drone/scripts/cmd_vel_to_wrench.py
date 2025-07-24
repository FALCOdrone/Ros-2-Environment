#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, WrenchStamped
import math

class CmdVelToWrench(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_wrench')
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publish wrench commands
        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            '/model/quadrotor_0/link/base_link/wrench',
            10
        )
        
        # Control parameters
        self.force_scale = 10.0      # Scale factor for linear forces
        self.torque_scale = 5.0      # Scale factor for angular torques
        self.hover_force = 14.5      # Force needed to hover (mass * gravity)
        
        self.get_logger().info('CmdVel to Wrench converter started')

    def cmd_vel_callback(self, msg):
        """Convert Twist message to Wrench and publish -> TODO: modify the forces and torques computation"""
        wrench_stamped = WrenchStamped()
        wrench_stamped.header.stamp = self.get_clock().now().to_msg()
        wrench_stamped.header.frame_id = "base_link"
        
        # Convert linear velocities to forces
        wrench_stamped.wrench.force.x = msg.linear.x * self.force_scale
        wrench_stamped.wrench.force.y = msg.linear.y * self.force_scale
        
        # For vertical movement, add to hover force
        wrench_stamped.wrench.force.z = self.hover_force + (msg.linear.z * self.force_scale)
        
        # Convert angular velocities to torques
        wrench_stamped.wrench.torque.x = msg.angular.x * self.torque_scale
        wrench_stamped.wrench.torque.y = msg.angular.y * self.torque_scale
        wrench_stamped.wrench.torque.z = msg.angular.z * self.torque_scale
        
        # Publish the wrench
        self.wrench_pub.publish(wrench_stamped)
        
        # Log for debugging
        if any([msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]):
            self.get_logger().info(
                f'Force: [{wrench_stamped.wrench.force.x:.2f}, {wrench_stamped.wrench.force.y:.2f}, {wrench_stamped.wrench.force.z:.2f}] '
                f'Torque: [{wrench_stamped.wrench.torque.x:.2f}, {wrench_stamped.wrench.torque.y:.2f}, {wrench_stamped.wrench.torque.z:.2f}]'
            )

def main():
    rclpy.init()
    
    converter = CmdVelToWrench()
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
