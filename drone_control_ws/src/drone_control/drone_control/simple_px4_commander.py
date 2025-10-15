#!/usr/bin/env python3

"""
Simple example of sending commands to PX4 using MAVROS.
This demonstrates how to use PX4's built-in controllers.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode


class SimplePX4Commander(Node):
    def __init__(self):
        super().__init__('simple_px4_commander')
        
        # Publisher for position setpoints
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Timer for publishing setpoints
        self.timer = self.create_timer(0.1, self.publish_setpoint)
        
        # Current setpoint
        self.setpoint = PoseStamped()
        self.setpoint.pose.position.z = 2.0  # 2 meters altitude
        self.setpoint.pose.orientation.w = 1.0  # No rotation
        
        self.get_logger().info('Simple PX4 Commander started')
        self.get_logger().info('Commands:')
        self.get_logger().info('  ros2 service call /simple_px4_commander/arm std_srvs/srv/Empty')
        self.get_logger().info('  ros2 service call /simple_px4_commander/set_offboard std_srvs/srv/Empty')
        self.get_logger().info('  ros2 topic pub /simple_px4_commander/goto geometry_msgs/msg/Point "{x: 1.0, y: 1.0, z: 2.0}"')

    def publish_setpoint(self):
        """Continuously publish setpoint (required for OFFBOARD mode)"""
        self.setpoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint.header.frame_id = 'map'
        self.setpoint_pub.publish(self.setpoint)

    def arm_vehicle(self):
        """Arm the vehicle"""
        request = CommandBool.Request()
        request.value = True
        
        future = self.arming_client.call_async(request)
        self.get_logger().info('Arming vehicle...')

    def set_offboard_mode(self):
        """Set OFFBOARD mode"""
        request = SetMode.Request()
        request.custom_mode = 'OFFBOARD'
        
        future = self.set_mode_client.call_async(request)
        self.get_logger().info('Setting OFFBOARD mode...')

    def goto_position(self, x, y, z):
        """Set new target position"""
        self.setpoint.pose.position.x = float(x)
        self.setpoint.pose.position.y = float(y)
        self.setpoint.pose.position.z = float(z)
        self.get_logger().info(f'New target: ({x}, {y}, {z})')


def main(args=None):
    rclpy.init(args=args)
    
    node = SimplePX4Commander()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
