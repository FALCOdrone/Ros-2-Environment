#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TwistStamped
from builtin_interfaces.msg import Time
import math

class ReferenceTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('reference_trajectory_publisher')

        # Publishers for pose and velocity
        self.pose_pub = self.create_publisher(PoseStamped, '/reference_pose', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/reference_velocity', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info("Reference trajectory publisher started.")

    def timer_callback(self):

        hov_pose_msg, hover_vel_msg = self.hover_trajectory()

        # Publish
        self.pose_pub.publish(hov_pose_msg)
        self.vel_pub.publish(hover_vel_msg)

    def hover_trajectory(self):
        # Hover trajectory at a fixed position
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 1.0

        current_time = self.get_clock().now()
        delta_time = current_time - self.start_time
        vel_msg = TwistStamped()
        vel_msg.header.stamp = current_time.to_msg()
        vel_msg.header.frame_id = "world"
        vel_msg.twist.linear.x = 0.0
        vel_msg.twist.linear.y = 0.0
        vel_msg.twist.linear.z = 0.0
        vel_msg.twist.angular.x = 0.0
        vel_msg.twist.angular.y = 0.0
        vel_msg.twist.angular.z = 0.0
        self.get_logger().info(f"Publishing hover trajectory at: ({pose_msg.pose.position.x}, {pose_msg.pose.position.y}, {pose_msg.pose.position.z})")

        return pose_msg, vel_msg
    
    def circular_trajectory(self):
        t_now = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time

        # Example: circular trajectory in XY with constant Z and speed
        radius = 2.0  # meters
        speed = 0.5   # radians/sec
        x = radius * math.cos(speed * t_now)
        y = radius * math.sin(speed * t_now)
        z = 1.0  # constant height

        vx = -radius * speed * math.sin(speed * t_now)
        vy = radius * speed * math.cos(speed * t_now)
        vz = 0.0

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0
        vel_msg = TwistStamped()
        vel_msg.header.stamp = pose_msg.header.stamp
        vel_msg.header.frame_id = "world"
        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        vel_msg.twist.linear.z = vz
        vel_msg.twist.angular.x = 0.0
        vel_msg.twist.angular.y = 0.0
        vel_msg.twist.angular.z = 0.0
        self.get_logger().info(f"Publishing circular trajectory at: ({x:.2f}, {y:.2f}, {z:.2f}), velocity: ({vx:.2f}, {vy:.2f}, {vz:.2f})")
        return pose_msg, vel_msg
    
def main(args=None):
    rclpy.init(args=args)
    node = ReferenceTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()