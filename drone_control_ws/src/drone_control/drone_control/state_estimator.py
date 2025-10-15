#!/usr/bin/env python3
# TODO: write this node on C++ if topic not available in PX4 MAVROS

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np
from scipy.spatial.transform import Rotation

class StateEstimator(Node):
    def __init__(self):
        super().__init__('state_estimator')
        
        # State variables
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.orientation = np.array([0, 0, 0, 1])  # quaternion [x,y,z,w]
        self.angular_velocity = np.zeros(3)
        
        # IMU bias estimation
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        
        # Kalman filter variables (simplified)
        self.dt = 0.01
        self.gravity = np.array([0, 0, 9.81])
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/drone/pose',
            10
        )
        
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            '/drone/velocity',
            10
        )
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/drone/imu',
            self.imu_callback,
            10
        )
        
        self.sonar_sub = self.create_subscription(
            Float64,
            '/drone/sonar',
            self.sonar_callback,
            10
        )
        
        self.barometer_sub = self.create_subscription(
            Float64,
            '/drone/barometer',
            self.barometer_callback,
            10
        )
        
        # Estimation timer
        self.estimation_timer = self.create_timer(self.dt, self.estimation_loop)
        
        self.get_logger().info('State Estimator initialized')

    def imu_callback(self, msg):
        """Process IMU data for attitude and angular velocity estimation"""
        # Extract angular velocity (with bias compensation)
        self.angular_velocity = np.array([
            msg.angular_velocity.x - self.gyro_bias[0],
            msg.angular_velocity.y - self.gyro_bias[1],
            msg.angular_velocity.z - self.gyro_bias[2]
        ])
        
        # Simple gyro bias estimation (very basic)
        alpha = 0.001  # Learning rate
        if np.linalg.norm(self.angular_velocity) < 0.01:  # When stationary
            self.gyro_bias += alpha * np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])

    def sonar_callback(self, msg):
        """Process sonar altitude measurement"""
        # Use sonar for altitude correction (simplified)
        if msg.data > 0.05 and msg.data < 10.0:  # Valid range
            self.position[2] = msg.data  # Direct update for simplicity

    def barometer_callback(self, msg):
        """Process barometer altitude measurement"""
        # Convert pressure to altitude (simplified)
        # In real implementation, you'd use proper pressure-altitude conversion
        altitude = (1013.25 - msg.data) * 8.5  # Rough approximation
        
        # Sensor fusion with current altitude estimate
        alpha = 0.1
        self.position[2] = (1 - alpha) * self.position[2] + alpha * altitude

    def estimation_loop(self):
        """Main state estimation loop"""
        try:
            # Integrate angular velocity to get orientation (simplified)
            # In real implementation, use proper quaternion integration
            
            # Create and publish pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world'
            
            pose_msg.pose.position.x = float(self.position[0])
            pose_msg.pose.position.y = float(self.position[1])
            pose_msg.pose.position.z = float(self.position[2])
            
            pose_msg.pose.orientation.x = float(self.orientation[0])
            pose_msg.pose.orientation.y = float(self.orientation[1])
            pose_msg.pose.orientation.z = float(self.orientation[2])
            pose_msg.pose.orientation.w = float(self.orientation[3])
            
            self.pose_pub.publish(pose_msg)
            
            # Create and publish velocity
            velocity_msg = TwistStamped()
            velocity_msg.header.stamp = self.get_clock().now().to_msg()
            velocity_msg.header.frame_id = 'world'
            
            velocity_msg.twist.linear.x = float(self.velocity[0])
            velocity_msg.twist.linear.y = float(self.velocity[1])
            velocity_msg.twist.linear.z = float(self.velocity[2])
            
            velocity_msg.twist.angular.x = float(self.angular_velocity[0])
            velocity_msg.twist.angular.y = float(self.angular_velocity[1])
            velocity_msg.twist.angular.z = float(self.angular_velocity[2])
            
            self.velocity_pub.publish(velocity_msg)
            
        except Exception as e:
            self.get_logger().error(f'Estimation loop error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = StateEstimator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
