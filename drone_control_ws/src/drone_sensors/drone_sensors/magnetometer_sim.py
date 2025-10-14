#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, PoseStamped
import numpy as np

class MagnetometerSimulator(Node):
    def __init__(self):
        super().__init__('magnetometer_simulator')
        
        # Earth's magnetic field parameters (typical values)
        # Magnetic field in NED frame (North, East, Down)
        self.earth_magnetic_field = np.array([0.2, 0.0, 0.4])  # Gauss (typical values)
        
        # Noise parameters
        self.mag_noise_std = 0.01  # Gauss
        self.bias_drift_std = 0.001  # Gauss/s
        self.mag_bias = np.zeros(3)
        
        # Current attitude
        self.current_orientation = np.array([0, 0, 0, 1])  # quaternion [x,y,z,w]
        
        # Publishers
        self.mag_pub = self.create_publisher(
            Vector3Stamped,
            '/drone/magnetometer',
            10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone/pose',
            self.pose_callback,
            10
        )
        
        # Timer for sensor updates
        self.timer = self.create_timer(0.05, self.publish_magnetometer)  # 20 Hz
        
        self.get_logger().info('Magnetometer Simulator initialized')

    def pose_callback(self, msg):
        """Update current orientation from pose"""
        self.current_orientation = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix"""
        x, y, z, w = q
        
        # Rotation matrix from quaternion
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])
        
        return R

    def publish_magnetometer(self):
        """Publish simulated magnetometer readings"""
        try:
            # Get rotation matrix from current orientation
            R_ned_to_body = self.quaternion_to_rotation_matrix(self.current_orientation).T
            
            # Transform earth's magnetic field to body frame
            mag_body = R_ned_to_body @ self.earth_magnetic_field
            
            # Add noise and bias
            noise = np.random.normal(0, self.mag_noise_std, 3)
            self.mag_bias += np.random.normal(0, self.bias_drift_std * 0.05, 3)  # Bias drift
            
            measured_mag = mag_body + noise + self.mag_bias
            
            # Publish magnetometer reading
            mag_msg = Vector3Stamped()
            mag_msg.header.stamp = self.get_clock().now().to_msg()
            mag_msg.header.frame_id = 'magnetometer_link'
            
            mag_msg.vector.x = float(measured_mag[0])
            mag_msg.vector.y = float(measured_mag[1])
            mag_msg.vector.z = float(measured_mag[2])
            
            self.mag_pub.publish(mag_msg)
            
        except Exception as e:
            self.get_logger().error(f'Magnetometer simulation error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
