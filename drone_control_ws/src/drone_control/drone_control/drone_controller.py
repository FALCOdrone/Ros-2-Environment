#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3, PoseStamped, TwistStamped, Wrench
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
from .controllers import Controllers
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
from rclpy.time import Time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Initialize controller
        self.controller = Controllers(drone_mass=1.0)
        
        # Current state
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.current_attitude = np.zeros(3)  # roll, pitch, yaw
        self.current_angular_velocity = np.zeros(3)
        
        # Desired state
        self.desired_position = np.zeros(3)
        self.desired_velocity = np.zeros(3)
        self.desired_attitude = np.zeros(3)
        
        # Publishers
        self.control_pub = self.create_publisher(
            Wrench, 
            '/drone/control_wrench', 
            10
        )
        
        self.debug_pub = self.create_publisher(
            Float64MultiArray,
            '/drone/control_debug',
            10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone/pose',
            self.pose_callback,
            10
        )
        
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/drone/velocity',
            self.velocity_callback,
            10
        )
        
        self.setpoint_sub = self.create_subscription(
            PoseStamped,
            '/drone/setpoint',
            self.setpoint_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/drone/imu',
            self.imu_callback,
            10
        )
        
        # Control timer
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Drone Controller initialized')

    def pose_callback(self, msg):
        """Update current position and orientation from pose"""
        self.current_position[0] = msg.pose.position.x
        self.current_position[1] = msg.pose.position.y
        self.current_position[2] = msg.pose.position.z
        
        # Convert quaternion to Euler angles
        quaternion = msg.pose.orientation
        self.current_attitude = self.quaternion_to_euler(
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        )

    def velocity_callback(self, msg):
        """Update current velocity"""
        self.current_velocity[0] = msg.twist.linear.x
        self.current_velocity[1] = msg.twist.linear.y
        self.current_velocity[2] = msg.twist.linear.z
        
        self.current_angular_velocity[0] = msg.twist.angular.x
        self.current_angular_velocity[1] = msg.twist.angular.y
        self.current_angular_velocity[2] = msg.twist.angular.z

    def imu_callback(self, msg):
        """Update angular velocity from IMU (backup)"""
        self.current_angular_velocity[0] = msg.angular_velocity.x
        self.current_angular_velocity[1] = msg.angular_velocity.y
        self.current_angular_velocity[2] = msg.angular_velocity.z

    def setpoint_callback(self, msg):
        """Update desired position and yaw"""
        self.desired_position[0] = msg.pose.position.x
        self.desired_position[1] = msg.pose.position.y
        self.desired_position[2] = msg.pose.position.z
        
        # Extract yaw from quaternion
        quaternion = msg.pose.orientation
        _, _, yaw = self.quaternion_to_euler(
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        )
        self.desired_attitude[2] = yaw

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

    def control_loop(self):
        """Main control loop"""
        try:
            # Get control commands from the controller
            thrust, torque = self.controller.low_level_control(
                self.desired_position,
                self.desired_velocity,  # Currently zero, could be extended
                self.desired_attitude,
                self.current_position,
                self.current_velocity,
                self.current_attitude
            )
            
            # Create and publish control message
            control_msg = Wrench()
            control_msg.force.z = float(thrust)  # Thrust in Z direction
            control_msg.torque.x = float(torque[0])  # Roll torque
            control_msg.torque.y = float(torque[1])  # Pitch torque
            control_msg.torque.z = float(torque[2])  # Yaw torque
            
            self.control_pub.publish(control_msg)
            
            # Publish debug information
            debug_msg = Float64MultiArray()
            debug_msg.data = [
                float(self.desired_position[0]), float(self.desired_position[1]), float(self.desired_position[2]),
                float(self.current_position[0]), float(self.current_position[1]), float(self.current_position[2]),
                float(thrust), float(torque[0]), float(torque[1]), float(torque[2])
            ]
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
