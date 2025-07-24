#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import time

class SimpleGazeboController(Node):
    def __init__(self):
        super().__init__('simple_gazebo_controller')
        
        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.current_cmd = Twist()
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        self.get_logger().info('Simple Gazebo Controller started')
        self.get_logger().info('Listening on /cmd_vel topic')
        self.get_logger().info('Applying forces directly via ign service calls')

    def cmd_vel_callback(self, msg):
        """Receive velocity commands"""
        self.current_cmd = msg
        
    def control_loop(self):
        """Apply forces using ign service calls"""
        if (self.current_cmd.linear.x == 0 and 
            self.current_cmd.linear.y == 0 and 
            self.current_cmd.linear.z == 0 and
            self.current_cmd.angular.z == 0):
            return  # No command to apply
            
        # Calculate forces
        force_x = self.current_cmd.linear.x * 10.0  # Scale factor
        force_y = self.current_cmd.linear.y * 10.0
        force_z = self.current_cmd.linear.z * 10.0 + 10.0  # Add gravity compensation
        torque_z = self.current_cmd.angular.z * 5.0
        
        # Apply force using ign service
        try:
            cmd = [
                'ign', 'service', '-s', '/world/empty/set_physics',
                '--reqtype', 'gz.msgs.Physics',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'gravity: {{x: 0, y: 0, z: -9.81}}'
            ]
            
            # For now, let's try moving the entity directly
            move_cmd = [
                'ign', 'service', '-s', '/world/empty/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'name: "x3", position: {{x: {force_x*0.1}, y: {force_y*0.1}, z: {1.0 + force_z*0.01}}}'
            ]
            
            # Execute the command (non-blocking)
            subprocess.run(move_cmd, capture_output=True, timeout=0.5)
            
        except Exception as e:
            self.get_logger().debug(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    controller = SimpleGazeboController()
    
    print("Simple Gazebo Controller Ready!")
    print("Send commands using:")
    print("  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {z: 1.0}}'")
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
