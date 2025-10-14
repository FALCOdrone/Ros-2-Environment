#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')
        
        # Trajectory parameters
        self.waypoints = []
        self.current_waypoint_index = 0
        self.position_tolerance = 0.1  # meters
        
        # Current drone state
        self.current_position = np.zeros(3)
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/drone/setpoint',
            10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone/pose',
            self.pose_callback,
            10
        )
        
        # Planning timer
        self.planning_timer = self.create_timer(0.1, self.planning_loop)  # 10 Hz
        
        # Initialize with a simple square trajectory
        self.initialize_square_trajectory()
        
        self.get_logger().info('Trajectory Planner initialized')

    def initialize_square_trajectory(self):
        """Initialize a simple square trajectory for testing"""
        self.waypoints = [
            np.array([0.0, 0.0, -2.0]),   # Takeoff
            np.array([2.0, 0.0, -2.0]),   # Forward
            np.array([2.0, 2.0, -2.0]),   # Right
            np.array([0.0, 2.0, -2.0]),   # Back
            np.array([0.0, 0.0, -2.0]),   # Left (back to start)
            np.array([0.0, 0.0, -0.5]),   # Land
        ]
        self.current_waypoint_index = 0

    def pose_callback(self, msg):
        """Update current position"""
        self.current_position[0] = msg.pose.position.x
        self.current_position[1] = msg.pose.position.y
        self.current_position[2] = msg.pose.position.z

    def planning_loop(self):
        """Main trajectory planning loop"""
        try:
            if len(self.waypoints) == 0:
                return
            
            # Get current target waypoint
            if self.current_waypoint_index >= len(self.waypoints):
                self.current_waypoint_index = 0  # Loop trajectory
            
            target_waypoint = self.waypoints[self.current_waypoint_index]
            
            # Check if we've reached the current waypoint
            distance_to_waypoint = np.linalg.norm(
                self.current_position - target_waypoint
            )
            
            if distance_to_waypoint < self.position_tolerance:
                self.current_waypoint_index += 1
                self.get_logger().info(
                    f'Reached waypoint {self.current_waypoint_index - 1}, '
                    f'moving to next waypoint'
                )
                
                # If we've finished all waypoints, stay at the last one
                if self.current_waypoint_index >= len(self.waypoints):
                    self.current_waypoint_index = len(self.waypoints) - 1
                    target_waypoint = self.waypoints[self.current_waypoint_index]
            
            # Publish the current setpoint
            setpoint_msg = PoseStamped()
            setpoint_msg.header.stamp = self.get_clock().now().to_msg()
            setpoint_msg.header.frame_id = 'world'
            
            setpoint_msg.pose.position.x = float(target_waypoint[0])
            setpoint_msg.pose.position.y = float(target_waypoint[1])
            setpoint_msg.pose.position.z = float(target_waypoint[2])
            
            # Keep orientation level (yaw = 0)
            setpoint_msg.pose.orientation.x = 0.0
            setpoint_msg.pose.orientation.y = 0.0
            setpoint_msg.pose.orientation.z = 0.0
            setpoint_msg.pose.orientation.w = 1.0
            
            self.setpoint_pub.publish(setpoint_msg)
            
        except Exception as e:
            self.get_logger().error(f'Planning loop error: {e}')

    def add_waypoint(self, x, y, z):
        """Add a waypoint to the trajectory"""
        self.waypoints.append(np.array([x, y, z]))

    def clear_waypoints(self):
        """Clear all waypoints"""
        self.waypoints.clear()
        self.current_waypoint_index = 0

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
