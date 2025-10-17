#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point, Vector3
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Header
import numpy as np


class MissionCommander(Node):
    def __init__(self):
        super().__init__('mission_commander')
        
        # Parameters
        self.vehicle_name = self.declare_parameter('vehicle_name', 'iris').value
        self.auto_arm = self.declare_parameter('auto_arm', True).value
        self.auto_takeoff = self.declare_parameter('auto_takeoff', True).value
        self.takeoff_altitude = self.declare_parameter('takeoff_altitude', 2.0).value
        
        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.mission_step = 0
        self.armed = False
        self.offboard_enabled = False
        
        # QoS profile for MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers - Use PX4's native setpoint topics
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            qos_profile
        )
        
        # Alternative: Raw setpoints for more direct control
        self.setpoint_raw_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            qos_profile
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        # Mission timer
        self.mission_timer = self.create_timer(0.1, self.mission_loop)  # 10 Hz
        
        # Setpoint publishing timer (required for OFFBOARD mode)
        self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint)  # 20 Hz
        
        # Current setpoint
        self.current_setpoint = PoseStamped()
        self.current_setpoint.pose.position.x = 0.0
        self.current_setpoint.pose.position.y = 0.0
        self.current_setpoint.pose.position.z = self.takeoff_altitude
        self.current_setpoint.pose.orientation.w = 1.0
        
        # Mission waypoints (simple square pattern)
        self.waypoints = [
            [0.0, 0.0, self.takeoff_altitude],      # Takeoff position
            [2.0, 0.0, self.takeoff_altitude],      # Point 1
            [2.0, 2.0, self.takeoff_altitude],      # Point 2  
            [0.0, 2.0, self.takeoff_altitude],      # Point 3
            [0.0, 0.0, self.takeoff_altitude],      # Return home
            [0.0, 0.0, 0.5],                       # Landing position
        ]
        self.current_waypoint = 0
        self.waypoint_reached_threshold = 0.3  # meters
        
        self.get_logger().info(f'Mission Commander initialized for {self.vehicle_name}')
        self.get_logger().info(f'Mission: Square pattern with {len(self.waypoints)} waypoints')

    def state_callback(self, msg):
        """Update vehicle state from MAVROS"""
        self.current_state = msg
        self.armed = msg.armed
        self.offboard_enabled = (msg.mode == "OFFBOARD")

    def pose_callback(self, msg):
        """Update current pose from MAVROS"""
        self.current_pose = msg

    def publish_setpoint(self):
        """Publish current setpoint (required for OFFBOARD mode)"""
        self.current_setpoint.header.stamp = self.get_clock().now().to_msg()
        self.current_setpoint.header.frame_id = "map"
        self.local_pos_pub.publish(self.current_setpoint)

    def mission_loop(self):
        """Main mission logic using PX4's built-in controllers"""
        
        if not self.current_state.connected:
            if self.mission_step == 0:
                self.get_logger().info(f"Waiting for FCU connection... Current state: connected={self.current_state.connected}, mode={self.current_state.mode}")
            return
            
        # Mission Step 0: Set OFFBOARD mode and publish setpoints
        if self.mission_step == 0:
            self.get_logger().info(f"FCU Connected! Mode: {self.current_state.mode}, Armed: {self.current_state.armed}")
            if not self.offboard_enabled:
                self.set_offboard_mode()
            else:
                self.get_logger().info("OFFBOARD mode enabled")
                self.mission_step = 1
                
        # Mission Step 1: Arm the vehicle
        elif self.mission_step == 1:
            if not self.armed and self.auto_arm:
                self.arm_vehicle()
            elif self.armed:
                self.get_logger().info("Vehicle armed")
                self.mission_step = 2
                
        # Mission Step 2: Takeoff
        elif self.mission_step == 2:
            #wp = self.waypoints[waypoint_index]
            altitude_ref = self.takeoff_altitude
            self.current_setpoint.pose.position.x = float(0.0)
            self.current_setpoint.pose.position.y = float(0.0)
            self.current_setpoint.pose.position.z = float(altitude_ref)
            pass
            """
            self.update_setpoint_to_waypoint(0)  # Takeoff position
            if self.is_waypoint_reached(0):
                self.get_logger().info(f"Takeoff complete at {self.takeoff_altitude}m")
                self.mission_step = 3
            """
                
        # Mission Step 3: Execute waypoint mission
        elif self.mission_step == 3:
            pass
            """
            if self.current_waypoint < len(self.waypoints):
                self.update_setpoint_to_waypoint(self.current_waypoint)
                
                if self.is_waypoint_reached(self.current_waypoint):
                    self.get_logger().info(f"Reached waypoint {self.current_waypoint}: {self.waypoints[self.current_waypoint]}")
                    self.current_waypoint += 1
                    
                    if self.current_waypoint >= len(self.waypoints):
                        self.get_logger().info("Mission complete! Landing...")
                        self.mission_step = 4
            """
        # Mission Step 4: Landing
        elif self.mission_step == 4:
            pass
            """
            self.land_vehicle()
            self.mission_step = 5  # Mission complete
            """

        # Mission Step 5: Mission complete
        elif self.mission_step == 5:
            pass  # Do nothing, mission complete

    def update_setpoint_to_waypoint(self, waypoint_index):
        """Update the current setpoint to a specific waypoint"""
        if waypoint_index < len(self.waypoints):
            wp = self.waypoints[waypoint_index]
            self.current_setpoint.pose.position.x = float(wp[0])
            self.current_setpoint.pose.position.y = float(wp[1])
            self.current_setpoint.pose.position.z = float(wp[2])

    def is_waypoint_reached(self, waypoint_index):
        """Check if current waypoint is reached"""
        if waypoint_index >= len(self.waypoints):
            return True
            
        wp = self.waypoints[waypoint_index]
        current_pos = self.current_pose.pose.position
        
        distance = np.sqrt(
            (current_pos.x - wp[0])**2 + 
            (current_pos.y - wp[1])**2 + 
            (current_pos.z - wp[2])**2
        )
        
        return distance < self.waypoint_reached_threshold

    def set_offboard_mode(self):
        """Set vehicle to OFFBOARD mode"""
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            return
            
        request = SetMode.Request()
        request.custom_mode = 'OFFBOARD'
        
        future = self.set_mode_client.call_async(request)
        # Don't wait for response to avoid blocking

    def arm_vehicle(self):
        """Arm the vehicle"""
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            return
            
        request = CommandBool.Request()
        request.value = True
        
        future = self.arming_client.call_async(request)
        # Don't wait for response to avoid blocking

    def land_vehicle(self):
        """Land the vehicle using PX4's built-in landing"""
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            return
            
        request = SetMode.Request()
        request.custom_mode = 'AUTO.LAND'
        
        future = self.set_mode_client.call_async(request)
        self.get_logger().info("Landing command sent")


def main(args=None):
    rclpy.init(args=args)
    
    node = MissionCommander()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
