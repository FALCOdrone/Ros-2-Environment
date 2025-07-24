#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Wrench
from std_msgs.msg import Header
import time

class IgnitionDroneController(Node):
    def __init__(self, drone_name="quadrotor"):
        super().__init__('ignition_drone_controller')
        
        self.drone_name = drone_name
        
        # Create subscriber for cmd_vel commands
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create publisher for force/torque commands to Ignition Gazebo
        # Topic format for Ignition Gazebo ApplyLinkWrench plugin
        self.force_publisher = self.create_publisher(
            Wrench,
            f'/model/x3/wrench',  # Use the actual model name from Gazebo
            10
        )
        
        # Physical parameters
        self.mass = 1.477  # kg (from URDF)
        self.arm_length = 0.17  # meters (distance from center to motor)
        self.gravity = 9.81  # m/s²
        
        # Motor coefficients
        self.k_thrust = 1.0e-6  # Thrust coefficient (N⋅s²/rad²)
        self.k_torque = 1.0e-8  # Torque coefficient (N⋅m⋅s²/rad²)
        
        # Control gains for PID-like behavior
        self.kp_z = 15.0      # Altitude control gain
        self.kp_xy = 10.0     # Horizontal position control gain
        self.kp_roll = 8.0    # Roll control gain
        self.kp_pitch = 8.0   # Pitch control gain
        self.kp_yaw = 5.0     # Yaw control gain
        
        # Current velocities (desired)
        self.linear_vel = Vector3()
        self.angular_vel = Vector3()
        
        # Motor speeds (rad/s) - [front, right, back, left]
        self.motor_speeds = [0.0, 0.0, 0.0, 0.0]
        
        # Hover motor speed (to counteract gravity)
        self.hover_speed = (self.mass * self.gravity / (4 * self.k_thrust)) ** 0.5
        
        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        self.get_logger().info(f'Realistic Motor-Based Drone Controller started for {self.drone_name}')
        self.get_logger().info(f'Physical parameters:')
        self.get_logger().info(f'  Mass: {self.mass} kg')
        self.get_logger().info(f'  Arm length: {self.arm_length} m')
        self.get_logger().info(f'  Thrust coefficient: {self.k_thrust} N⋅s²/rad²')
        self.get_logger().info(f'  Hover motor speed: {self.hover_speed:.1f} rad/s')
        self.get_logger().info('Motor layout:')
        self.get_logger().info('    1(F)')
        self.get_logger().info('  4   2')  
        self.get_logger().info('    3(B)')
        self.get_logger().info('Listening on /cmd_vel topic')
        self.get_logger().info('Publishing realistic motor-based forces to Ignition Gazebo')

    def cmd_vel_callback(self, msg):
        """Receive velocity commands"""
        self.linear_vel = msg.linear
        self.angular_vel = msg.angular
        
    def control_loop(self):
        """Convert velocity commands to motor speeds, then to forces and torques"""

        # Computing the error between desired and current velocities

        # the measured position and orientation would typically come from sensors
        # or a state estimation node, therefore we take the measured position and orientation from 
        # the Ignition Gazebo simulation, where we can access the current state of the drone and compare it with 
        # the desired velocities, position and orientation. For simplicity, we assume the desired velocities are already set by cmd_vel.

        current_linear_vel = Vector3()
        current_angular_vel = Vector3()

        # now we take the current velocities from the Ignition Gazebo simulation
        # In a real application, you would subscribe to the drone's state topic to get these
        
        

        # Compute the error between desired and current velocities  
        # For now, we'll use direct velocity control without feedback
        # In a real system, you'd get current velocities from sensors
        
        vel_error_x = self.linear_vel.x - current_linear_vel.x
        vel_error_y = self.linear_vel.y - current_linear_vel.y  
        vel_error_z = self.linear_vel.z - current_linear_vel.z
        
        ang_error_x = self.angular_vel.x - current_angular_vel.x
        ang_error_y = self.angular_vel.y - current_angular_vel.y
        ang_error_z = self.angular_vel.z - current_angular_vel.z

        # Step 1: Convert desired velocities to required forces and torques
        desired_thrust = self.mass * self.gravity + (self.linear_vel.z * self.kp_z) # Perhaps the gravity therm needs to be negative
        desired_force_x = self.linear_vel.x * self.kp_xy
        desired_force_y = self.linear_vel.y * self.kp_xy
        
        # Convert horizontal forces to roll/pitch angles (small angle approximation)
        desired_roll = desired_force_y / (self.mass * self.gravity)  # Roll for left/right
        desired_pitch = -desired_force_x / (self.mass * self.gravity)  # Pitch for forward/back -> needs to be added the inertia therm
        
        # Desired torques for attitude control
        desired_torque_roll = desired_roll * self.kp_roll + self.angular_vel.x * self.kp_roll * 0.1
        desired_torque_pitch = desired_pitch * self.kp_pitch + self.angular_vel.y * self.kp_pitch * 0.1
        desired_torque_yaw = self.angular_vel.z * self.kp_yaw
        
        # Step 2: Calculate individual motor speeds using allocation matrix
        # Quadrotor motor arrangement:
        #     X (front)
        #  4     1
        #    \ /
        #     +  Y (right)
        #    / \
        #  3     2
        #
        # Motor allocation matrix equations:
        # Total_Thrust = F1 + F2 + F3 + F4
        # Roll_Torque = L * (F4 - F2)
        # Pitch_Torque = L * (F1 - F3)  
        # Yaw_Torque = k_d/k_t * (F1 - F2 + F3 - F4)
        
        # Base thrust per motor (hover condition)
        base_thrust = desired_thrust / 4.0
        
        # Torque to thrust conversion
        roll_thrust_diff = desired_torque_roll / self.arm_length
        pitch_thrust_diff = desired_torque_pitch / self.arm_length
        yaw_thrust_diff = desired_torque_yaw / (self.k_torque / self.k_thrust)
        
        # Calculate individual motor thrusts
        F1 = base_thrust + pitch_thrust_diff/2.0 + yaw_thrust_diff/4.0   # Front motor
        F2 = base_thrust - roll_thrust_diff/2.0 - yaw_thrust_diff/4.0   # Right motor  
        F3 = base_thrust - pitch_thrust_diff/2.0 + yaw_thrust_diff/4.0  # Back motor
        F4 = base_thrust + roll_thrust_diff/2.0 - yaw_thrust_diff/4.0   # Left motor
        
        # Step 3: Convert thrusts to motor speeds using F = k_t * ω²
        def thrust_to_motor_speed(thrust):
            if thrust < 0:
                return 0.0
            return (thrust / self.k_thrust) ** 0.5
        
        self.motor_speeds[0] = thrust_to_motor_speed(F1)  # Front
        self.motor_speeds[1] = thrust_to_motor_speed(F2)  # Right
        self.motor_speeds[2] = thrust_to_motor_speed(F3)  # Back
        self.motor_speeds[3] = thrust_to_motor_speed(F4)  # Left
        
        # Step 4: Calculate actual forces and torques from motor speeds
        # This is what the physics engine will see
        actual_forces = [self.k_thrust * speed**2 for speed in self.motor_speeds]
        
        wrench_msg = Wrench()
        
        # Total thrust (Z-direction)
        wrench_msg.force.z = sum(actual_forces)
        
        # For horizontal forces, we'll use a simplified model
        # In reality, these come from tilting the drone
        wrench_msg.force.x = desired_force_x
        wrench_msg.force.y = desired_force_y
        
        # Torques from motor speed differences
        wrench_msg.torque.x = self.arm_length * (actual_forces[3] - actual_forces[1])  # Roll
        wrench_msg.torque.y = self.arm_length * (actual_forces[0] - actual_forces[2])  # Pitch
        wrench_msg.torque.z = (self.k_torque/self.k_thrust) * (
            actual_forces[0] - actual_forces[1] + actual_forces[2] - actual_forces[3]
        )  # Yaw
        
        # Publish the wrench command
        self.force_publisher.publish(wrench_msg)
        
        # Debug output (uncomment to see motor speeds)
        # self.get_logger().info(f'Motor speeds: F={self.motor_speeds[0]:.1f}, R={self.motor_speeds[1]:.1f}, B={self.motor_speeds[2]:.1f}, L={self.motor_speeds[3]:.1f}')
        # self.get_logger().info(f'Forces: {[f"F{i+1}={f:.2f}" for i, f in enumerate(actual_forces)]}')

    def takeoff(self, height=1.0):
        """Takeoff to specified height"""
        self.get_logger().info(f'Taking off to {height}m...')
        
        # Send upward force for a duration
        msg = Twist()
        msg.linear.z = 2.0  # Upward velocity
        
        for _ in range(int(height * 20)):  # 20Hz * height seconds
            self.cmd_vel_callback(msg)
            time.sleep(0.05)
        
        # Stop
        self.stop()
        
    def land(self):
        """Land the drone"""
        self.get_logger().info('Landing...')
        msg = Twist()
        msg.linear.z = -1.0  # Downward velocity
        
        for _ in range(60):  # 3 seconds
            self.cmd_vel_callback(msg)
            time.sleep(0.05)
            
        self.stop()
        
    def stop(self):
        """Stop all movement"""
        msg = Twist()
        self.cmd_vel_callback(msg)
        
    def move_forward(self, speed=1.0, duration=1.0):
        """Move forward"""
        self.get_logger().info(f'Moving forward at {speed} m/s for {duration} seconds')
        msg = Twist()
        msg.linear.x = speed
        
        for _ in range(int(duration * 20)):
            self.cmd_vel_callback(msg)
            time.sleep(0.05)
        
        self.stop()
        
    def move_up(self, speed=1.0, duration=1.0):
        """Move up"""
        self.get_logger().info(f'Moving up at {speed} m/s for {duration} seconds')
        msg = Twist()
        msg.linear.z = speed
        
        for _ in range(int(duration * 20)):
            self.cmd_vel_callback(msg)
            time.sleep(0.05)
        
        self.stop()

    def get_motor_info(self):
        """Get current motor speeds and forces for debugging"""
        forces = [self.k_thrust * speed**2 for speed in self.motor_speeds]
        total_thrust = sum(forces)
        
        info = {
            'motor_speeds_rad_s': self.motor_speeds.copy(),
            'motor_forces_N': forces,
            'total_thrust_N': total_thrust,
            'hover_thrust_N': self.mass * self.gravity
        }
        return info

    def show_motor_physics(self, duration=5.0):
        """Demonstrate realistic motor physics by showing motor speeds during maneuvers"""
        self.get_logger().info("=== MOTOR PHYSICS DEMONSTRATION ===")
        self.get_logger().info("Watch how individual motor speeds change for different maneuvers...")
        
        # Hover
        self.get_logger().info("\n1. HOVER - All motors at equal speed")
        msg = Twist()
        self.cmd_vel_callback(msg)
        time.sleep(1.0)
        info = self.get_motor_info()
        self.get_logger().info(f"Hover motor speeds: F={info['motor_speeds_rad_s'][0]:.1f}, R={info['motor_speeds_rad_s'][1]:.1f}, B={info['motor_speeds_rad_s'][2]:.1f}, L={info['motor_speeds_rad_s'][3]:.1f} rad/s")
        
        # Climb
        self.get_logger().info("\n2. CLIMB - All motors increase equally")
        msg = Twist()
        msg.linear.z = 1.0
        self.cmd_vel_callback(msg)
        time.sleep(1.0)
        info = self.get_motor_info()
        self.get_logger().info(f"Climb motor speeds: F={info['motor_speeds_rad_s'][0]:.1f}, R={info['motor_speeds_rad_s'][1]:.1f}, B={info['motor_speeds_rad_s'][2]:.1f}, L={info['motor_speeds_rad_s'][3]:.1f} rad/s")
        
        # Roll right
        self.get_logger().info("\n3. ROLL RIGHT - Left motors faster than right motors")
        msg = Twist()
        msg.angular.x = 0.5
        self.cmd_vel_callback(msg)
        time.sleep(1.0)
        info = self.get_motor_info()
        self.get_logger().info(f"Roll right motor speeds: F={info['motor_speeds_rad_s'][0]:.1f}, R={info['motor_speeds_rad_s'][1]:.1f}, B={info['motor_speeds_rad_s'][2]:.1f}, L={info['motor_speeds_rad_s'][3]:.1f} rad/s")
        
        # Pitch forward  
        self.get_logger().info("\n4. PITCH FORWARD - Back motors faster than front motors")
        msg = Twist()
        msg.angular.y = 0.5
        self.cmd_vel_callback(msg)
        time.sleep(1.0)
        info = self.get_motor_info()
        self.get_logger().info(f"Pitch forward motor speeds: F={info['motor_speeds_rad_s'][0]:.1f}, R={info['motor_speeds_rad_s'][1]:.1f}, B={info['motor_speeds_rad_s'][2]:.1f}, L={info['motor_speeds_rad_s'][3]:.1f} rad/s")
        
        # Yaw left
        self.get_logger().info("\n5. YAW LEFT - Alternating motor pattern")
        msg = Twist()
        msg.angular.z = 0.5
        self.cmd_vel_callback(msg)
        time.sleep(1.0)
        info = self.get_motor_info()
        self.get_logger().info(f"Yaw left motor speeds: F={info['motor_speeds_rad_s'][0]:.1f}, R={info['motor_speeds_rad_s'][1]:.1f}, B={info['motor_speeds_rad_s'][2]:.1f}, L={info['motor_speeds_rad_s'][3]:.1f} rad/s")
        
        # Stop
        self.get_logger().info("\n6. STOP - Return to hover")
        self.stop()
        self.get_logger().info("=== DEMONSTRATION COMPLETE ===")

    # ...existing methods...

def main(args=None):
    rclpy.init(args=args)
    
    controller = IgnitionDroneController()
    
    # Make controller functions available globally for interactive use
    globals()['controller'] = controller
    globals()['takeoff'] = controller.takeoff
    globals()['land'] = controller.land
    globals()['stop'] = controller.stop
    globals()['move_forward'] = controller.move_forward
    globals()['move_up'] = controller.move_up
    
    print("\nIgnition Drone Controller Ready!")
    print("Available commands:")
    print("  takeoff(1.0)      - Take off to 1m height")
    print("  move_forward(1.0, 2.0) - Move forward 1 m/s for 2 seconds")
    print("  move_up(0.5, 1.0) - Move up 0.5 m/s for 1 second")
    print("  land()            - Land the drone")
    print("  stop()            - Stop all movement")
    print("\nOr publish to /cmd_vel topic:")
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
