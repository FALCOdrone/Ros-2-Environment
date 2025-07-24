#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import sys

class DroneController(Node):
    def __init__(self, drone_id=0):
        super().__init__('drone_controller')
        
        # Create publisher for drone velocity commands
        topic_name = f'/quadrotor_{drone_id}/cmd_vel'
        self.publisher = self.create_publisher(Twist, topic_name, 10)
        
        self.get_logger().info(f'Drone Controller started for {topic_name}')
        self.get_logger().info('Use the following commands:')
        self.get_logger().info('  takeoff()     - Take off to 1m height')
        self.get_logger().info('  land()        - Land the drone')
        self.get_logger().info('  move_forward(speed, duration)')
        self.get_logger().info('  move_back(speed, duration)') 
        self.get_logger().info('  move_left(speed, duration)')
        self.get_logger().info('  move_right(speed, duration)')
        self.get_logger().info('  move_up(speed, duration)')
        self.get_logger().info('  move_down(speed, duration)')
        self.get_logger().info('  rotate_left(speed, duration)')
        self.get_logger().info('  rotate_right(speed, duration)')
        self.get_logger().info('  stop()        - Stop all movement')
        
    def send_velocity(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_z=0.0):
        """Send velocity command to the drone"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y  
        msg.linear.z = linear_z
        msg.angular.z = angular_z
        self.publisher.publish(msg)
        
    def takeoff(self, height=1.0):
        """Takeoff to specified height"""
        self.get_logger().info(f'Taking off to {height}m...')
        # Send upward velocity for calculated time
        duration = height / 0.5  # Assume 0.5 m/s upward speed
        self.move_up(0.5, duration)
        
    def land(self):
        """Land the drone"""
        self.get_logger().info('Landing...')
        self.move_down(0.5, 3.0)  # Move down for 3 seconds
        
    def move_forward(self, speed=1.0, duration=1.0):
        """Move forward for specified duration"""
        self.get_logger().info(f'Moving forward at {speed} m/s for {duration} seconds')
        self._timed_movement(speed, 0.0, 0.0, 0.0, duration)
        
    def move_back(self, speed=1.0, duration=1.0):
        """Move backward for specified duration"""
        self.get_logger().info(f'Moving backward at {speed} m/s for {duration} seconds')
        self._timed_movement(-speed, 0.0, 0.0, 0.0, duration)
        
    def move_left(self, speed=1.0, duration=1.0):
        """Move left for specified duration"""
        self.get_logger().info(f'Moving left at {speed} m/s for {duration} seconds')
        self._timed_movement(0.0, speed, 0.0, 0.0, duration)
        
    def move_right(self, speed=1.0, duration=1.0):
        """Move right for specified duration"""
        self.get_logger().info(f'Moving right at {speed} m/s for {duration} seconds')
        self._timed_movement(0.0, -speed, 0.0, 0.0, duration)
        
    def move_up(self, speed=1.0, duration=1.0):
        """Move up for specified duration"""
        self.get_logger().info(f'Moving up at {speed} m/s for {duration} seconds')
        self._timed_movement(0.0, 0.0, speed, 0.0, duration)
        
    def move_down(self, speed=1.0, duration=1.0):
        """Move down for specified duration"""
        self.get_logger().info(f'Moving down at {speed} m/s for {duration} seconds')
        self._timed_movement(0.0, 0.0, -speed, 0.0, duration)
        
    def rotate_left(self, speed=1.0, duration=1.0):
        """Rotate left for specified duration"""
        self.get_logger().info(f'Rotating left at {speed} rad/s for {duration} seconds')
        self._timed_movement(0.0, 0.0, 0.0, speed, duration)
        
    def rotate_right(self, speed=1.0, duration=1.0):
        """Rotate right for specified duration"""
        self.get_logger().info(f'Rotating right at {speed} rad/s for {duration} seconds')
        self._timed_movement(0.0, 0.0, 0.0, -speed, duration)
        
    def stop(self):
        """Stop all movement"""
        self.get_logger().info('Stopping...')
        self.send_velocity(0.0, 0.0, 0.0, 0.0)
        
    def _timed_movement(self, linear_x, linear_y, linear_z, angular_z, duration):
        """Execute movement for a specific duration"""
        import time
        start_time = time.time()
        
        # Send commands at 10Hz
        while (time.time() - start_time) < duration:
            self.send_velocity(linear_x, linear_y, linear_z, angular_z)
            time.sleep(0.1)
            
        # Stop after duration
        self.stop()

def main(args=None):
    rclpy.init(args=args)
    
    # Get drone ID from command line arguments
    drone_id = 0
    if len(sys.argv) > 1:
        try:
            drone_id = int(sys.argv[1])
        except ValueError:
            print("Invalid drone ID. Using default: 0")
    
    controller = DroneController(drone_id)
    
    # Make controller available in global namespace for interactive use
    globals()['controller'] = controller
    globals()['takeoff'] = controller.takeoff 
    globals()['land'] = controller.land
    globals()['move_forward'] = controller.move_forward
    globals()['move_back'] = controller.move_back
    globals()['move_left'] = controller.move_left
    globals()['move_right'] = controller.move_right
    globals()['move_up'] = controller.move_up
    globals()['move_down'] = controller.move_down
    globals()['rotate_left'] = controller.rotate_left
    globals()['rotate_right'] = controller.rotate_right
    globals()['stop'] = controller.stop
    
    print("Drone controller ready! You can now use commands like:")
    print("  takeoff()")
    print("  move_forward(1.0, 2.0)  # speed=1.0 m/s, duration=2.0 seconds")
    print("  rotate_left(0.5, 1.0)   # speed=0.5 rad/s, duration=1.0 seconds")
    print("  stop()")
    print("  land()")
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
