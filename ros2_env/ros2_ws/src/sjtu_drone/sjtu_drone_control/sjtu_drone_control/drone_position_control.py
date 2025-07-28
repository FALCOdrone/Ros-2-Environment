import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Int8
from sjtu_drone_control.drone_utils.drone_object import DroneObject

class DronePositionControl(DroneObject):
    def __init__(self):
        super().__init__('drone_position_control')

        self.TARGET_ALTITUDE = 5.0  # Target altitude for the drone
        self.TARGET_X = 10.0  # Target X coordinate
        self.TARGET_Y = 2.0   # Target Y coordinate
        self.SAFETY_MARGIN = 0.5  # Safety margin for landing
        land_after_moving = True  # Flag to control landing after moving to goal position

        # Wait a bit for sensor data to be available before taking off
        time.sleep(2.0)
        
        self.takeOff()
        self.get_logger().info('Drone takeoff')

        # Set the m_posCtrl flag to True
        self.posCtrl(True)
        self.get_logger().info('Position control mode set to True')

        # Send a command to move the drone to a defined pose
        self.move_drone_to_pose(self.TARGET_X, self.TARGET_Y, self.TARGET_ALTITUDE)
        
        # Use a timer-based approach to make sure the drone is ready for landing
        if land_after_moving:
            self.landing_timer = self.create_timer(5.0, self.check_landing_ready)

    def check_landing_ready(self):
        """Check if drone is ready for landing based on sonar data availability."""
        try:
            # Check if sonar data is available and valid
            if hasattr(self.sonar, 'range') and self.sonar.range > 0:
                error = abs(self.sonar.range - self.TARGET_ALTITUDE)
                self.get_logger().info(f'Sonar data available. Current altitude: {self.sonar.range}, Target: {self.TARGET_ALTITUDE}, Error: {error}')
                
                if self.state == 'Flying' and error <= self.SAFETY_MARGIN:
                    self.get_logger().info('Landing condition met, initiating safe landing...')
                    time.sleep(3.0)
                    self.landing_timer.cancel()
                    self.land_safty_position()
            else:
                self.get_logger().info('Waiting for sonar data...', throttle_duration_sec=2)
        except Exception as e:
            self.get_logger().warn(f'Error checking landing condition: {str(e)}')

    def land_safty_position(self):
        """Land the drone at a safe position based on sonar readings."""
        try:
            self.get_logger().info('Landing at a safe position...')
            
            # Check if sonar data is valid
            if not hasattr(self.sonar, 'range') or self.sonar.range <= 0:
                self.get_logger().warn('Invalid sonar data, landing anyway for safety')
                self.land()
                return
            
            # Check current sonar error
            error = abs(self.sonar.range - self.TARGET_ALTITUDE)
            if error <= self.SAFETY_MARGIN:
                self.get_logger().info('Landing position is safe.')
                self.land()
                return
                
            self.get_logger().warn('Landing position is not safe!')

            # Search around for a safe position
            search_attempts = 0
            max_attempts = 5
            while rclpy.ok() and search_attempts < max_attempts:
                rclpy.spin_once(self, timeout_sec=1.0)
                self.get_logger().info(f'Searching for a safe landing position... (attempt {search_attempts + 1})')
                # TODO: double check if this logic works
                self.move_drone_to_pose(self.TARGET_X + search_attempts, self.TARGET_Y, self.TARGET_ALTITUDE)
                
                # Wait a bit for the drone to move
                import time
                time.sleep(1.0)
                rclpy.spin_once(self, timeout_sec=0.1)
                
                if hasattr(self.sonar, 'range') and self.sonar.range > 0:
                    error = abs(self.sonar.range - self.TARGET_ALTITUDE)
                    self.get_logger().info(f'Current sonar error: {error}')
                    if error <= self.SAFETY_MARGIN:
                        self.get_logger().info('Found a safe landing position.')
                        self.land()
                        return
                        
                search_attempts += 1
            
            # If no safe position found, land anyway for safety
            self.get_logger().warn('No safe landing position found, landing for safety')
            self.land()
            
        except Exception as e:
            self.get_logger().error(f'Error in landing procedure: {str(e)}')
            self.get_logger().info('Emergency landing...')
            self.land()


    def move_drone_to_pose(self, x, y, z):
        # Override the move_drone_to_pose method if specific behavior is needed
        super().moveTo(x, y, z)
        self.get_logger().info(f'Moving drone to pose: x={x}, y={y}, z={z}')


def main(args=None):
    rclpy.init(args=args)
    drone_position_control_node = DronePositionControl()
    rclpy.spin(drone_position_control_node)
    drone_position_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()