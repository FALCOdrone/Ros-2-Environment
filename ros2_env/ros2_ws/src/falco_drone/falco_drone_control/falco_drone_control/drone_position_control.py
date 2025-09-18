import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Int8, Bool
from falco_drone_control.drone_utils.drone_object import DroneObject

class DronePositionControl(DroneObject):
    def __init__(self):
        super().__init__('drone_position_control')

        self.TARGET_ALTITUDE = self.hover_distance  # Target altitude for the drone
        self.TARGET_X = 10.0  # Target X coordinate
        self.TARGET_Y = 2.0   # Target Y coordinate
        self.SAFETY_MARGIN = 0.5  # Safety margin for landing
        self.land_after_moving = False  # Flag to control landing after moving to goal position
        self.initialization_complete = False
        self.init_start_time = None
        self.takeoff_time = None
        self.landing_initiated = False
        self.landing_in_progress = False

        # Create a timer to handle the initialization and main logic
        self.init_timer = self.create_timer(0.1, self.initialization_callback)

    def initialization_callback(self):
        """Handle initialization and main logic in a timer callback to avoid spin conflicts"""
        try:
            if not self.initialization_complete:
                # Wait a bit for sensor data to be available before taking off
                if self.init_start_time is None:
                    self.init_start_time = time.time()
                    return
                
                if time.time() - self.init_start_time < 2.0:
                    return

                # taking off and hovering 
                if self.isArming:

                    # Wait for start button to be pressed
                    if not self.isTakingOff:
                        self.get_logger().info('Waiting for start button press...', throttle_duration_sec=5)
                        return

                    # Execute takeoff command only once
                    if self.takeoff_time is None:
                        self.takeOff()
                        self.takeoff_time = time.time()
                        return
                    
                    if time.time() - self.takeoff_time < 3.0:
                        return
                    
                    # Update hover distance to actual current altitude
                    if hasattr(self._sonar, 'range') and self._sonar.range > 0:
                        self._hover_distance = self._sonar.range
                        self.get_logger().info(f'Updated hover distance to actual altitude: {self._hover_distance:.2f}m')
                    else:
                        self.get_logger().warn('No sonar data available, keeping default hover distance')

                    # Set the m_posCtrl flag to True
                    self.posCtrl(True)
                    self.get_logger().info('Position control mode set to True')

                    # Send a command to move the drone to a defined pose
                    self.move_drone_to_pose(0.0, 0.0, self.TARGET_ALTITUDE)

                    # Mark initialization as complete
                    self.initialization_complete = True
                    
                    # Cancel the initialization timer
                    self.init_timer.cancel()
                    
                    # Create a timer for periodic checks if landing is needed
                    if self.land_after_moving:
                        self.landing_timer = self.create_timer(5.0, self.check_landing_ready)

                else:
                    self.get_logger().error('Drone not ready for takeoff')
                    self.initialization_complete = True
                    if hasattr(self, 'init_timer'):
                        self.init_timer.cancel()
                    
        except Exception as e:
            self.get_logger().error(f'Error in initialization: {str(e)}')
            self.initialization_complete = True
            if hasattr(self, 'init_timer'):
                self.init_timer.cancel()

    def check_landing_ready(self):
        """Check if drone is ready for landing based on sonar data availability."""
        try:
            # Check if sonar data is available and valid
            if hasattr(self._sonar, 'range') and self._sonar.range > 0:
                error = abs(self._sonar.range - self.TARGET_ALTITUDE)
                self.get_logger().info(f'Sonar data available. Current altitude: {self._sonar.range}, Target: {self.TARGET_ALTITUDE}, Error: {error}')
                
                if self.state == 'Flying' and error <= self.SAFETY_MARGIN:
                    # Check if landing position is safe BEFORE initiating landing
                    self.get_logger().info('Landing condition met, checking if position is safe...')
                    
                    # Do the safety check while still flying/hovering
                    if self.is_landing_position_safe():
                        self.get_logger().info('Landing position is safe, initiating landing...')
                        # Cancel the timer first to prevent multiple calls
                        try:
                            self.get_logger().info('Cancelling landing timer to prevent multiple calls...')
                            self.landing_timer.cancel()
                        except:
                            pass
                        # Call landing directly but with a flag to prevent multiple calls
                        if not self.landing_initiated:
                            self.get_logger().info('Setting landing initiated flag and calling land_now()...')
                            self.landing_initiated = True
                            self.land_now()
                    else:
                        self.get_logger().warn('Landing position is not safe, continuing to monitor...')
            else:
                self.get_logger().info('Waiting for sonar data...', throttle_duration_sec=2)
        except Exception as e:
            self.get_logger().warn(f'Error checking landing condition: {str(e)}')

    def is_landing_position_safe(self):
        """Check if the current position is safe for landing while drone is still flying"""
        try:
            # Check if sonar data is valid
            if not hasattr(self._sonar, 'range') or self._sonar.range <= 0:
                self.get_logger().warn('Invalid sonar data for safety check')
                return False
            
            # Check current sonar error while hovering at target altitude
            error = abs(self._sonar.range - self.TARGET_ALTITUDE)
            return error <= self.SAFETY_MARGIN
            
        except Exception as e:
            self.get_logger().error(f'Error checking landing safety: {str(e)}')
            return False

    def land_now(self):
        """Directly land the drone without additional safety checks (safety already verified)"""
        try:
            # Prevent multiple calls
            if hasattr(self, 'landing_in_progress') and self.landing_in_progress:
                return
            
            self.landing_in_progress = True
            self.get_logger().info('Landing at confirmed safe position...')
            
            # Cancel any remaining timers first to prevent callback conflicts
            if hasattr(self, 'landing_timer'):
                try:
                    self.landing_timer.cancel()
                except:
                    pass
            
            # Land directly since position safety was already confirmed
            self.land()
            
        except Exception as e:
            self.get_logger().error(f'Error during landing: {str(e)}')
            self.get_logger().info('Emergency landing...')
            try:
                self.land()
            except Exception as land_error:
                self.get_logger().error(f'Error during emergency landing: {str(land_error)}')
        finally:
            # Reset the flag
            self.landing_in_progress = False

    def land_safety_position(self):
        """Legacy method - now redirects to new safer landing approach"""
        self.get_logger().warn('Using legacy land_safety_position method - consider updating code')
        self.land_now()

    def move_drone_to_pose(self, x, y, z):
        """Move drone to specified pose"""
        try:
            # Override the move_drone_to_pose method if specific behavior is needed
            super().moveTo(x, y, z)
            self.get_logger().info(f'Moving drone to pose: x={x}, y={y}, z={z}')
        except Exception as e:
            self.get_logger().error(f'Error moving drone: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    drone_position_control_node = None
    try:
        drone_position_control_node = DronePositionControl()
        rclpy.spin(drone_position_control_node)
    except KeyboardInterrupt:
        print("Program interrupted by user")
    except Exception as e:
        print(f"Error in main: {str(e)}")
        # Try to emergency land if possible
        if drone_position_control_node is not None:
            try:
                drone_position_control_node.get_logger().error(f"Emergency shutdown due to error: {str(e)}")
                if hasattr(drone_position_control_node, 'isFlying') and drone_position_control_node.isFlying:
                    drone_position_control_node.land()
            except:
                pass
    finally:
        if drone_position_control_node is not None:
            try:
                # Cancel all timers
                if hasattr(drone_position_control_node, 'init_timer'):
                    drone_position_control_node.init_timer.cancel()
                if hasattr(drone_position_control_node, 'landing_timer'):
                    drone_position_control_node.landing_timer.cancel()
                drone_position_control_node.destroy_node()
            except:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()