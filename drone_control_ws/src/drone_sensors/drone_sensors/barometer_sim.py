#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import numpy as np

class BarometerSimulator(Node):
    def __init__(self):
        super().__init__('barometer_simulator')
        
        # Barometer parameters
        self.sea_level_pressure = 1013.25  # hPa
        self.temperature = 15.0  # Celsius at sea level
        self.lapse_rate = 0.0065  # K/m
        
        # Noise parameters
        self.pressure_noise_std = 0.1  # hPa
        self.bias_drift_std = 0.01  # hPa/s
        self.pressure_bias = 0.0
        
        # Current altitude
        self.current_altitude = 0.0
        
        # Publishers
        self.pressure_pub = self.create_publisher(
            Float64,
            '/drone/barometer',
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
        self.timer = self.create_timer(0.1, self.publish_pressure)  # 10 Hz
        
        self.get_logger().info('Barometer Simulator initialized')

    def pose_callback(self, msg):
        """Update current altitude from pose"""
        # Convert from NED to altitude above ground
        self.current_altitude = -msg.pose.position.z

    def altitude_to_pressure(self, altitude):
        """Convert altitude to barometric pressure using standard atmosphere"""
        # Standard atmosphere model
        temp_at_altitude = self.temperature - self.lapse_rate * altitude
        temp_kelvin = temp_at_altitude + 273.15
        temp_sea_level_kelvin = self.temperature + 273.15
        
        # Barometric formula
        pressure = self.sea_level_pressure * \
                   (temp_kelvin / temp_sea_level_kelvin) ** (9.80665 * 0.0289644 / (8.31432 * self.lapse_rate))
        
        return pressure

    def publish_pressure(self):
        """Publish simulated barometric pressure"""
        try:
            # Calculate ideal pressure
            ideal_pressure = self.altitude_to_pressure(self.current_altitude)
            
            # Add noise and bias
            noise = np.random.normal(0, self.pressure_noise_std)
            self.pressure_bias += np.random.normal(0, self.bias_drift_std * 0.1)  # Bias drift
            
            measured_pressure = ideal_pressure + noise + self.pressure_bias
            
            # Publish pressure
            pressure_msg = Float64()
            pressure_msg.data = float(measured_pressure)
            self.pressure_pub.publish(pressure_msg)
            
        except Exception as e:
            self.get_logger().error(f'Barometer simulation error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BarometerSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
