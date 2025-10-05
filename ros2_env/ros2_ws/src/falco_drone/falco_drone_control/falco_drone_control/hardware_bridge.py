#!/usr/bin/env python3
"""
Hardware Bridge Node for Teensy 4.1 Communication
Bridges ROS2 simulation commands to real hardware via serial communication
"""

import rclpy
import serial
import struct
import json
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, Wrench
from std_msgs.msg import Empty, Bool
from sensor_msgs.msg import Imu, Range, Barometer
from threading import Thread, Lock
import time

class TeensyHardwareBridge(Node):
    def __init__(self):
        super().__init__('teensy_hardware_bridge')
        
        # Serial connection parameters
        self.serial_port = '/dev/ttyACM0'  # Adjust based on your system
        self.baud_rate = 115200
        self.serial_conn = None
        self.serial_lock = Lock()
        
        # Initialize serial connection
        self.init_serial()
        
        # ROS2 Subscribers (receive commands from real-time controller)
        # Use separate namespace to avoid conflicts with simulation
        self.cmd_sub = self.create_subscription(
            Twist, '/real_drone/cmd_vel', self.cmd_callback, 10)
        self.takeoff_sub = self.create_subscription(
            Empty, '/real_drone/takeoff', self.takeoff_callback, 10)
        self.land_sub = self.create_subscription(
            Empty, '/real_drone/land', self.land_callback, 10)
        self.posctrl_sub = self.create_subscription(
            Bool, '/real_drone/posctrl', self.posctrl_callback, 10)
        self.control_wrench_sub = self.create_subscription(
            Wrench, '/real_drone/control_output', self.control_wrench_callback, 10)
        
        # ROS2 Publishers (publish real sensor data)
        # Use separate namespace to avoid conflicts with simulation
        self.imu_pub = self.create_publisher(Imu, '/real_drone/imu/out', 10) # Real IMU data from hardware
        self.sonar_pub = self.create_publisher(Range, '/real_drone/sonar/out', 10)
        self.barometer_pub = self.create_publisher(Barometer, '/real_drone/barometer/out', 10)
        self.gps_position_pub = self.create_publisher(PointStamped, '/real_drone/gps_position/out', 10)

        # Start serial reading thread
        self.reading_thread = Thread(target=self.serial_reader_thread, daemon=True)
        self.reading_thread.start()
        
        self.get_logger().info('Teensy Hardware Bridge initialized')
    
    def init_serial(self):
        """Initialize serial connection to Teensy"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0,
                write_timeout=1.0
            )
            time.sleep(2.0)  # Allow time for connection
            self.get_logger().info(f'Serial connection established on {self.serial_port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Teensy: {str(e)}')
            self.serial_conn = None
    
    def send_command(self, command_type, data):
        """Send command to Teensy via serial"""
        if not self.serial_conn:
            return
        
        try:
            with self.serial_lock:
                # Create command packet: [START_BYTE, CMD_TYPE, DATA_LENGTH, DATA..., CHECKSUM]
                packet = {
                    'cmd': command_type,
                    'data': data,
                    'timestamp': time.time()
                }
                
                json_data = json.dumps(packet) + '\n'
                self.serial_conn.write(json_data.encode('utf-8'))
                self.serial_conn.flush()
                
        except Exception as e:
            self.get_logger().error(f'Serial write error: {str(e)}')
    
    def cmd_callback(self, msg):
        """Handle movement commands"""
        cmd_data = {
            'linear': {
                'x': msg.linear.x,
                'y': msg.linear.y,
                'z': msg.linear.z
            },
            'angular': {
                'x': msg.angular.x,
                'y': msg.angular.y,
                'z': msg.angular.z
            }
        }
        self.send_command('CMD_VEL', cmd_data)
        self.get_logger().debug(f'Sent movement command: {cmd_data}')
    
    def takeoff_callback(self, msg):
        """Handle takeoff command"""
        self.send_command('TAKEOFF', {})
        self.get_logger().info('Sent takeoff command to Teensy')
    
    def land_callback(self, msg):
        """Handle landing command"""
        self.send_command('LAND', {})
        self.get_logger().info('Sent landing command to Teensy')
    
    def posctrl_callback(self, msg):
        """Handle position control mode change"""
        self.send_command('POS_CTRL', {'enabled': msg.data})
        self.get_logger().info(f'Sent position control: {msg.data}')
    
    def control_wrench_callback(self, msg):
        """Handle control wrench commands from the drone controller"""
        wrench_data = {
            'force': {
                'x': msg.force.x,
                'y': msg.force.y,
                'z': msg.force.z
            },
            'torque': {
                'x': msg.torque.x,
                'y': msg.torque.y,
                'z': msg.torque.z
            }
        }
        self.send_command('CONTROL_WRENCH', wrench_data)
        self.get_logger().debug(f'Sent control wrench: force=[{msg.force.x:.3f}, {msg.force.y:.3f}, {msg.force.z:.3f}], torque=[{msg.torque.x:.3f}, {msg.torque.y:.3f}, {msg.torque.z:.3f}]')
    
    def serial_reader_thread(self):
        """Thread to read sensor data from Teensy"""
        while rclpy.ok():
            if not self.serial_conn:
                time.sleep(1.0)
                continue
            
            try:
                with self.serial_lock:
                    if self.serial_conn.in_waiting > 0:
                        line = self.serial_conn.readline().decode('utf-8').strip()
                        if line:
                            self.process_sensor_data(line)
            except Exception as e:
                self.get_logger().error(f'Serial read error: {str(e)}')
            
            time.sleep(0.01)  # 100Hz reading rate
    
    def process_sensor_data(self, data_line):
        """Process incoming sensor data from Teensy"""
        try:
            data = json.loads(data_line)
            
            if data.get('type') == 'IMU':
                self.publish_imu_data(data['data'])
            elif data.get('type') == 'SONAR':
                self.publish_sonar_data(data['data'])
            elif data.get('type') == 'GPS':
                self.publish_gps_position_data(data['data'])
            elif data.get('type') == 'BAROMETER':
                self.publish_barometer_data(data['data'])
            elif data.get('type') == 'STATUS':
                self.get_logger().info(f"Teensy status: {data['data']}")
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid JSON from Teensy: {data_line}')
        except Exception as e:
            self.get_logger().error(f'Error processing sensor data: {str(e)}')
    
    def publish_imu_data(self, imu_data):
        """Publish IMU data to ROS2"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        # Orientation (quaternion) - from Teensy data structure
        msg.orientation.w = imu_data.get('qw', 1.0)
        msg.orientation.x = imu_data.get('qx', 0.0)
        msg.orientation.y = imu_data.get('qy', 0.0)
        msg.orientation.z = imu_data.get('qz', 0.0)
        
        # Angular velocity (rad/s) - from Teensy angular_velocity structure
        angular_vel = imu_data.get('angular_velocity', {})
        msg.angular_velocity.x = angular_vel.get('x', 0.0)
        msg.angular_velocity.y = angular_vel.get('y', 0.0)
        msg.angular_velocity.z = angular_vel.get('z', 0.0)
        
        # Linear acceleration (m/s²) - from Teensy linear_acceleration structure
        linear_accel = imu_data.get('linear_acceleration', {})
        msg.linear_acceleration.x = linear_accel.get('x', 0.0)
        msg.linear_acceleration.y = linear_accel.get('y', 0.0)
        msg.linear_acceleration.z = linear_accel.get('z', 0.0)
        
        self.imu_pub.publish(msg)
    
    def publish_sonar_data(self, sonar_data):
        """Publish sonar/range data to ROS2"""
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sonar_link'
        msg.radiation_type = Range.ULTRASONIC
        msg.field_of_view = 0.1  # radians
        msg.min_range = 0.02  # meters
        msg.max_range = 4.0   # meters
        msg.range = sonar_data.get('distance', 0.0)
        
        self.sonar_pub.publish(msg)
    
    # Publishing barometer and GPS data
    def publish_barometer_data(self, baro_data):
        """Publish barometer data to ROS2"""
        msg = Barometer()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'barometer_link'
        msg.pressure = baro_data.get('pressure', 0.0)
        msg.altitude = baro_data.get('altitude', 0.0)
        self.barometer_pub.publish(msg)

    def publish_gps_position_data(self, gps_data):
        """Publish ENU position data to ROS2"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'  # ENU world frame
        msg.point.x = gps_data.get('x', 0.0)  # East
        msg.point.y = gps_data.get('y', 0.0)  # North
        msg.point.z = gps_data.get('z', 0.0)  # Up
        
        # Publish the position message
        self.gps_position_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = TeensyHardwareBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()