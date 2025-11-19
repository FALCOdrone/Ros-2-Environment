#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MotorPublisher(Node):
    def __init__(self):
        super().__init__('motor_pub')
        self.motors = [
            self.create_publisher(Float64, '/world/default/model/x500_enhanced/joint/motor1/effort_cmd', 10),
            self.create_publisher(Float64, '/world/default/model/x500_enhanced/joint/motor2/effort_cmd', 10),
            self.create_publisher(Float64, '/world/default/model/x500_enhanced/joint/motor3/effort_cmd', 10),
            self.create_publisher(Float64, '/world/default/model/x500_enhanced/joint/motor4/effort_cmd', 10),
        ]
        self.timer = self.create_timer(0.02, self.publish)
        self.get_logger().info("Motor publisher started.")

    def publish(self):
        # Example hover values
        hover_thrust = 2.5
        for i, pub in enumerate(self.motors):
            msg = Float64()
            msg.data = hover_thrust  # Replace with control logic
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
