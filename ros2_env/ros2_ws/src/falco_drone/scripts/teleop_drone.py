#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

msg = """
Drone Keyboard Controller
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

space key : force stop
CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),     # forward
    'o': (1, 0, 0, -1),    # forward + turn right
    'j': (0, 0, 0, 1),     # turn left
    'l': (0, 0, 0, -1),    # turn right
    'u': (1, 0, 0, 1),     # forward + turn left
    ',': (-1, 0, 0, 0),    # backward
    '.': (-1, 0, 0, 1),    # backward + turn left
    'm': (-1, 0, 0, -1),   # backward + turn right
    't': (0, 0, 1, 0),     # up
    'b': (0, 0, -1, 0),    # down
    'k': (0, 0, 0, 0),     # stop
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

class TeleopDrone(Node):
    def __init__(self, drone_id=0):
        super().__init__('teleop_drone')
        
        topic_name = f'/quadrotor_{drone_id}/cmd_vel'
        self.publisher = self.create_publisher(Twist, topic_name, 1)
        
        self.speed = 0.5 
        self.turn = 1.0 
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        
        self.get_logger().info(f'Teleoperation started for {topic_name}')

    def getKey(self, settings):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def vels(self, speed, turn):
        return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        
        print(msg)
        print(self.vels(self.speed, self.turn))
        
        try:
            while True:
                key = self.getKey(settings)
                
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0] 
                    self.y = moveBindings[key][1]
                    self.z = moveBindings[key][2]
                    self.th = moveBindings[key][3]
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    print(self.vels(self.speed, self.turn))
                elif key == ' ':
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.th = 0
                elif key == '\x03':  # Ctrl-C
                    break
                else:
                    if key != '':
                        print(f"Key '{key}' not recognized")
                    continue

                # Publish twist message
                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.z = self.th * self.turn
                self.publisher.publish(twist)

        except Exception as e:
            print(e)
        finally:
            # Stop the drone
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main():
    rclpy.init()
    
    drone_id = 0
    if len(sys.argv) > 1:
        try:
            drone_id = int(sys.argv[1])
        except ValueError:
            print("Invalid drone ID. Using default: 0")
    
    teleop = TeleopDrone(drone_id)
    teleop.run()
    
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
