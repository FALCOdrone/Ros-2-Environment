#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Quadrotor!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease linear speed by 10%  
e/c : increase/decrease angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit

Current settings:
Linear speed: 0.5 m/s
Angular speed: 1.0 rad/s

Movement keys:
i - Move forward
, - Move backward  
j - Turn left
l - Turn right
u - Move forward + turn left
o - Move forward + turn right
m - Move backward + turn left
. - Move backward + turn right
t - Move up
b - Move down
"""

moveBindings = {
    'i': (1, 0, 0, 0),     # Move forward
    'o': (1, 0, 0, -1),    # Move forward + turn right
    'j': (0, 0, 0, 1),     # Turn left
    'l': (0, 0, 0, -1),    # Turn right
    'u': (1, 0, 0, 1),     # Move forward + turn left
    ',': (-1, 0, 0, 0),    # Move backward
    '.': (-1, 0, 0, 1),    # Move backward + turn left
    'm': (-1, 0, 0, -1),   # Move backward + turn right
    't': (0, 0, 1, 0),     # Move up
    'b': (0, 0, -1, 0),    # Move down
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

class TeleopQuadrotor(Node):
    def __init__(self):
        super().__init__('teleop_quadrotor')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0
        self.y = 0
        self.z = 0
        self.th = 0
        self.status = 0
        
        # Initialize terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)

    def run(self):
        print(msg)
        print(self.vels(self.speed, self.turn))
        
        try:
            while rclpy.ok():
                key = self.getKey()
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.y = moveBindings[key][1]
                    self.z = moveBindings[key][2]
                    self.th = moveBindings[key][3]
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    print(self.vels(self.speed, self.turn))
                    if (self.status == 14):
                        print(msg)
                    self.status = (self.status + 1) % 15
                elif key == ' ' or key == 'k':
                    self.x = 0
                    self.y = 0
                    self.z = 0
                    self.th = 0
                elif key == '\x03':  # Ctrl-C
                    break
                else:
                    if (key == '\x1b'):  # ESC key
                        break
                    else:
                        self.x = 0
                        self.y = 0
                        self.z = 0
                        self.th = 0

                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.th * self.turn
                self.pub.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    rclpy.init()
    
    teleop_quadrotor = TeleopQuadrotor()
    teleop_quadrotor.run()
    
    teleop_quadrotor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
