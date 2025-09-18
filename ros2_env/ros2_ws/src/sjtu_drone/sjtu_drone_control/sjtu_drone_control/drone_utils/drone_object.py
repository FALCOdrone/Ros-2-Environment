#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Empty, Bool, Int8, String
from geometry_msgs.msg import Twist, Pose, Vector3
from sensor_msgs.msg import Range, Image, Imu
import time

STATES = {
    0: "Landed",
    1: "Flying",
    2: "Taking off",
    3: "Landing",
    4: "Hovering"
}

MODES = ["velocity", "position"]


class DroneObject(Node):
    def __init__(self, node_name: str = "drone_node"):
        super().__init__(node_name)
        self._state = STATES[0] # Default state is landed
        self._mode = MODES[0] # Default mode is velocity
        self._hover_distance = 5.0 # Hover distance from the ground
        self.isFlying = False # Flag to indicate if the drone is flying
        self.isPosctrl = False # Position control mode
        self.isVelMode = False # Velocity control mode
        self.isArming = True  # Flag to indicate if the drone is armed (true by default for simulation)
        self.isTakingOff = False  # Flag to indicate if the drone is taking off

        self.logger = self.get_logger()

        # Define QoS profiles for better reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.pubTakeOff = self.create_publisher(Empty, '/simple_drone/takeoff', qos_profile)
        self.pubLand = self.create_publisher(Empty, '/simple_drone/land', qos_profile)
        self.pubReset = self.create_publisher(Empty, '/simple_drone/reset', qos_profile)
        self.pubPosCtrl = self.create_publisher(Bool, '/simple_drone/posctrl', qos_profile)
        self.pubCmd = self.create_publisher(Twist, '/simple_drone/cmd_vel', qos_profile)
        self.pubVelMode = self.create_publisher(Bool, '/simple_drone/dronevel_mode', qos_profile)

        # Subscribers
        self.sub_sonar = self.create_subscription(Range, '/simple_drone/sonar/out', self.cb_sonar, qos_profile)
        self.sub_imu = self.create_subscription(Imu, '/simple_drone/imu/out', self.cb_imu, qos_profile)
        self.sub_front_img = self.create_subscription(Image, '/simple_drone/front/image_raw',
                                                      self.cb_front_img, qos_profile)
        self.sub_bottom_img = self.create_subscription(Image, '/simple_drone/bottom/image_raw',
                                                       self.cb_bottom_img, qos_profile)
        self.sub_gt_pose = self.create_subscription(Pose, '/simple_drone/gt_pose', self.cb_gt_pose, qos_profile)
        self.sub_state = self.create_subscription(Int8, '/simple_drone/state', self.cb_state, qos_profile)
        self.sub_cmd_mode = self.create_subscription(String, '/simple_drone/cmd_mode', self.cb_cmd_mode, qos_profile)
        self.sub_emergency_land = self.create_subscription(Bool, '/simple_drone/emergency_land',
                                                          self.emergency_land, qos_profile)

        # subscribe to the starting button topic for taking off the drone -> for now publish the starting button
        # use `ros2 topic pub /start_button std_msgs/msg/Int8 "data: 1"` to take off the drone
        self.start_button_subscriber = self.create_subscription(
            Int8,
            '/start_button',
            self.start_button_callback,
            qos_profile
        )
        # subscribe to armed boolean topic to monitor arming status
        self.armed_subscriber = self.create_subscription(
            Bool,
            '/is_armed',
            self.armed_callback,
            qos_profile
        )

        self._sonar = Range()
        self._imu = Imu()
        self._front_img = Image()
        self._bottom_img = Image()
        self._gt_pose = Pose()

        while self.pubTakeOff.get_subscription_count() == 0:
            self.logger.info("Waiting for drone to spawn", throttle_duration_sec=1)

    @property
    def state(self):
        return self._state

    @property
    def mode(self):
        return self._mode

    @property
    def hover_distance(self):
        return self._hover_distance

    @property
    def sonar(self):
        return self._sonar

    @property
    def imu(self):
        return self._imu

    @property
    def front_img(self):
        return self._front_img

    @property
    def bottom_img(self):
        return self._bottom_img

    @property
    def gt_pose(self):
        return self._gt_pose

    @state.setter
    def state(self, value):
        self._state = value

    @mode.setter
    def mode(self, value):
        self._mode = value

    @hover_distance.setter
    def hover_distance(self, value):
        self._hover_distance = value

    @sonar.setter
    def sonar(self, value):
        self._sonar = value

    @imu.setter
    def imu(self, value):
        self._imu = value

    @front_img.setter
    def front_img(self, value):
        self._front_img = value

    @bottom_img.setter
    def bottom_img(self, value):
        self._bottom_img = value

    @gt_pose.setter
    def gt_pose(self, value):
        self._gt_pose = value

    def takeOff(self):
        """
        Take off the drone
        :return: True if the command was sent successfully, False if drone is already flying
        """
        try:
            #if self.isFlying:
            #    return False
            self.logger.info("Taking off")
            empty_msg = Empty()
            self.pubTakeOff.publish(empty_msg)
            self.isFlying = True
            return True
        except Exception as e:
            self.logger.error(f"Error in takeOff method: {str(e)}")
            return False
    
    def check_hover(self):
        """
        Check if the drone is hovering
        :return: True if the drone is hovering, False otherwise.
        TODO:Implement the same function also with the barometer data.
        """
        # Check if we have valid sonar data
        if not hasattr(self._sonar, 'range') or self._sonar.range <= 0:
            self.logger.warn("No valid sonar data for hover check")
            return False
            
        sonar_error = abs(self._sonar.range - self._hover_distance)
        self.logger.info(f"Sonar reading: {self._sonar.range:.2f}m, Hover distance: {self._hover_distance:.2f}m, Error: {sonar_error:.2f}m", throttle_duration_sec=2)

        # Check if drone is flying and altitude is stable (increased tolerance)
        if self._state == 'Flying' and sonar_error < 0.5:  # More realistic tolerance
            self.logger.info("Drone is hovering stably.")
            self._state = 'Hovering'  # Update state to Hovering
            self.isTakingOff = False  # Reset the takeoff flag
        elif sonar_error >= 0.5:
            self.logger.info("Drone altitude is not stable yet.")

    def land(self):
        """
        Land the drone
        :return: True if the command was sent successfully, False if drone is not flying
        """
        try:
            if not self.isFlying:
                return False
            self.logger.info("Landing")
            empty_msg = Empty()
            self.pubLand.publish(empty_msg)
            self.isFlying = False
            return True
        except Exception as e:
            self.logger.error(f"Error in land method: {str(e)}")
            return False

    def hover(self):
        """
        Hover the drone
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def posCtrl(self, flag: bool):
        """
        Enable or disable position control mode
        :param flag: True to enable position control, False to disable
        :return: True if the command was sent successfully
        """
        try:
            self.isPosctrl = flag
            bool_msg = Bool()
            bool_msg.data = flag
            self.pubPosCtrl.publish(bool_msg)
            return True
        except Exception as e:
            self.logger.error(f"Error in posCtrl method: {str(e)}")
            return False

    def velMode(self, on):
        """
        Turn on/off velocity control mode
        :param on: True to turn on velocity control mode, False to turn off
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        self.isVelMode = on
        bool_msg = Bool()
        bool_msg.data = on
        self.pubVelMode.publish(bool_msg)
        return True

    def move(self, v_linear: Vector3 = Vector3(),
             v_angular: Vector3 = Vector3()):
        """
        Move the drone using velocity control along the linear x and z axis and rotation around
        the x, y and z axis
        :param v_linear: Linear velocity in m/s
        :param v_angular: Angular velocity in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist(linear=v_linear, angular=v_angular)
        self.pubCmd.publish(twist_msg)
        return True

    def moveTo(self, x: float, y: float, z: float):
        """
        Move the drone to a specific position
        :param x: X position in m
        :param y: Y position in m
        :param z: Z position in m
        :return: True if the command was sent successfully, False if drone is not flying
        """
        try:
            if not self.isFlying:
                return False
            twist_msg = Twist()
            twist_msg.linear.x = float(x)
            twist_msg.linear.y = float(y)
            twist_msg.linear.z = float(z)
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0
            self.pubCmd.publish(twist_msg)
            return True
        except Exception as e:
            self.logger.error(f"Error in moveTo method: {str(e)}")
            return False

    def pitch(self, speed):
        """
        Pitch the drone
        :param speed: Pitch speed in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 1.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = speed
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def roll(self, speed: float):
        """
        Roll the drone
        :param speed: Roll speed in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 1.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = speed
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def rise(self, speed: float):
        """
        Rise or fall the drone
        :param speed: Rise speed in m/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = speed
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.pubCmd.publish(twist_msg)
        return True

    def yaw(self, speed: float):
        """
        Rotate the drone around the z-axis
        :param speed: Rotation speed in rad/s
        :return: True if the command was sent successfully, False if drone is not flying
        """
        if not self.isFlying:
            return False
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = speed
        self.pubCmd.publish(twist_msg)
        return True
    
    def armed_callback(self, msg):
        try:
            #self.isArming = msg.data
            if self.isArming:
                self.get_logger().info('Drone is armed and ready for takeoff.')
            else:
                self.get_logger().info('Drone is disarmed.')
        except Exception as e:
            self.get_logger().error(f"Error in armed callback: {str(e)}")

    def start_button_callback(self, msg):
        try:
            if msg.data == 1 and self.isArming:
                self.get_logger().info('Button pressed, initiating takeoff...')
                time.sleep(3.0)
                self.isTakingOff = True
        except Exception as e:
            self.get_logger().error(f"Error in start button callback: {str(e)}")

    def emergency_land(self, msg: Bool):
        try:
            if self.isFlying and msg.data:
                self.get_logger().warn('Emergency landing initiated!')
                time.sleep(1.0)
                self.land()
            else:
                self.get_logger().info('Drone is already landed.')
        except Exception as e:
            self.get_logger().error(f"Error in emergency land callback: {str(e)}")

    def cb_sonar(self, msg: Range):
        """Callback for the sonar sensor"""
        try:
            self._sonar = msg
            #self._hover_distance = msg.min_range
        except Exception as e:
            self.logger.error(f"Error in sonar callback: {str(e)}")

    def cb_imu(self, msg: Imu):
        """Callback for the imu sensor"""
        try:
            self._imu = msg
        except Exception as e:
            self.logger.error(f"Error in IMU callback: {str(e)}")

    def cb_front_img(self, msg: Image):
        """Callback for the front camera"""
        try:
            self._front_img = msg
        except Exception as e:
            self.logger.error(f"Error in front image callback: {str(e)}")

    def cb_bottom_img(self, msg: Image):
        """Callback for the rear camera"""
        try:
            self._bottom_img = msg
        except Exception as e:
            self.logger.error(f"Error in bottom image callback: {str(e)}")

    def cb_gt_pose(self, msg: Pose) -> None:
        """Callback for the ground truth pose"""
        try:
            self._gt_pose = msg
        except Exception as e:
            self.logger.error(f"Error in ground truth pose callback: {str(e)}")

    def cb_state(self, msg: Int8):
        """Callback for the drone state"""
        try:
            if msg.data in STATES:
                self._state = STATES[msg.data]
                self.logger.info("State: {}".format(self._state), throttle_duration_sec=1)
            else:
                self.logger.error(f"Invalid state received: {msg.data}")
                # Keep current state if invalid
        except Exception as e:
            self.logger.error(f"Error in state callback: {str(e)}")

    def cb_cmd_mode(self, msg: String):
        """Callback for the command mode"""
        try:
            if msg.data in MODES:
                self._mode = msg.data
                self.logger.info("Changed command mode to: {}".format(self._mode))
            else:
                self.logger.error("Invalid command mode: {}".format(msg.data))
        except Exception as e:
            self.logger.error(f"Error in command mode callback: {str(e)}")

    def reset(self):
        try:
            empty_msg = Empty()
            self.pubReset.publish(empty_msg)
        except Exception as e:
            self.logger.error(f"Error in reset method: {str(e)}")
