"""
Foxglove Teleop panel settings
- Publish rate: 5.00
- Topic: /cmd_vel
- Up   : linear-x   0.56
- Down : linear-x  -0.56
- Left : angular-z  8.00
- Right: angular-z -8.00
Refer to https://foxglove.dev/docs/studio/panels/teleop

Alternatives
- ROS1: https://wiki.ros.org/diff_drive_controller
- ROS2: https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html
"""

from math import pi

import numpy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# https://adafruit.com/product/2927
# https://adafruit.com/product/3777
from adafruit_motorkit import MotorKit
_kit = MotorKit()
lmotor = _kit.motor1
rmotor = _kit.motor2
MAX_RPM = 180  # Roughly measured
MAX_MAV = MAX_RPM * 2 * pi / 60  # Max motor angular velocity (rad/s)
MIN_MAV = -MAX_MAV  # Min motor angular velocity (rad/s)


class Controller(Node):
    def __init__(self) -> None:
        super().__init__("controller")

        self.declare_parameters(
            namespace='',
            parameters=[
                ("wheel_separation", 0.1),  # meter
                ("wheel_radius", 0.03),  # meter
                ("cmd_vel_timeout", 0.3)])  # second

        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.wheel_radius = self.get_parameter("wheel_radius").value

        cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value
        self.cmd_vel_timeout_timer = self.create_timer(cmd_vel_timeout, self.brake)

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel, 10)

    def cmd_vel(self, msg: Twist):
        self.cmd_vel_timeout_timer.reset()

        # https://answers.ros.org/question/185427/make-sure-geometry_msgstwist-or-cmd_vel-units/
        𝑣 = msg.linear.x  # Max 0.56 m/s (3 revolutions of the motor)
        𝜔 = msg.angular.z  # rad/s
        𝑟 = self.wheel_radius
        𝑏 = self.wheel_separation

        # https://en.wikipedia.org/wiki/Differential_wheeled_robot#Kinematics_of_Differential_Drive_Robots
        𝜔ᴸ = (𝑣 - 𝜔 * 𝑏 / 2) / 𝑟
        𝜔ᴿ = (𝑣 + 𝜔 * 𝑏 / 2) / 𝑟

        𝜔ᴸ = numpy.clip(𝜔ᴸ, MIN_MAV, MAX_MAV)
        𝜔ᴿ = numpy.clip(𝜔ᴿ, MIN_MAV, MAX_MAV)

        # Normalise to [-1, 1] range
        # https://en.wikipedia.org/wiki/Normalization_(statistics)
        # https://en.wikipedia.org/wiki/Feature_scaling
        # https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
        𝜔ᴸ =  (𝜔ᴸ - MIN_MAV) * (1 - -1) / (MAX_MAV - MIN_MAV) + -1
        𝜔ᴿ =  (𝜔ᴿ - MIN_MAV) * (1 - -1) / (MAX_MAV - MIN_MAV) + -1

        # FIXME: These are very basic motors, and have no built-inencoders,speed
        # control or positional feedback. Voltage goes in, rotation goes out!
        lmotor.throttle = 𝜔ᴸ
        rmotor.throttle = -𝜔ᴿ

    def brake(self):
        lmotor.throttle = 0.0
        rmotor.throttle = 0.0

def process(args=None):
    rclpy.init(args=args)
    teleop = Controller()
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    process()
