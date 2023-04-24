import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleOp(Node):
    def __init__(self) -> None:
        super().__init__("teleop")
        self.twist_sub = self.create_subscription(
            Twist,
            "foxglove_teleop",
            self.callback,
            10,
        )

    def callback(self, msg: Twist):
        """ TODO
        - https://foxglove.dev/docs/studio/panels/teleop
        - https://wiki.ros.org/diff_drive_controller
        """

        # print(msg.linear.x, msg.linear.y, msg.linear.z)
        # print(msg.angular.x, msg.angular.y, msg.angular.z)


def process(args=None):
    rclpy.init(args=args)
    teleop = TeleOp()
    rclpy.spin(teleop)
    teleop.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    process()
