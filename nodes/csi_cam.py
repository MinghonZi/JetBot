import atexit

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CSICam(Node):
    def __init__(self) -> None:
        super().__init__("csi_cam")

        self.cam = cv2.VideoCapture(
            "nvarguscamerasrc sensor-id=0 !"
            "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, framerate=(fraction)60/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            , cv2.CAP_GSTREAMER)
        atexit.register(self.cam.release)

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, "/csi_cam", 10)
        self.timer = self.create_timer(0.1, self.publish)

    def publish(self) -> None:
        ret, frame = self.cam.read()
        if ret is not True:
            return
        self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))


def process(args=None):
    rclpy.init(args=args)
    publisher = CSICam()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    process()
