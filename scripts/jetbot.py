import atexit
from typing import Optional

import cv2
from adafruit_motorkit import MotorKit

cam = cv2.VideoCapture(
    "nvarguscamerasrc sensor-id=0 !"
    "video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, framerate=(fraction)30/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=(string)BGR ! appsink"
    , cv2.CAP_GSTREAMER)

_kit = MotorKit()

lmotor = _kit.motor1
rmotor = _kit.motor2

def rectilinear(speed: Optional[float]):
    lmotor.throttle = speed
    rmotor.throttle = -speed

def rotate(speed: Optional[float]):
    lmotor.throttle = speed
    rmotor.throttle = speed

def pivot_left(speed: Optional[float]):
    lmotor.throttle = 0.0
    rmotor.throttle = -speed

def pivot_right(speed: Optional[float]):
    lmotor.throttle = speed
    rmotor.throttle = 0.0

def brake():
    lmotor.throttle = 0.0
    rmotor.throttle = 0.0

@atexit.register
def _motors_off():
    lmotor.throttle = None
    rmotor.throttle = None
