import atexit
from typing import Optional

from adafruit_motorkit import MotorKit

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
