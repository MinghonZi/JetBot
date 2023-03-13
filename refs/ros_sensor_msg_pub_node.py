#!/usr/bin/env python3

from random import random

import rospy
from gazebo_msgs.msg import ModelState, ModelStates


class Ball():

    def __init__(self):
        self.set_model_state = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.rate = rospy.Rate(10)  # 10hz

    def move(self):
        while not rospy.is_shutdown():
            model = rospy.wait_for_message('gazebo/model_states', ModelStates)
            for i in range(len(model.name)):
                if model.name[i] == 'ball':
                    ball = ModelState()
                    ball.model_name = model.name[i]
                    ball.pose = model.pose[i]

                    ball.pose.position.x += (random() - 0.5) * 0.5
                    ball.pose.position.y += (random() - 0.5) * 0.5

                    self.set_model_state.publish(ball)
                    self.rate.sleep()


def main():
    rospy.init_node('move_ball')
    ball = Ball()
    try:
        ball.move()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()

##############################################################################

import board
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag

import numpy as np
import rospy
from sensor_msgs.msg import Imu

# I2C connection:
i2c = board.I2C()
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)

def read_data():
    pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    imu_msg = Imu()
    
    while not rospy.is_shutdown():
        # Read acceleration, magnetometer, gyroscope, temperature.
        accel_x, accel_y, accel_z = accel.acceleration
        mag_x, mag_y, mag_z = mag.magnetic
        
        #Header
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        
        #linear accelerations from accelerometer
        imu_msg.linear_acceleration.x = accel_x
        imu_msg.linear_acceleration.y = accel_y
        imu_msg.linear_acceleration.z = accel_z
        
        # Print values.
        print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
        print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
        
        pub.publish(imu_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        read_data()
    except rospy.ROSInterruptException:
        pass
