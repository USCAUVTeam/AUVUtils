#!/usr/bin/env python3

import rospy
import busio
import numpy as np
import board
from adafruit_pca9685 import PCA9685

from thruster.srv import PWMToI2C, PWMToI2CResponse

voltage = None
pca = None


def microseconds_to_int16(time, freq):
    return time*freq*65536/1000000

def pwm_to_i2c(req):
    global voltage, pca
    try:
        rospy.logdebug("Received pwm signal: %d", req.pwm_val)
        pca.channels[req.channel].duty_cycle = round(microseconds_to_int16(req.pwm_val, req.freq))
        return PWMToI2CResponse(True)
    except Exception as e:
        rospy.logerror("Error of type %s inside of pwm_to_i2c service", e)
        return PWMToI2CResponse(False)

def pwm_to_i2c_server():
    global voltage, pca
    i2c_bus = busio.I2C(board.SCL, board.SDA)

    rospy.init_node("pwm_to_i2c_server", log_level = rospy.DEBUG)
    pca = PCA9685(i2c_bus)
    pca.frequency = rospy.get_param("freq", default=286)
    s = rospy.Service('pwm_to_i2c', PWMToI2C, pwm_to_i2c)
    rospy.loginfo("Successfully created the pwm_to_i2c service")
    rospy.spin()

if __name__ == "__main__":
    pwm_to_i2c_server()