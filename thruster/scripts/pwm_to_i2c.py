#!/usr/bin/env python3

import rospy
import sys
import busio
import numpy as np
import board
from adafruit_pca9685 import PCA9685

from std_msgs.msg import Bool
from thruster.srv import PWMToI2C, PWMToI2CResponse

reset = False
pca = None
i2c_addr = 0

def microseconds_to_int16(time, freq):
    return time*freq*65536/1000000

def pwm_to_i2c(req):
    global pca, reset
    if reset == True:
        return PWMToI2CResponse(False)
    try:
        if(req.channel == 3):
            rospy.logdebug("Received pwm signal: %d", req.pwm_val)
            # rospy.logdebug("Received freq signal: %d", req.freq)
            # rospy.logdebug("Received channel signal: %d", req.channel)
        pca.channels[req.channel].duty_cycle = round(microseconds_to_int16(req.pwm_val, req.freq))
        return PWMToI2CResponse(True)
    except Exception as e:
        rospy.logerror("Error of type %s inside of pwm_to_i2c service", e)
        return PWMToI2CResponse(False)

def reset_pca(data):
    global pca, reset, i2c_addr
    if data.data:
        try:
            pca.reset()
            reset = True
        except Exception as e:
            rospy.logerror("Error of type %s inside of pwm_to_i2c service", e)
            return PWMToI2CResponse(False)
    else:
        reset = False
        i2c_bus = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c_bus, address=int(i2c_addr, 16))
        pca.frequency = rospy.get_param("freq", default=218)

def pwm_to_i2c_server():
    global pca, i2c_addr
    if len(sys.argv) < 2:
        rospy.logerr("Not enough input arguments to create pwm_to_i2c node!")
        raise ValueError("Not enough input arguments to create pwm_to_i2c node!")
    i2c_addr = sys.argv[1]

    i2c_bus = busio.I2C(board.SCL, board.SDA)

    rospy.init_node("pwm_to_i2c_server", log_level = rospy.DEBUG, anonymous=True)
    pca = PCA9685(i2c_bus, address=int(i2c_addr, 16))
    pca.frequency = rospy.get_param("freq", default=218)
    srv_name = "/pwm_to_i2c_" + i2c_addr
    s = rospy.Service(srv_name, PWMToI2C, pwm_to_i2c)
    rospy.loginfo("Successfully created the pwm_to_i2c service")
    rospy.Subscriber("/soft_kill_switch", Bool, reset_pca)
    rospy.spin()

if __name__ == "__main__":
    pwm_to_i2c_server()