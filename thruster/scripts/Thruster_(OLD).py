#!/usr/bin/env python

'''
This code was used for the raspberry pi and connects to the PCA boards directly rather than
using the ESP8266
'''
import rospy
from thruster.msg import FloatStamped
from scipy import interpolate
import numpy as np
import rospkg
import pandas
from thruster.srv import *
from std_msgs.msg import Bool, Float64
from time import sleep

class Thruster():
    '''
    With these thrusters, I want the input to be how much thrust (kg f) the user wants and
    the output to depend whether the Thruster is used in the sim (output will be a FloatStamped)
    or in real life (output will be a PWM or I2C signal).
    '''
    def __init__(self, saturation=3, voltage=12, thruster_num = 0, is_reverse=False, output="sim", freq=218, pca_name="/pwm_to_i2c"):
        '''
        Parameters
        -----
        saturation : int/float
            The max thrust of the motor
        voltage : int
            How many volts is supplied to the 
        thruster_num : int
            What number thurster this is
        output : "sim" or "real"
            The type of output you want
        '''
        
        self.VALID_VOLTAGES = [10, 12, 14, 16, 18, 20]
        self.saturation = saturation
        self.current_thrust = 0
        self.output = output
        self.pwm = 1500
        self.is_reverse = is_reverse
        self.freq = freq
        self.pca_name = pca_name
        self.duty_cycle = None
        self.pub = None
        self.running = True
        self.thruster_num = thruster_num
        self.channel_num = 0
        self.pwm_to_i2c = None
        # rospy.init_node("thruster_" + str(self.thruster_num))
        self.min_voltage = rospy.get_param("/min_voltage", default=10)
        self.max_voltage = rospy.get_param("/max_voltage", default=20)
        self.voltage=self.min_voltage
        self.update_voltage(voltage)
        self.namespace = rospy.get_param("/namespace", default="manataray")
        self.float_frame = rospy.get_param("/float_frame", "")

        # self.test_pub = rospy.Publisher('/mantaray/i2c_pwm_val', Int64, queue_size=10)

        if self.output == "sim":
            # Publishes the output to the simulation mantaray
            self.pub = rospy.Publisher('/' + self.namespace + '/thrusters/'+str(self.thruster_num)+'/input', FloatStamped, queue_size = 10)

        r = rospkg.RosPack()
        dir = r.get_path('thruster')
        self.data_file = dir + "/data/T200 Data.csv"
        self.data = [pandas.read_csv(self.data_file,  header=0, usecols=["x1neg", "y1neg"]),
                     pandas.read_csv(self.data_file,  header=0, usecols=["x1pos", "y1pos"]),
                     pandas.read_csv(self.data_file,  header=0, usecols=["x2neg", "y2neg"]),
                     pandas.read_csv(self.data_file,  header=0, usecols=["x2pos", "y2pos"]),
                     pandas.read_csv(self.data_file,  header=0, usecols=["x3neg", "y3neg"]),
                     pandas.read_csv(self.data_file,  header=0, usecols=["x3pos", "y3pos"]),
                     pandas.read_csv(self.data_file,  header=0, usecols=["x4neg", "y4neg"]),
                     pandas.read_csv(self.data_file,  header=0, usecols=["x4pos", "y4pos"]),
                     pandas.read_csv(self.data_file,  header=0, usecols=["x5neg", "y5neg"]),
                     pandas.read_csv(self.data_file,  header=0, usecols=["x5pos", "y5pos"]),
                     pandas.read_csv(self.data_file,  header=0, usecols=["x6neg", "y6neg"]),
                     pandas.read_csv(self.data_file,  header=0, usecols=["x6pos", "y6pos"])]
        self.interp_functions = list()
        for i in range(6):
            x1 = self.data[2*i]['x{}neg'.format(i+1)].array
            y1 = self.data[2*i]['y{}neg'.format(i+1)].array
            x2 = self.data[2*i+1]['x{}pos'.format(i+1)].array
            y2 = self.data[2*i+1]['y{}pos'.format(i+1)].array
            self.interp_functions.append([interpolate.interp1d(x1[~np.isnan(x1)], y1[~np.isnan(y1)], kind='cubic', bounds_error=False, fill_value=1500), 
                                interpolate.interp1d(x2[~np.isnan(x2)], y2[~np.isnan(y2)], kind='cubic', bounds_error=False, fill_value=1500)]) 
        self.set_thrust(self.current_thrust)

    def kill_thruster(self):
        self.running = False
        self.set_thrust(0)
    
    def enable_thruster(self):
        self.running = True

    #freq of cycle (hz)
    #returns a 16 bit number
    def microseconds_to_int16(self, time):
        return time*self.freq*65536/1000000
    
    def get_duty_cycle(self):
        return self.duty_cycle

    def get_thrust(self):
        return self.current_thrust
    
    def add_thrust(self, thrust):
        self.set_thrust(self.current_thrust+thrust)

    def set_thrust(self, thrust):
        if self.running:
            self.current_thrust = max(min(thrust, self.saturation), -self.saturation)
        else:
            self.current_thrust = 0
        if self.output == "sim":
            self.update_pwm()
        else:
            self.update_pwm()
            self.send_i2c()

    def update_voltage(self, volt):
        # Use in case of dynamic voltage. Can be useful for long-term operations
        for i in range(len(self.VALID_VOLTAGES)):
            if self.VALID_VOLTAGES[i]-int(volt) > -1:
                self.min_voltage = self.VALID_VOLTAGES[i]
                self.max_voltage = self.VALID_VOLTAGES[i+1]
                self.voltage == volt
                return
        rospy.logwarn("Voltage %d V is not a valid voltage for this thruster!", volt)

    def update_pwm(self):
        # This calculates the approximate PWM signal necessary to get the needed thrust
        if not self.running:
            self.current_thrust = 0
        if self.current_thrust <= 0 and self.is_reverse:
            is_positive = 1
        elif self.current_thrust >= 0 and not self.is_reverse:
            is_positive = 1
        else:
            is_positive = 0
        y1 = self.interp_functions[self.VALID_VOLTAGES.index(self.max_voltage)][is_positive](self.current_thrust * (-1 if self.is_reverse else 1))
        y0 = self.interp_functions[self.VALID_VOLTAGES.index(self.min_voltage)][is_positive](self.current_thrust * (-1 if self.is_reverse else 1))
        d = y1 - y0
        self.pwm = d*(self.voltage-(self.min_voltage))/2+y0

    def update_duty_cycle(self):
        self.duty_cycle = round(self.microseconds_to_int16(self.pwm))

    def send_i2c(self):
        # This sends an I2C signal to the PCA 9685 board to move the thruster IRL
        if self.output == "sim":
            # if (int(self.thruster_num) == 3):
            #     self.test_pub.publish(Int64(-10))
            rospy.logwarn("Unable to send i2c command to PCA board cause output mode is in sim mode and not real mode!")
            return
        try:
            # if (int(self.thruster_num) == 3):
            #     self.test_pub.publish(Int64(self.pwm))
            # rospy.wait_for_service('/pwm_to_i2c', timeout=0.2)
            # self.pwm_to_i2c = rospy.ServiceProxy(self.pca_name, PWMToI2C)
            resp = self.pwm_to_i2c(int(self.pwm), int(self.freq), int(self.channel_num))
            return resp.okay
        except Exception as e:
            # if (int(self.thruster_num) == 3):
            #     self.test_pub.publish(Int64(-20))
            rospy.logdebug("Service call failed: %s"%e)
    
    def publish_to_sim(self):
        if self.output=="sim":
            rospy.logdebug("Inside publish_to_sim of thruster_%s", str(self.thruster_num))
            float_msg = FloatStamped()
            float_msg.header.frame_id = self.float_frame
            float_msg.header.stamp = rospy.Time.now()
            float_msg.data = float(self.get_thrust()*30)
            rospy.logdebug("publishing to %s: %f", self.pub.name, float_msg.data)
            self.pub.publish(float_msg)
    
    def get_pwm(self):
        # This is in microseconds
        return self.pwm

    def reset(self):
        self.set_thrust(0)

def thruster_state_callback(data):
    global thruster
    if (data.data):
        thruster.enable_thruster()
        rospy.loginfo("Enabling thruster")
    else:
        thruster.kill_thruster()
        rospy.loginfo("Killing thruster")

def thruster_output_callback(data):
    global thruster
    rospy.logdebug("Inside callback of thruster_%s", str(thruster.thruster_num))
    thruster.set_thrust(data.data)
    if thruster.output == "sim":
        thruster.publish_to_sim()
    else:
        thruster.send_i2c()

if __name__ == "__main__":
    num_thruster = sys.argv[1]
    reverse = False if sys.argv[2] == "False" else True
    pca_num = sys.argv[3]

    if len(sys.argv) < 4:
        rospy.logerr("Not enough input arguments to create thruster node!")
        raise ValueError("Not enough input arguments to create thruster node!")
    node_name = "thruster_" + str(num_thruster)
    rospy.init_node(node_name, log_level=rospy.DEBUG, anonymous=False)
    output_type = rospy.get_param("/output_type", default="sim")
    frequency = rospy.get_param("/freq", default=218)
    pca_srv = "/pwm_to_i2c_" + pca_num
    thruster = Thruster(thruster_num=num_thruster, is_reverse=reverse, output=output_type, freq = frequency, pca_name=pca_srv)
    
    if output_type == "sim":
        rate = rospy.Rate(10)

    # Publishers
    # if (int(num_thruster) == 3):
    # thruster.test_pub = rospy.Publisher('/mantaray/i2c_pwm_val', Int64, queue_size=10)
    rospy.loginfo("Thruster output type = %s", output_type)
    
    # Subscribers
    rospy.Subscriber('/mantaray/thruster_'+str(num_thruster)+'/is_on', Bool, thruster_state_callback)
    rospy.Subscriber('/mantaray/thruster_'+str(num_thruster)+'/output', Float64, thruster_output_callback)
    
    if (output_type == "real"):
        rospy.wait_for_service(thruster.pca_name, timeout=10)
        thruster.channel_num = (int(thruster.thruster_num)%4) + 1
        thruster.pwm_to_i2c = rospy.ServiceProxy(thruster.pca_name, PWMToI2C)
        rospy.loginfo("Thruster initializing from i2c")
        thruster.send_i2c()
        sleep(3)
        rospy.loginfo("Initial thruster i2c connection initialed")
        # rospy.wait_for_service(pca_srv, timeout=30)
    rospy.spin()

    # Create a publisher that publishes to the appropriate topic (sim thruster input or service client for )