import os
import rospy
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
import rospkg
import pandas as pd
#TODO:WILL Finish creating this class. Currently DOESN'T WORK. This version uses a very simple method of getting PWM from voltage w/o interpolating
class Thruster():
    '''
    With these thrusters, I want the input to be how much thrust (kg f) the user wants and
    the output to depend whether the Thruster is used in the sim (output will be a FloatStamped)
    or in real life (output will be a PWM or I2C signal).
    '''
    def __init__(self, saturation=3, voltage=12, thruster_num = 0, output="sim", freq=0):
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
        self.saturation = saturation
        self.current_thrust = 0
        min_voltage = rospy.get_param("min_voltage", default=10)
        max_voltage = rospy.get_param("max_voltage", default=20)
        self.voltage = max(min(voltage, max_voltage), min_voltage)
        self.voltage = str(self.voltage)
        self.thruster_num = thruster_num
        self.output = output
        self.pwm = None
        self.freq = freq
        self.duty_cycle = 0
        self.namespace = rospy.get_param("namespace")
        self.pub = None

        rospy.init_node("thruster_" + str(self.thruster_num))

        if self.output == "sim":
            # Publishes the output to the simulation mantaray
            self.pub = rospy.Publisher('/' + self.namespace + '/thrusters/'+str(self.thruster_num)+'/input', FloatStamped, queue_size = 10)
        elif self.output == "real":
            # Publishes the output to the real sub through the service pwm_to_i2c
            pass

        r = rospkg.RosPack()
        file_path = r.find_path("thruster")
        iodata_path = os.path.join(file_path, "data","t200_data", self.voltage+" V", "force_pwm.csv")

        self.iodata = pd.read_csv(iodata_path)

        self.update_pwm()

    def set_thrust(self, thrust):
        self.current_thrust = max(min(thrust, self.saturation), -self.saturation)
        if self.output == "pwm":
            self.update_pwm()
        elif self.output == "i2c":
            self.update_i2c()
        else:
            # If unknown or "both", update both for resiliency
            self.update_pwm()
            self.update_i2c()

    def get_thrust(self):
        return self.current_thrust
    
    def add_thrust(self, thrust):
        self.set_thrust(self.current_thrust+thrust)

    def update_voltage(self, volt):
        # Use in case of dynamic voltage. Can be useful for long-term operations
        pass

    def reset(self):
        self.current_thrust = 0
        if self.output == "pwm":
            self.update_pwm()
        elif self.output == "i2c":
            self.update_i2c()

    def update_pwm(self):
        # This calculates the approximate PWM signal necessary to get the needed thrust
        df_closest = self.iodata.iloc[(self.iodata[' Force (Kg f)']-self.current_thrust).abs().argsort()[:1]]
        # df_closest = self.iodata.iloc[(self.iodata[' Force (Kg f)']-self.current_thrust).abs().argsort()[:2]] # returns the 2 closest pwm values
        self.pwm = df_closest.values[0][2]

    def update_i2c(self):
        # This calculates the approximate PWM signal necessary to get the needed thrust
        pass
        

    def get_pwm(self):
        # This is in microseconds
        return self.pwm
    
    def get_duty_cycle(self):
        # This utilizes the pca9685 library
        pass


# class SimThruster(Thruster):
#     def __init__(self, saturation):
#         super().__init__(saturation)


# class RealThruster(Thruster):
#     def __init__(self, saturation):
#         super().__init__(saturation)

# if __name__ == "__main__":
#     sat = 3
#     test_thruster = Thruster(sat)
#     print(test_thruster.get_pwm())