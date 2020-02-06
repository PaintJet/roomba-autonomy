#!/usr/bin/env python
import rospy
import pigpio
from geometry_msgs.msg import Twist

RIGHT_MOTOR = 18
LEFT_MOTOR = 19
FREQUENCY = 50


class TalonPWM:

    def __init__(self):
        rospy.Subscriber('/cmd_vel', Twist, self.get_cmd_vel)
        self.pi = pigpio.pi()

    # get cmd_vel message, and get linear velocity and angular velocity
    def get_cmd_vel(self, data):
        x, angular = self.range_cmd_vel(data.linear.x, data.angular.z)
        self.generate_pwm_values(x, angular)

    # Cap Values between [-1,1]
    @staticmethod
    def range_cmd_vel(x, angular):
        x = max(-1, min(x, 1))
        angular = max(-1, min(angular, 1))
        return x, angular

    def generate_pwm_values(self, x, angular):
        left_duty_cycle, right_duty_cycle = self.get_duty_cycles(x, angular)

        # Set Right Motor
        self.set_motor(RIGHT_MOTOR, right_duty_cycle)

        # Set Left Motor
        self.set_motor(LEFT_MOTOR, left_duty_cycle)

    def set_motor(self, motor, duty_cycle):
        self.pi.hardware_PWM(motor, FREQUENCY, duty_cycle * 10000)
	rospy.loginfo(duty_cycle)

    def stop_motors(self):
        self.set_motor(RIGHT_MOTOR, 0)
        self.set_motor(LEFT_MOTOR, 0)

    # Get the duty Cycles for the Left and Right Motors
    @staticmethod
    def get_duty_cycles(x, angular):
        PMAX = 0.002
        PMIN = 0.001

        # Calculate W MAX
        LEFT_WL_MAX = (1 + 0.275)/0.127
        RIGHT_WL_MAX = (1 - 0.275)/0.127

        # Calculate WL
        W_LEFT = (x - (angular * 0.275)) / 0.127
        W_RIGHT = (x + (angular * 0.275)) / 0.127

        # Determine Period For Each Motor
        PL = (PMAX - PMIN) / (LEFT_WL_MAX * 2) * W_LEFT + PMIN
        PR = PMAX - ( (PMAX - PMIN) / (RIGHT_WL_MAX * 2) * W_RIGHT) # + PMIN

        return PL * FREQUENCY * 150, PR * FREQUENCY * 150


if __name__ == '__main__':
    # Initialize Node
    rospy.init_node('talon_pwm')

    # Create talon pwm object
    talon = TalonPWM()

    # Zero Motors
    talon.stop_motors()

    # Log Success
    rospy.loginfo("Talon PWM Started")

    # Loop Continuously
    rospy.spin()

    # Zero Motors
    talon.stop_motors()
