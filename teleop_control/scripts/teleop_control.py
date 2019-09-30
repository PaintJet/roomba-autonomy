#!/usr/bin/env python

# ----------------------------------------------- #
# Table of index number of /joy.buttons:
# Index   Button name on the actual controller
#
# 0   A
# 1   B
# 3   X
# 4   Y
# 6   L1
# 7   R1
# 6   back
# 7   start
# 11  right middle button
# 13   Button stick left
# 14  Button stick right

# Table of index number of /joy.axes:
# Index   Axis name on the actual controller
#
# 0   Moving left joystick left (+) and right (-)
# 1   Moving left joystick up (+) and down (-)
# 2   Moving right joystick left (+) and right (-)
# 3   Moving right joystick up (+) and down (-)
# 4   Right Trigger - pressing further goes from 1 -> -1
# 5   Left Trigger
# 6   DPad left/right - (+) is left, (-) is right
# 7   DPad up/down - (+) is up, (-) is down
# ----------------------------------------------- #

# import required packages
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopControl:

    # Constructor
    def __init__(self):

        # Initialize publishers
        self.drive_pub = rospy.Publisher('cmd_vel', Twist, queue_size=0)
        # other publishers will be added when necessary

        # Initialize subscribers
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)


    # Callback function for joystick controls
    def joy_callback(self, data):
        self.set_drive_speed(data)

    # Sets drive speed based on left joystick input
    def set_drive_speed(self, data):
        twist = Twist()
        twist.linear.x = data.axes[1]
        twist.angular.z = data.axes[2]
        self.drive_pub.publish(twist)


if __name__ == '__main__':

    # Initialize as ROS node
    rospy.init_node('teleop_control')

    # Create a TeleopControl object
    control = TeleopControl()

    # Ready to go
    rospy.loginfo("Teleop Control initialized...")

    # Loop continuously
    rospy.spin()