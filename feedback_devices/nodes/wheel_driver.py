#!/usr/bin/python

from ArduController import *

import roslib; roslib.load_manifest('feedback_wheel')
import rospy

from feedback_wheel.msg import *


class WheelDriver:
    def __init__(self, port):
        rospy.init_node('Wheel_Driver')
        self.micro = ArduWheel(port)
        rospy.Subscriber("feedback_wheel/braking_force", BrakeForce, self.callback)
        self.pub = rospy.Publisher("feedback_wheel/brake_state", BrakeState)
        rate = rospy.Rate(rospy.get_param('~hz', 32))
        self.brake_force = 0.0
        while not rospy.is_shutdown():
            rate.sleep()
            self.micro.Control(self.brake_force)
            self.pub.publish(self.brake_force)

    def callback(self, data):
        self.brake_force = data.braking


if __name__ == "__main__":
    port = "/dev/ttyACM0"
    #port = rospy.get_param("VexBot_Driver/port")
    WheelDriver(port)
