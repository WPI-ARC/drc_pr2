#!/usr/bin/python

import roslib; roslib.load_manifest('arm_plan_pkg')
import rospy
import math

from std_msgs.msg import String
from gazebo_msgs.msg import *
from arm_plan_pkg.srv import *

class SimController:
    def __init__(self, valve_path):
        rospy.init_node('SimController', anonymous=False)
        #Allocate some empty status variables
        self.latest_pose = None
        self.latest_twist = None
        #Setup the service callback
        self.GetHandler = rospy.Service("Valve_Get", ValveGet, self.GetHandler)
        self.SetHandler = rospy.Service("Valve_Set", ValveSet, self.SetHandler)
        #Announce
        print "SimController loaded..."
        #Setup the subscribers
        rospy.Subscriber("gazebo/link_states", LinkStates, self.LinkStatesCB)
        #Spin FAST
        rate = rospy.Rate(rospy.get_param('~hz', 240))
        while not rospy.is_shutdown():
            rate.sleep()

    def LinkStatesCB(self, message):
        index = 0
        for i in range(len(message.name)):
            if "valve_urdf" in message.name[i] and "rod" in message.name[i]:
                index = i
                break
        self.latest_pose = message.pose[index]
        self.latest_twist = message.twist[index]
        

    def GetHandler(self, request):
        print "Service:\n", request
        return 0

    def SetHandler(self, request):
        print "Service:\n", request
        return 0

class HardController:
    def __init__(self, valve_path):
        rospy.init_node('HardController', anonymous=False)
        #Allocate some empty status variables
        self.Valve = None
        #Setup the service callback
        self.GetHandler = rospy.Service("Valve_Get", ValveGet, self.GetHandler)
        self.SetHandler = rospy.Service("Valve_Set", ValveSet, self.SetHandler)
        #Setup the subscribers
        #Spin FAST
        rate = rospy.Rate(rospy.get_param('~hz', 240))
        while not rospy.is_shutdown():
            rate.sleep()

    def GetHandler(self, request):
        print "Service:\n", request
        return 0

    def SetHandler(self, request):
        print "Service:\n", request
        return 0

if __name__ == '__main__':
    '''Controller for the simulated valve wheel in gazebo'''
    SimController("I don't care")
    '''Controller for the real force-feedback valve wheel'''
    #HardController("/dev/hidraw0")
