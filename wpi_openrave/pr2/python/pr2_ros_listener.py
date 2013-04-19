#!/usr/bin/env python
#
# Bener Suay, December 2012
# benersuay@wpi.edu
#
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
import sys
import roslib; 
roslib.load_manifest('std_msgs')
import rospy
from std_msgs.msg import Float64


class TrajectoryPlanner:
    def __init__(self,args=None):
        print "1"
        self.c = 0
        self.env = Environment()
        robot = self.env.ReadRobotURI('robots/pr2-beta-static.zae')
        self.env.Add(robot)
        self.env.SetViewer('qtcoin')
        rospy.init_node('listener', anonymous=False)
        rospy.Subscriber("chatter", Float64, self.callback)
        rospy.spin()

    def callback(self,data):
        target = RaveCreateKinBody(self.env,'')
        target.InitFromBoxes(array([[0,0.1,0,0.01,0.1,0.01]]),True)
        target.SetName('object_'+str(self.c))
        self.c = self.c + 1
        self.env.Add(target)
        print "3"
        T = eye(4)
        T[0:3,3] = [1,1,data.data]
        target.SetTransform(T)
        print "4"

if __name__ == "__main__":
    tp = TrajectoryPlanner()
