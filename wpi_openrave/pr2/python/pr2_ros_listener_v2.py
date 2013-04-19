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
from std_msgs.msg import Float64MultiArray
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *


class TrajectoryPlanner:
    def __init__(self,args=None):
        self.env = Environment()
        self.env.Load('../../models/driving_wheel.robot2.xml')
        self.robot = self.env.ReadRobotURI('robots/pr2-beta-static.zae')
        self.crankid = self.env.GetRobots()[0]
        self.env.Add(self.robot)
        self.env.SetViewer('qtcoin')
        rospy.init_node('openrave', anonymous=False)
        # Subscribe to valve_pose_updater node.
        rospy.Subscriber("valve_pose_updater", Float64MultiArray, self.callback)
        rospy.spin()

    def callback(self,data):
        #self.crankid.SetTransform(array(MakeTransform(matrix(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2,0,0]))), transpose(matrix([0.5,0,0.8128])))))
        torso_lift_link = self.robot.GetLinks()[16].GetTransform()
        wheel_x = torso_lift_link[0][3]+data.data[0]
        wheel_y = torso_lift_link[1][3]+data.data[1]
        wheel_z = torso_lift_link[2][3]+data.data[2]
        w=[wheel_x,wheel_y,wheel_z]
        self.crankid.SetTransform(array(MakeTransform(matrix(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2,0,0]))), transpose(matrix(w)))))
        print "4"

if __name__ == "__main__":
    tp = TrajectoryPlanner()
