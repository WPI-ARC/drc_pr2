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
roslib.load_manifest('geometry_msgs')
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
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
        rospy.Subscriber("wheel_pose", PoseStamped, self.callback)
        rospy.spin()

    def callback(self,wheelPose):
        T0_tll = self.robot.GetLinks()[16].GetTransform() # Transform from the origin to torso_lift_link frame
        Ttll_wheel = MakeTransform(rotationMatrixFromQuat([wheelPose.pose.orientation.w,wheelPose.pose.orientation.x,wheelPose.pose.orientation.y,wheelPose.pose.orientation.z]),matrix([wheelPose.pose.position.x,wheelPose.pose.position.y,wheelPose.pose.position.z])) # Transform from torso_lift_link to racing wheel's frame
        
        #wheel_x = torso_lift_link[0][3]+data.data[0]
        #wheel_y = torso_lift_link[1][3]+data.data[1]
        #wheel_z = torso_lift_link[2][3]+data.data[2]
        #w=[wheel_x,wheel_y,wheel_z]

        wheelStraightUp = MakeTransform(matrix(dot(rodrigues([0,0,-pi/2]),rodrigues([pi/2,0,0]))),transpose(matrix([0,0,0])))

        wheelAtCorrectLocation = dot(T0_tll,Ttll_wheel)
        wheelAtCorrectLocation = dot(wheelAtCorrectLocation, wheelStraightUp)

        self.crankid.SetTransform(array(wheelAtCorrectLocation))
        

if __name__ == "__main__":
    tp = TrajectoryPlanner()
