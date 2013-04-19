#!/usr/bin/python

#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team
#
#   Manages execution of a given trajectory using a PR2
#   Each trajectory is added to the library of trajectories
#   for use with Dynamic Time Warping to classify and identify
#   new trajectories as encountered in execution.
#

import roslib; roslib.load_manifest('arm_plan_pkg')
import rospy
import math
import random
import time
from trajectory import *
from dtw import *

from std_msgs.msg import String
from tf import *
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from arm_plan_pkg.srv import *

class TestManager:
    def __init__(self, arm_name):
        rospy.init_node(arm_name + 'test_manager', anonymous=False)
        if arm_name == "l_":
            self.arm = "left"
        else:
            self.arm = "right"
        print "Loading " + self.arm + " arm test manager"
        
        #Setup the service clients
        rospy.wait_for_service("arm_plan_pkg/Trajectory_Record")
        self.traj_client = rospy.ServiceProxy("arm_plan_pkg/Trajectory_Record", TrajectoryRecord)
        #self.valv_client = rospy.ServiceClient
        print "Service host up"
        #Record a trajectory
        self.Record(5.0)

    def GenTrajectory1(self):
        trajrequest = JointTrajectory()
        trajrequest.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        start_point = JointTrajectoryPoint()
        start_point.positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        start_point.time_from_start = rospy.Duration(1.0)
        trajrequest.points = [start_point]
        return trajrequest

    def Record(self, delay):
        #Generate a series of trajectories
        print "Recording a trajectory"
        test_name = "Record Wheel Example"
        trajrequest = self.GenTrajectory1()
        result = self.traj_client(trajrequest, test_name, delay)
        print "Recorded a trajectory:", result

if __name__ == '__main__':
    test = TestManager("r_")
