#!/usr/bin/python

#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team
#
#   Manages execution of a given trajectory using a PR2
#   Each trajectory is added to the library of trajectories
#   for use with Dynamic Time Warping to classify and identify
#   new trajectories as encountered in execution.
#

import roslib; roslib.load_manifest('task_control')
import rospy
import math
import random
import time

from std_msgs.msg import String
from tf import *
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from task_control.msg import *
from task_control.srv import *

class Initer:
    def __init__(self):
        rospy.init_node('set_init_config', anonymous=False)
        self.left_arm_client = actionlib.SimpleActionClient("l_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.left_arm_client.wait_for_server()
        self.right_arm_client = actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.right_arm_client.wait_for_server()
        #Set goal states
        left_trajrequest = JointTrajectory()
        left_trajrequest.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        left_start_point = JointTrajectoryPoint()
        left_start_point.positions = [0.85,-0.1,0.114,-1.9,0.0,-1.00,-1.00]
        left_start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        left_start_point.time_from_start = rospy.Duration(1.0)
        left_trajrequest.points = [left_start_point]
        right_trajrequest = JointTrajectory()
        right_trajrequest.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        right_start_point = JointTrajectoryPoint()
        right_start_point.positions = [-0.85,-0.1,0.114,-1.9,0.0,-1.00,1.00]
        right_start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        right_start_point.time_from_start = rospy.Duration(1.0)
        right_trajrequest.points = [right_start_point]
        #Send trajectories
        left_goal = JointTrajectoryGoal()
        left_goal.trajectory = left_trajrequest
        right_goal = JointTrajectoryGoal()
        right_goal.trajectory = right_trajrequest
        self.left_arm_client.send_goal(left_goal)
        self.right_arm_client.send_goal(right_goal)
        self.left_arm_client.wait_for_result()
        self.right_arm_client.wait_for_result()
        print "Initconfig reached"

if __name__ == '__main__':
    test = Initer()
