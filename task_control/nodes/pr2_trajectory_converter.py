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
import time
import subprocess
from copy import deepcopy
from two_arm_trajectory import *
from trajectory_analyzer import *
from pose_evaluator import *
from joint_to_pose_converter import *
import os

from std_msgs.msg import String
from tf import *
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *
from task_control.msg import *
from task_control.srv import *

class PR2TrajectoryConverter:
    def __init__(self, output_path, output_file):
        self.output_path = output_path
        self.output_file = output_file
        rospy.init_node('pr2_trajectory_converter', anonymous=False)
        #Setup the service callback
        self.RequestHandler = rospy.Service("task_control/Trajectory_Request", PR2ManagedJointTrajectory, self.RequestHandler)
        print "[STARTUP] ManagedJointTrajectory service host up"
        print bcolors.OKGREEN + bcolors.BOLD + "[STARTUP] Trajectory converter running..."
        print bcolors.ENDC
        rate = rospy.Rate(rospy.get_param('~hz', 10))
        while not rospy.is_shutdown():
            rate.sleep()
    
    def RequestHandler(self, request):
        print bcolors.OKBLUE + "----------\n[STATUS] Converting a trajectory..." + bcolors.ENDC
        left_prep = request.left_arm_command.prep_trajectory
        right_prep = request.right_arm_command.prep_trajectory
        left_main = request.left_arm_command.main_trajectory
        right_main = request.right_arm_command.main_trajectory
        left_return = request.left_arm_command.return_trajectory
        right_return = request.right_arm_command.return_trajectory
        left_post = request.left_arm_command.post_trajectory
        right_post = request.right_arm_command.post_trajectory
        g_number = 0
        new_clock = rospy.Duration(0.0)
        trajfile = open(self.output_path + "/" + self.output_file, "w")
        header = "time,number,l_shoulder_pan_joint,l_shoulder_lift_joint,l_upper_arm_roll_joint,l_elbow_flex_joint,l_forearm_roll_joint,l_wrist_flex_joint,l_wrist_roll_joint,r_shoulder_pan_joint,r_shoulder_lift_joint,r_upper_arm_roll_joint,r_elbow_flex_joint,r_forearm_roll_joint,r_wrist_flex_joint,r_wrist_roll_joint"
        output_string = header + "\n"
        output_string += "NOTE: open gripper to maximum\n"
        assert(len(left_prep.points) == len(right_prep.points))
        g_execution_clock = new_clock
        for index in range(len(left_prep.points)):
            lpoint = left_prep.points[index]
            rpoint = right_prep.points[index]
            [line_string, new_clock] = self.ConvertPointsToLine(lpoint, rpoint, g_execution_clock, g_number)
            g_number += 1
            output_string += line_string + "\n"
        output_string += "NOTE: close gripper with maximum force\n"
        assert(len(left_main.points) == len(right_main.points))
        g_execution_clock = new_clock
        for index in range(len(left_main.points)):
            lpoint = left_main.points[index]
            rpoint = right_main.points[index]
            [line_string, new_clock] = self.ConvertPointsToLine(lpoint, rpoint, g_execution_clock, g_number)
            g_number += 1
            output_string += line_string + "\n"
        output_string += "NOTE: open gripper to maximum\n"
        assert(len(left_return.points) == len(right_return.points))
        g_execution_clock = new_clock
        for index in range(len(left_return.points)):
            lpoint = left_return.points[index]
            rpoint = right_return.points[index]
            [line_string, new_clock] = self.ConvertPointsToLine(lpoint, rpoint, g_execution_clock, g_number)
            g_number += 1
            output_string += line_string + "\n"
        assert(len(left_post.points) == len(right_post.points))
        g_execution_clock = new_clock
        for index in range(len(left_post.points)):
            lpoint = left_post.points[index]
            rpoint = right_post.points[index]
            [line_string, new_clock] = self.ConvertPointsToLine(lpoint, rpoint, g_execution_clock, g_number)
            g_number += 1
            output_string += line_string + "\n"
        trajfile.write(output_string)
        trajfile.close()
        return "NOT APPLICABLE"

    def ConvertPointsToLine(self, left_point, right_point, g_execution_clock, g_number):
        left = str(left_point.positions).lstrip("[").rstrip("]").lstrip("(").rstrip(")")
        right = str(right_point.positions).lstrip("[").rstrip("]").lstrip("(").rstrip(")")
        execution_clock = g_execution_clock + left_point.time_from_start
        time = str(float(str(execution_clock)) / 1000000000.0)
        return [time + "," + str(g_number) + "," + left + "," + right, execution_clock]

if __name__ == '__main__':
    path = subprocess.check_output("rospack find task_control", shell=True)
    path = path.strip("\n")
    full_path = path + "/data"
    filename = "pr2_gazebo_error_trajectory.trj"
    print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting trajectory converter... " + bcolors.ENDC
    PR2TrajectoryConverter(full_path, filename)
