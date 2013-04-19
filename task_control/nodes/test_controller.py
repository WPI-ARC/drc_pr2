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

class TrajectoryManager:
    def __init__(self):
        rospy.init_node('test_manager', anonymous=False)
        print "Loading pr2 arm test manager"
        #Setup the service clients
        rospy.wait_for_service("task_control/Trajectory_Request")
        self.traj_client = rospy.ServiceProxy("task_control/Trajectory_Request", PR2ManagedJointTrajectory)
        print "Service client up"
        print "Building goals"
        self.close_gripper_goal = Pr2GripperCommandGoal()
        self.close_gripper_goal.command.position = 0.00
        self.close_gripper_goal.command.max_effort = -1.0
        self.open_gripper_goal = Pr2GripperCommandGoal()
        self.open_gripper_goal.command.position = 0.09
        self.open_gripper_goal.command.max_effort = -1.0
        #Build library
        print "--- Building example library ---"
        library = self.BuildLibrary(4)
        print "*** Example library built ***"
        print "Complete...!"

    def GenStartToGrasp(self):
        #Generate the trajectory from the safe start position to the grasp start point
        left_trajrequest = JointTrajectory()
        left_trajrequest.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        left_start_point = JointTrajectoryPoint()
        left_start_point.positions = [0.6555987609701418, -0.07557009733181258, 0.8876102066400376, -1.1555330251015765, 0.7164323245861368, -1.3929341132542807, -2.6680098443659164]
        left_start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        left_start_point.time_from_start = rospy.Duration(1.0)
        left_end_point = JointTrajectoryPoint()
        left_end_point.positions = [0.5585151120451115, -0.06474194717444243, 1.1212471886609618, -1.3401156821155744, 0.742463513713889, -1.0669652838681194, -2.7403216695714425]
        left_end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        left_end_point.time_from_start = rospy.Duration(2.0)
        left_trajrequest.points = [left_start_point, left_end_point]
        right_trajrequest = JointTrajectory()
        right_trajrequest.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        right_start_point = JointTrajectoryPoint()
        right_start_point.positions = [-0.7378203882748354, -0.17354634681100922, -1.0326601067958974, -1.095887492011563, -0.5332236884070002, -1.4107528998077328, -0.5704304605960075]
        right_start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        right_start_point.time_from_start = rospy.Duration(1.0)
        right_end_point = JointTrajectoryPoint()
        right_end_point.positions = [-0.7374058551880507, 0.09453496450700766, -1.0632878786325461, -1.4889399734178408, -0.6790561946093634, -0.9147947195924561, -0.4794533086677557]
        right_end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        right_end_point.time_from_start = rospy.Duration(2.0)
        right_trajrequest.points = [right_start_point, right_end_point]
        return [left_trajrequest, right_trajrequest]

    def GenGraspToIdle(self):
        left_trajrequest = JointTrajectory()
        left_trajrequest.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        left_start_point = JointTrajectoryPoint()
        left_start_point.positions = [0.5585151120451115, -0.06474194717444243, 1.1212471886609618, -1.3401156821155744, 0.742463513713889, -1.0669652838681194, -2.7403216695714425]
        left_start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        left_start_point.time_from_start = rospy.Duration(1.0)
        left_end_point = JointTrajectoryPoint()
        left_end_point.positions = [2.134487001384054, -0.3191188809495376, -0.016630659259750713, -1.961616346085059, -0.04622366977005289, -2.009499019985633, -3.047277100874081]
        left_end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        left_end_point.time_from_start = rospy.Duration(2.0)
        left_trajrequest.points = [left_start_point, left_end_point]
        right_trajrequest = JointTrajectory()
        right_trajrequest.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        right_start_point = JointTrajectoryPoint()
        right_start_point.positions = [-0.7374058551880507, 0.09453496450700766, -1.0632878786325461, -1.4889399734178408, -0.6790561946093634, -0.9147947195924561, -0.4794533086677557]
        right_start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        right_start_point.time_from_start = rospy.Duration(1.0)
        right_end_point = JointTrajectoryPoint()
        right_end_point.positions = [-2.1350456105921167, -0.353056773638662, 0.15588986625769907, -1.867804925108533, 0.013199894927994088, -2.005563346489288, -0.09091865217837736]
        right_end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        right_end_point.time_from_start = rospy.Duration(2.0)
        right_trajrequest.points = [right_start_point, right_end_point]
        return [left_trajrequest, right_trajrequest]

    def GenIdleToGrasp(self):
        left_trajrequest = JointTrajectory()
        left_trajrequest.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        left_start_point = JointTrajectoryPoint()
        left_start_point.positions = [2.134487001384054, -0.3191188809495376, -0.016630659259750713, -1.961616346085059, -0.04622366977005289, -2.009499019985633, -3.047277100874081]
        left_start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        left_start_point.time_from_start = rospy.Duration(1.0)
        left_end_point = JointTrajectoryPoint()
        left_end_point.positions = [0.5585151120451115, -0.06474194717444243, 1.1212471886609618, -1.3401156821155744, 0.742463513713889, -1.0669652838681194, -2.7403216695714425]
        left_end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        left_end_point.time_from_start = rospy.Duration(2.0)
        left_trajrequest.points = [left_start_point, left_end_point]
        right_trajrequest = JointTrajectory()
        right_trajrequest.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        right_start_point = JointTrajectoryPoint()
        right_start_point.positions = [-2.1350456105921167, -0.353056773638662, 0.15588986625769907, -1.867804925108533, 0.013199894927994088, -2.005563346489288, -0.09091865217837736]
        right_start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        right_start_point.time_from_start = rospy.Duration(1.0)
        right_end_point = JointTrajectoryPoint()
        right_end_point.positions = [-0.7374058551880507, 0.09453496450700766, -1.0632878786325461, -1.4889399734178408, -0.6790561946093634, -0.9147947195924561, -0.4794533086677557]
        right_end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        right_end_point.time_from_start = rospy.Duration(2.0)
        right_trajrequest.points = [right_start_point, right_end_point]
        return [left_trajrequest, right_trajrequest]

    def GenIdleToStart(self):
        left_trajrequest = JointTrajectory()
        left_trajrequest.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        left_start_point = JointTrajectoryPoint()
        left_start_point.positions = [2.134487001384054, -0.3191188809495376, -0.016630659259750713, -1.961616346085059, -0.04622366977005289, -2.009499019985633, -3.047277100874081]
        left_start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        left_start_point.time_from_start = rospy.Duration(1.0)
        left_end_point = JointTrajectoryPoint()
        left_end_point.positions = [0.6555987609701418, -0.07557009733181258, 0.8876102066400376, -1.1555330251015765, 0.7164323245861368, -1.3929341132542807, -2.6680098443659164]
        left_end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        left_end_point.time_from_start = rospy.Duration(2.0)
        left_trajrequest.points = [left_start_point, left_end_point]
        right_trajrequest = JointTrajectory()
        right_trajrequest.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        right_start_point = JointTrajectoryPoint()
        right_start_point.positions = [-2.1350456105921167, -0.353056773638662, 0.15588986625769907, -1.867804925108533, 0.013199894927994088, -2.005563346489288, -0.09091865217837736]
        right_start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        right_start_point.time_from_start = rospy.Duration(1.0)
        right_end_point = JointTrajectoryPoint()
        right_end_point.positions = [-0.7378203882748354, -0.17354634681100922, -1.0326601067958974, -1.095887492011563, -0.5332236884070002, -1.4107528998077328, -0.5704304605960075]
        right_end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        right_end_point.time_from_start = rospy.Duration(2.0)
        right_trajrequest.points = [right_start_point, right_end_point]
        return [left_trajrequest, right_trajrequest]

    def RunTest(self):
        trajectory_to_execute = PR2ManagedJointTrajectory._request_class()
        #print dir(trajectory_to_execute)
        #Set trajectories
        trajectory_to_execute.left_arm_command.prep_trajectory = self.GenStartToGrasp()[0]
        trajectory_to_execute.left_arm_command.main_trajectory = self.GenGraspToIdle()[0]
        trajectory_to_execute.left_arm_command.return_trajectory = self.GenIdleToGrasp()[0]
        trajectory_to_execute.left_arm_command.post_trajectory = self.GenIdleToStart()[0]
        trajectory_to_execute.right_arm_command.prep_trajectory = self.GenStartToGrasp()[1]
        trajectory_to_execute.right_arm_command.main_trajectory = self.GenGraspToIdle()[1]
        trajectory_to_execute.right_arm_command.return_trajectory = self.GenIdleToGrasp()[1]
        trajectory_to_execute.right_arm_command.post_trajectory = self.GenIdleToStart()[1]
        #Set gripper commands
        trajectory_to_execute.left_gripper_commands = [self.open_gripper_goal, self.close_gripper_goal, self.open_gripper_goal, self.open_gripper_goal, self.close_gripper_goal]
        trajectory_to_execute.right_gripper_commands = [self.open_gripper_goal, self.close_gripper_goal, self.open_gripper_goal, self.open_gripper_goal, self.close_gripper_goal]
        #Set header info
        trajectory_to_execute.code = "test"
        trajectory_to_execute.filepath = "test"
        #Send the trajectory to the trajectory controller
        result = self.traj_client(trajectory_to_execute)
        return result

    def BuildLibrary(self, times):
        #Generate a series of trajectories
        codes = []
        for iteration in range(times):
            #Assemble a test
            print "Running a wheel turning trajectory"
            #Assemble a trajectory
            new_code = self.RunTest()
            print new_code
            #Wait a second for everything to stabilize
            time.sleep(1.0)
            codes.append(new_code)
        #Return the return codes
        return codes

if __name__ == '__main__':
    test = TrajectoryManager()
