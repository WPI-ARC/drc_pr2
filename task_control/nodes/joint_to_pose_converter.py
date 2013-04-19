#!/usr/bin/python

#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team

import roslib; roslib.load_manifest('task_control')
import rospy
import math
import random
import time
import os
import sys
from two_arm_trajectory import *
import xml
from xml.etree.ElementTree import *

from std_msgs.msg import String
import kinematics_msgs
from kinematics_msgs.msg import *
from kinematics_msgs.srv import *
import arm_navigation_msgs
from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *

class JointToPoseConverter:

    def __init__(self):
        #Set the planning scene WHY, OH, WHY, DO I HAVE TO SET THIS SHIT? FUCK YOU WILLOW GARAGE
        rospy.wait_for_service("/environment_server/set_planning_scene_diff")
        self.planner_host = rospy.ServiceProxy("/environment_server/set_planning_scene_diff", SetPlanningSceneDiff)
        dummy_setup = SetPlanningSceneDiff._request_class()
        self.planner_host.call(dummy_setup)
        #Set up the left arm FK service call
        rospy.wait_for_service("pr2_left_arm_kinematics/get_fk")
        self.left_FK_host = rospy.ServiceProxy("pr2_left_arm_kinematics/get_fk", GetPositionFK)
        #Set up the right arm FK service call
        rospy.wait_for_service("pr2_right_arm_kinematics/get_fk")
        self.right_FK_host = rospy.ServiceProxy("pr2_right_arm_kinematics/get_fk", GetPositionFK)
        print "Services loaded"

    def ConvertLeftArmStateToPose(self, joint_state):
        fk_request = GetPositionFK._request_class()
        fk_request.header.frame_id = "torso_lift_link"
        fk_request.fk_link_names = ["l_wrist_roll_link"]
        fk_request.robot_state.joint_state.name = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        fk_request.robot_state.joint_state.position = joint_state
        fk_result = self.left_FK_host.call(fk_request)
        real_pose = fk_result.pose_stamped[0].pose
        return [real_pose.position.x, real_pose.position.y, real_pose.position.z, real_pose.orientation.x, real_pose.orientation.y, real_pose.orientation.z, real_pose.orientation.w]

    def ConvertRightArmStateToPose(self, joint_state):
        fk_request = GetPositionFK._request_class()
        fk_request.header.frame_id = "torso_lift_link"
        fk_request.fk_link_names = ["r_wrist_roll_link"]
        fk_request.robot_state.joint_state.name = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        fk_request.robot_state.joint_state.position = joint_state
        fk_result = self.right_FK_host.call(fk_request)
        real_pose = fk_result.pose_stamped[0].pose
        return [real_pose.position.x, real_pose.position.y, real_pose.position.z, real_pose.orientation.x, real_pose.orientation.y, real_pose.orientation.z, real_pose.orientation.w]

    def ConvertTwoArmTrajectoryFile(self, joint_traj_file, target_file_name):
        #Load the two-arm joint trajectory file
        joint_trajectory = TwoArmTrajectory(joint_traj_file)
        #Make a new two-arm pose trajectory
        pose_trajectory = TwoArmTrajectory(None, "POSE")
        #For every state in joint trajectory, convert to poses
        for robot_state in joint_trajectory.states:
            #Get the joint states
            left_arm_joint_state = robot_state.left
            right_arm_joint_state = robot_state.right
            #Convert to pose states
            left_desired_pose = self.ConvertLeftArmStateToPose(left_arm_joint_state.desired)
            left_actual_pose = self.ConvertLeftArmStateToPose(left_arm_joint_state.actual)
            right_desired_pose = self.ConvertRightArmStateToPose(right_arm_joint_state.desired)
            right_actual_pose = self.ConvertRightArmStateToPose(right_arm_joint_state.actual)
            #Compute the pose error
            left_pose_error = self.PoseDistanceRaw(left_desired_pose, left_actual_pose)
            right_pose_error = self.PoseDistanceRaw(right_desired_pose, right_actual_pose)
            #Assemble into pose trajectory state
            new_left_pose_state = TrajectoryState(left_desired_pose, left_actual_pose, left_pose_error, left_arm_joint_state.secs, left_arm_joint_state.nsecs, "POSE")
            new_right_pose_state = TrajectoryState(right_desired_pose, right_actual_pose, right_pose_error, right_arm_joint_state.secs, right_arm_joint_state.nsecs, "POSE")
            new_pose_state = TwoArmTrajectoryState(new_left_pose_state, new_right_pose_state, "POSE")
            pose_trajectory.states.append(new_pose_state)
        pose_trajectory.code = joint_trajectory.code
        pose_trajectory.Write(target_file_name)
        return target_file_name

    def PoseDistanceRaw(self, pose1, pose2):
        x_dist = pose1[0] - pose2[0]
        y_dist = pose1[1] - pose2[1]
        z_dist = pose1[2] - pose2[2]
        angle_dist = self.QuaternionDistance(pose1, pose2)
        return [x_dist, y_dist, z_dist, angle_dist]

    def QuaternionDistance(self, q1, q2):
        dot_product = (q1[3]* q2[3]) + (q1[4] * q2[4]) + (q1[5] * q2[5]) + (q1[6] * q2[6])
        temp_value = 2 * (dot_product ** 2) - 1
        return math.acos(temp_value)
