#!/usr/bin/python

#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team

import roslib; roslib.load_manifest('task_control')
import rospy
import math
import random
import time
import os
import sys
from trajectory_analysis import *
from dtw import *
import xml
from xml.etree.ElementTree import *

from std_msgs.msg import String
import kinematics_msgs
from kinematics_msgs.msg import *
from kinematics_msgs.srv import *
import arm_navigation_msgs
from arm_navigation_msgs.msg import *
from arm_navigation_msgs.srv import *

class PoseAnalysisTrajectory:

    def __init__(self, filename=None):
        self.desired_poses = []
        self.actual_poses = []
        self.pose_errors = []
        self.times = []
        self.code = ""
        self.REAL = True
        if (filename != None):
            self.load(filename)

    def unload(self, filename):
        XML_string = "<trajectory>\n<info>\n"
        XML_string = XML_string + "<length>\n" + str(len(self.desired_poses)) + "\n</length>\n<code>\n" + self.code + "\n</code>\n</info>\n"
        XML_string = XML_string + "<executed>\n"
        for index in range(len(self.desired_poses)):
            state_string = self.cleanup(index)
            XML_string = XML_string + state_string + "\n"
        XML_string = XML_string + "</executed>\n</trajectory>"
        xml_file = open(filename, "w")
        xml_file.write(XML_string)
        xml_file.close()

    def cleanup(self, index):
        state_string = "<state>\n<info>\n"
        state_string = state_string + "secs: " + str(self.times[index][0]) + "\n"
        state_string = state_string + "nsecs: " + str(self.times[index][1]) + "\n"
        state_string = state_string + "</info>\n<desired>\n"
        state_string = state_string + "positions: " + self.PoseToString(self.desired_poses[index]) + "\n"
        state_string = state_string + "</desired>\n<actual>\n"
        state_string = state_string + "positions: " + self.PoseToString(self.actual_poses[index]) + "\n"
        state_string = state_string + "</actual>\n<error>\n"
        state_string = state_string + "distance: " + str(self.pose_errors[index]) + "\n"
        state_string = state_string + "</error>\n</state>\n"
        return state_string

    def PoseToString(self, new_pose):
        pose_list = [new_pose.position.x, new_pose.position.y, new_pose.position.z, new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, new_pose.orientation.w]
        return str(pose_list)

    def load(self, filename):
        traj_file = open(filename, "r")
        traj_tree = ElementTree().parse(traj_file)
        #load the code
        self.code = traj_tree[0][1].text.strip("\n")
        for entry in traj_tree[1]:
            #load the timing info
            chunks1 = entry[0].text.strip("\n").split("\n")
            times = []
            for chunk in chunks1:
                time = chunk.split(": ")[1]
                times.append(float(time))
            self.times.append(times)
            #load the desired
            chunks2 = entry[1].text.strip("\n").split("\n")
            position = chunks2[0].split(": ")[1]
            self.desired_poses.append(self.Str2List(position))
            #load the actual
            chunks3 = entry[2].text.strip("\n").split("\n")
            position = chunks3[0].split(": ")[1]
            self.actual_poses.append(self.Str2List(position))
            #load the error
            chunks4 = entry[3].text.strip("\n").split("\n")
            error = chunks4[0].split(": ")[1]
            self.pose_errors.append(self.Str2List(error))
        traj_file.close()

    def Str2List(self, raw):
        raw = raw.lstrip("[").rstrip("]")
        parts = raw.split(", ")
        cleaned = []
        for part in parts:
            cleaned.append(float(part))
        return cleaned

class PoseTrajectoryAnalyzer:

    def __init__(self, library_dir, distance_fn=None):
        self.library_path = library_dir
        self.evaluator = DTW()
        self.library = self.LoadFromDirectory(self.library_path)

    def LoadFromDirectory(self, directory_path):
        trajectories = []
        trajectory_files = os.listdir(directory_path)
        for file_name in trajectory_files:
            full_file_path = directory_path + "/" + file_name
            trajectories.append(PoseAnalysisTrajectory(full_file_path))
        return trajectories

    def MinimalAnalyzeDirectory(self, trajectory_path, verbose=True):
        trajectories_to_analyze = self.LoadFromDirectory(trajectory_path)
        results = []
        for traj in trajectories_to_analyze:
            try:
                result = self.MinimalAnalyzeTrajectory(traj)
                results.append(result)
                if (verbose):
                    print"[match,real]"
                    print result
            except:
                print "Could not analyze trajectory"
        return results

    def MinimalAnalyzeTrajectory(self, traj):
        fast_dtw_dict = {}
        for library_traj in self.library:
            if (traj.code != library_traj.code):
                fastcost = self.evaluator.Evaluate(traj.pose_errors, library_traj.pose_errors)
                fast_dtw_dict[str(fastcost)] = library_traj.code
            else:
                print "Self-comparisson ignored"
        fast_min_code = sorted(fast_dtw_dict.keys())[0]
        fast_match = fast_dtw_dict[fast_min_code]
        return [fast_match, traj.code]

    def LeaveOneOut(self):
        results = {}
        for index in range(len(self.library)):
            index_results = {}
            for compare in range(len(self.library)):
                cost = self.evaluator.Evaluate(self.library[index].pose_errors, self.library[compare].pose_errors)
                index_results[self.library[compare].code] = cost
            results[self.library[index].code] = index_results
        return results

class JointsToPoseConverter:

    def __init__(self):
        #Set the planning scene WHY, OH, WHY, DO I HAVE TO SET THIS SHIT? FUCK YOU WILLOW GARAGE
        rospy.wait_for_service("/environment_server/set_planning_scene_diff")
        self.planner_host = rospy.ServiceProxy("/environment_server/set_planning_scene_diff", SetPlanningSceneDiff)
        dummy_setup = SetPlanningSceneDiff._request_class()
        self.planner_host.call(dummy_setup)
        #Set up the FK service call
        rospy.wait_for_service("pr2_left_arm_kinematics/get_fk")
        self.FK_host = rospy.ServiceProxy("pr2_left_arm_kinematics/get_fk", GetPositionFK)
        print "Services loaded"

    def ConvertDirectory(self, directory_path, destination_path):
        joint_trajectories = self.LoadFromDirectory(directory_path)
        pose_trajectories = []
        print "Converting directory now..."
        for joint_trajectory in joint_trajectories:
            converted = self.ConvertToPose(joint_trajectory)
            converted.unload(destination_path + "/" + converted.code + ".xml")
            print "File converted"
        print "Done directory conversion"

    def ConvertToPose(self, joint_trajectory):
        new_pose_trajectory = PoseAnalysisTrajectory()
        for desired_state in joint_trajectory.desired_positions:
            desired_pose = self.ConvertStateToPose(desired_state)
            new_pose_trajectory.desired_poses.append(desired_pose)
        for actual_state in joint_trajectory.actual_positions:
            actual_pose = self.ConvertStateToPose(actual_state)
            new_pose_trajectory.actual_poses.append(actual_pose)
        new_pose_trajectory.times = list(joint_trajectory.times)
        for index in range(len(new_pose_trajectory.desired_poses)):
            desired = new_pose_trajectory.desired_poses[index]
            actual = new_pose_trajectory.actual_poses[index]
            error = self.PoseDistanceRaw(desired, actual)
            new_pose_trajectory.pose_errors.append(error)
        new_pose_trajectory.code = joint_trajectory.code
        return new_pose_trajectory

    def ConvertStateToPose(self, joint_state):
        fk_request = GetPositionFK._request_class()
        #print dir(fk_request)
        fk_request.header.frame_id = "torso_lift_link"
        fk_request.fk_link_names = ["l_wrist_roll_link"]
        fk_request.robot_state.joint_state.name = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        fk_request.robot_state.joint_state.position = joint_state
        fk_result = self.FK_host.call(fk_request)
        #print dir(fk_result)
        return fk_result.pose_stamped[0].pose
            
    def PoseDistanceRaw(self, pose1, pose2):
        x_dist = pose1.position.x - pose2.position.x
        y_dist = pose1.position.y - pose2.position.y
        z_dist = pose1.position.z - pose2.position.z
        angle_dist = self.QuaternionDistance(pose1.orientation, pose2.orientation)
        return [x_dist, y_dist, z_dist, angle_dist]

    def PositionDistance(self, position1, position2):
        return math.sqrt((position1.x - position2.x)**2 + (position1.y - position2.y)**2 + (position1.z - position2.z)**2)

    def PoseDistance(self, pose1, pose2, control):
        position_dist = self.PositionDistance(pose1.position, pose2.position)
        orientation_dist = self.QuaternionDistance(pose1.orientation, pose2.orientation)
        total_distance = (control * position_dist) + ((1 - control) * orientation_dist)
        return total_distance

    def QuaternionDistance(self, q1, q2):
        dot_product = (q1.x * q2.x) + (q1.y * q2.y) + (q1.z * q2.z) + (q1.w * q2.w)
        temp_value = 2 * (dot_product ** 2) - 1
        return math.acos(temp_value)

    def LoadFromDirectory(self, directory_path):
        trajectories = []
        trajectory_files = os.listdir(directory_path)
        for file_name in trajectory_files:
            full_file_path = directory_path + "/" + file_name
            trajectories.append(AnalysisTrajectory(full_file_path))
        return trajectories

if __name__ == '__main__':
    source_dir = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_library_pr2_wheel"
    target_dir = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_library_pr2_wheel_poses"
    converter = JointsToPoseConverter()
    converter.ConvertDirectory(source_dir, target_dir)
