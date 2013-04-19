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
from two_arm_trajectory import *
from trajectory_analyzer import *
from evaluator import *
from joint_to_pose_converter import *


class Evaluator:

    def __init__(self, library_path, POSE_ONLY=True, cost_threshold=-1.0, downsampled=True, downsample=50, verbose=False, test_mode=False):
        self.cost_threshold = cost_threshold
        self.testing = test_mode
        self.pose_analyzer = TwoArmTrajectoryPoseAnalyzer(library_path + "/pose/", downsampled, downsample, verbose)
        if (POSE_ONLY == False):
            self.joint_analyzer = TwoArmTrajectoryJointAnalyzer(library_path + "/joints/", downsampled, downsample, verbose)
        self.correct = 0
        self.false_pos = 0
        self.false_neg = 0

    def Evaluate(self, joint_traj_file, pose_traj_file):
        #Evaluate a given trajectory by joint and pose states
        print "Evaluating trajectory:\n" + joint_traj_file + " [joint]\n" + pose_traj_file + " [pose]"
        real_joint = TwoArmTrajectory(joint_traj_file)
        real_pose = TwoArmTrajectory(pose_traj_file)
        #Find best match [together] joint trajectory
        JTT = self.joint_analyzer.AnalyzeTogether(real_joint, "actual")
        #Find best match [separate] joint trajectory
        JTS = self.joint_analyzer.AnalyzeSeparately(real_joint, "actual")
        #Find best match [together] pose trajectory
        PTT = self.pose_analyzer.AnalyzeTogether(real_pose, "actual")
        #Find best match [separate] pose trajectory
        PTS = self.pose_analyzer.AnalyzeSeparately(real_pose, "actual")
        #Determine overall best match
        best = self.__find_best__(JTT, JTS, PTT, PTS)
        #Classify error based on best match
        success_code = self.__classify__(best[0], best[1])
        result_dict = {'code':success_code, 'cost':best[1], 'match':best[0], 'joint file':joint_traj_file, 'pose file':pose_traj_file}
        return result_dict

    def EvaluatePoseOnly(self, pose_traj_file):
        #Evaluate a given trajectory by joint and pose states
        #print "Evaluating trajectory:\n" + pose_traj_file + " [pose]"
        real_pose = TwoArmTrajectory(pose_traj_file)
        #Find best match [together] pose trajectory
        PTT = self.pose_analyzer.AnalyzeTogether(real_pose, "actual")
        #Classify error based on best match
        success_code = self.__classify__(PTT[0], PTT[1])
        result_dict = {'code':success_code, 'cost':PTT[1], 'match':PTT[0], 'joint file':None, 'pose file':pose_traj_file}
        return result_dict

    def __find_best_pose__(self, PTT, PTS):
        match_dict = {}
        match_dict[PTT[1]] = PTT[0]
        match_dict[PTS[1]] = PTS[0]
        match_dict[PTS[2]] = PTS[3]
        min_match_code = sorted(match_dict.keys())[0]
        return [match_dict[min_match_code], min_match_code]

    def __find_best__(self, JTT, JTS, PTT, PTS):
        match_dict = {}
        match_dict[JTT[1]] = JTT[0]
        match_dict[JTS[1]] = JTS[0]
        match_dict[JTS[3]] = JTS[3]
        match_dict[PTT[1]] = PTT[0]
        match_dict[PTS[1]] = PTS[0]
        match_dict[PTS[3]] = PTS[3]
        min_match_code = sorted(match_dict.keys())[0]
        return [match_dict[min_match_code], min_match_code]

    def __classify__(self, best_match, best_match_cost):
        if (best_match == None):
            return "NO LIBRARY"
        #First, check if the cost value is within the given threshold bounds
        if (self.cost_threshold > 0.0):
            #Valid threshold
            if ("KG" in best_match):
                if (float(best_match_cost) <= self.cost_threshold):
                    return "COMPLETE"
            return "MISALIGN"
        else:
            #No threshold
            if ("KG" in best_match):
                return "COMPLETE"
            else:
                return "MISALIGN"
        
