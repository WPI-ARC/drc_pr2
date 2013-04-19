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

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    QUESTION = '\033[90m'
    FAIL = '\033[91m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    ENDC = '\033[0m'

class Evaluator:

    def __init__(self, library_path, POSE_ONLY=True, cost_threshold=-1.0, downsampled=True, downsample=50, verbose=False, test_mode=False, full_verbose=False):
        self.cost_threshold = cost_threshold
        self.testing = test_mode
        self.verbose = verbose
        self.pose_analyzer = TwoArmTrajectoryPoseAnalyzer(library_path + "/pose/", downsampled, downsample, full_verbose)
        self.correct = 0
        self.false_pos = 0
        self.false_neg = 0
        self.unable = 0

    def GetLibraryStats(self):
        return self.pose_analyzer.GetLibraryStats()

    def cleanup(self):
        if (self.testing):
            summary_string = "Total correct: " + str(self.correct) + " Total false positives: " + str(self.false_pos) + " Total false negatives: " + str(self.false_neg)
            print bcolors.CYAN + "[DEBUG] (Testing mode) " + summary_string + bcolors.ENDC

    def RunLibraryTest(self):
        return self.pose_analyzer.RunLibraryTest()

    def Evaluate(self, pose_traj_file):
        start = time.clock()
        #Evaluate a given trajectory by joint and pose states
        #print "Evaluating trajectory:\n" + pose_traj_file + " [pose]"
        real_pose = TwoArmTrajectory(pose_traj_file)
        #Find best match [together] pose trajectory
        PTT = self.pose_analyzer.AnalyzeTogether(real_pose, "actual")
        #Classify error based on best match
        success_code = self.__classify__(PTT[0], PTT[1])
        result_dict = {'code':success_code, 'cost':PTT[1], 'match':PTT[0], 'joint file':None, 'pose file':pose_traj_file, "correct":"Unknown"}
        if (self.testing):
            correct = self.__check_test__(real_pose.code, result_dict)
            result_dict["correct"] = correct
        end = time.clock()
        if (self.verbose):
            print bcolors.CYAN + "[DEBUG] Trajectory evaluation took " + str(end - start) + " seconds" + bcolors.ENDC
        return result_dict

    def __check_correctness__(self, actual_code, match_code):
        try:
            if ('KG' in actual_code and 'KG' in match_code):
                return 0
            elif ('KG' in actual_code):
                return -1
            elif ('KG' in match_code):
                return 1
            return 0
        except:
            return 2

    def __check_test__(self, traj_code, result_dict):
        #Gather statistical data on match accuracy
        result_string = ""
        correctness = self.__check_correctness__(traj_code, result_dict['match'])
        correct = "Unknown"
        if (correctness == 0):
            self.correct += 1
            result_string += "Trajectory identified correctly"
            correct = "Correct"
        elif (correctness == 1):
            self.false_pos += 1
            result_string += "Trajectory identified incorrectly (false positive)"
            correct = "False positive"
        elif (correctness == -1):
            self.false_neg += 1
            result_string += "Trajectory identified incorrectly (false negative)"
            correct = "False negative"
        else:
            self.unable += 1
            result_string += "Unable to check trajectory identification!"
        print bcolors.CYAN + "[DEBUG] (Testing mode) " + result_string + bcolors.ENDC
        return correct

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
        
