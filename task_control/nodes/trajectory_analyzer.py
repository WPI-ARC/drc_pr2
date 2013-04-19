import math
import random
import os
from dtw import *
from two_arm_trajectory import *
import xml
from xml.etree.ElementTree import *

def euclidean_distance(state1, state2):
    assert(len(state1) == len(state2))
    total = 0.0
    for index in range(len(state1)):
        temp = (state1[index] - state2[index]) ** 2
        total = total + temp
    return math.sqrt(total)

def separate_pose_distance(state1, state2):
    assert(len(state1) == len(state2))
    position_dist = math.sqrt((state1[0] - state2[0])**2 + (state1[1] - state2[1])**2 + (state1[2] - state2[2])**2)
    dot_product = (state1[3] * state2[3]) + (state1[4] * state2[4]) + (state1[5] * state2[5]) + (state1[6] * state2[6])
    orientation_dist = 0.0
    if (dot_product < 0.99999999):
        orientation_dist = math.acos(2 * (dot_product ** 2) - 1)
    return position_dist + orientation_dist

def together_pose_distance(state1, state2):
    assert(len(state1) == len(state2))
    position_dist1 = math.sqrt((state1[0] - state2[0])**2 + (state1[1] - state2[1])**2 + (state1[2] - state2[2])**2)
    dot_product1 = (state1[3] * state2[3]) + (state1[4] * state2[4]) + (state1[5] * state2[5]) + (state1[6] * state2[6])
    orientation_dist1 = 0.0
    if (dot_product1 < 0.99999999):
        orientation_dist1 = math.acos(2 * (dot_product1 ** 2) - 1)
    position_dist2 = math.sqrt((state1[7] - state2[7])**2 + (state1[8] - state2[8])**2 + (state1[9] - state2[9])**2)
    dot_product2 = (state1[10] * state2[10]) + (state1[11] * state2[11]) + (state1[12] * state2[12]) + (state1[13] * state2[13])
    orientation_dist2 = 0.0
    if (dot_product2 < 0.99999999):
        orientation_dist2 = math.acos(2 * (dot_product2 ** 2) - 1)
    return position_dist1 + orientation_dist1 + position_dist2 + orientation_dist2

class TwoArmTrajectoryJointAnalyzer:

    def __init__(self, joint_library_dir, downsampled=True, downsample=50, verbose=False):
        self.evaluator = DTW()
        self.verbose = verbose
        self.downsampled = downsampled
        self.downsample = downsample
        [self.library, self.files] = self.LoadFromDirectory(joint_library_dir)

    def LoadFromDirectory(self, directory_path):
        trajectories = []
        trajectory_files = os.listdir(directory_path)
        for file_name in trajectory_files:
            full_file_path = directory_path + "/" + file_name
            if (self.downsampled):
                raw = TwoArmTrajectory(full_file_path)
                trajectories.append(raw.Downsample(self.downsample))
            else:
                trajectories.append(TwoArmTrajectory(full_file_path))
        return [trajectories, trajectory_files]

    def AnalyzeTogether(self, traj, evaltype):
        if (len(self.library) == 0):
            return [None, None, None]
        fast_dtw_dict = {}
        if (self.downsampled):
            traj = traj.Downsample(self.downsample)
        trajectory_actuals = self.__extract_raw_together__(traj, evaltype)
        for index in range(len(self.library)):
            library_traj = self.library[index]
            if (self.verbose):
                print "Comparing against library trajectory: " + self.files[index] + " [joint]"
            library_actuals = self.__extract_raw_together__(library_traj, evaltype)
            fastcost = self.evaluator.Evaluate(library_actuals, trajectory_actuals, euclidean_distance)
            if (self.verbose):
                print "Calculated cost: " + str(fastcost) + " [together]"
            fast_dtw_dict[str(fastcost)] = library_traj.code
        fast_min_code = sorted(fast_dtw_dict.keys())[0]
        fast_match = fast_dtw_dict[fast_min_code]
        return [fast_match, fast_min_code, traj.code]

    def AnalyzeSeparately(self, traj, evaltype):
        if (len(self.library) == 0):
            return [None, None, None, None, None]
        left_dtw_dict = {}
        right_dtw_dict = {}
        if (self.downsampled):
            traj = traj.Downsample(self.downsample)
        [left_actual, right_actual] = self.__extract_raw_separately__(traj, evaltype)
        for index in range(len(self.library)):
            library_traj = self.library[index]
            if (self.verbose):
                print "Comparing against library trajectory: " + self.files[index] + " [joint]"
            [left_library, right_library] = self.__extract_raw_separately__(library_traj, evaltype)
            leftcost = self.evaluator.Evaluate(left_library, left_actual, euclidean_distance)
            if (self.verbose):
                print "Calculated cost: " + str(leftcost) + " [left]"
            left_dtw_dict[str(leftcost)] = library_traj.code
            rightcost = self.evaluator.Evaluate(right_library, right_actual, euclidean_distance)
            if (self.verbose):
                print "Calculated cost: " + str(rightcost) + " [right]"
            right_dtw_dict[str(rightcost)] = library_traj.code
        left_min_code = sorted(left_dtw_dict.keys())[0]
        left_match = left_dtw_dict[left_min_code]
        right_min_code = sorted(right_dtw_dict.keys())[0]
        right_match = right_dtw_dict[right_min_code]
        return [left_match, left_min_code, right_match, right_min_code, traj.code]

    def __extract_raw_separately__(self, full_trajectory, field):
        raw_left = []
        raw_right = []
        for state in full_trajectory.states:
            if (field == "desired"):
                raw_left.append(state.left.desired)
                raw_right.append(state.right.desired)
            elif (field == "actual"):
                raw_left.append(state.left.actual)
                raw_right.append(state.right.actual)
            elif (field == "error"):
                raw_left.append(state.left.error)
                raw_right.append(state.right.error)
            else:
                raise AttributeError
        return [raw_left, raw_right]

    def __extract_raw_together__(self, full_trajectory, field):
        raw_combined = []
        for state in full_trajectory.states:
            if (field == "desired"):
                raw_left = state.left.desired
                raw_right = state.right.desired
                raw_combined.append(raw_left + raw_right)
            elif (field == "actual"):
                raw_left = state.left.actual
                raw_right = state.right.actual
                raw_combined.append(raw_left + raw_right)
            elif (field == "error"):
                raw_left = state.left.error
                raw_right = state.right.error
                raw_combined.append(raw_left + raw_right)
            else:
                raise AttributeError
        return raw_combined

class TwoArmTrajectoryPoseAnalyzer:

    def __init__(self, pose_library_dir, downsampled=True, downsample=50, verbose=False):
        self.verbose = verbose
        self.evaluator = DTW()
        self.downsampled = downsampled
        self.downsample = downsample
        [self.library, self.files] = self.LoadFromDirectory(pose_library_dir)

    def GetLibraryStats(self):
        library_stats = {}
        library_stats['total'] = len(self.library)
        KB = 0
        KG = 0
        for example in self.library:
            if (example.code == 'KB'):
                KB += 1
            elif (example.code == 'KG'):
                KG += 1
            else:
                print "[WARNING] Unusable example in library"
        library_stats['KB'] = KB
        library_stats['KG'] = KG
        return library_stats

    def RunExtendedLibraryTest(self, KG_only=False, KB_only=False):
        kg_test_results = []
        kb_test_results = []
        for example1 in self.library:
            start = time.clock()
            example1_actuals = self.__extract_raw_together__(example1, "actual")
            fast_dtw_dict = {}
            for example2 in self.library:
                if example2 is not example1:
                    if (KG_only and "KG" in example2.code):
                        example2_actuals = self.__extract_raw_together__(example2, "actual")
                        fastcost = self.evaluator.Evaluate(example1_actuals, example2_actuals, together_pose_distance)
                        fast_dtw_dict[str(fastcost)] = example2.code
                    elif (KB_only and "KB" in example2.code):
                        example2_actuals = self.__extract_raw_together__(example2, "actual")
                        fastcost = self.evaluator.Evaluate(example1_actuals, example2_actuals, together_pose_distance)
                        fast_dtw_dict[str(fastcost)] = example2.code
                    elif (not KB_only and not KG_only):
                        example2_actuals = self.__extract_raw_together__(example2, "actual")
                        fastcost = self.evaluator.Evaluate(example1_actuals, example2_actuals, together_pose_distance)
                        fast_dtw_dict[str(fastcost)] = example2.code
            fast_min_code = sorted(fast_dtw_dict.keys())[0]
            fast_match = fast_dtw_dict[fast_min_code]
            if ("KG" in example1.code):
                kg_test_results.append([fast_match, fast_min_code, example1.code])
            elif ("KB" in example1.code):
                kb_test_results.append([fast_match, fast_min_code, example1.code])
            end = time.clock()
            print "Trajectory evaluation took " + str(end - start) + " seconds"
        return [kg_test_results, kb_test_results]

    def RunLibraryTest(self):
        test_results = []
        for example1 in self.library:
            start = time.clock()
            example1_actuals = self.__extract_raw_together__(example1, "actual")
            fast_dtw_dict = {}
            for example2 in self.library:
                if example2 is not example1:
                    example2_actuals = self.__extract_raw_together__(example2, "actual")
                    fastcost = self.evaluator.Evaluate(example1_actuals, example2_actuals, together_pose_distance)
                    fast_dtw_dict[str(fastcost)] = example2.code
            fast_min_code = sorted(fast_dtw_dict.keys())[0]
            fast_match = fast_dtw_dict[fast_min_code]
            test_results.append([fast_match, fast_min_code, example1.code])
            end = time.clock()
            print "Trajectory evaluation took " + str(end - start) + " seconds"
        return test_results

    def LoadFromDirectory(self, directory_path):
        trajectories = []
        trajectory_files = os.listdir(directory_path)
        for file_name in trajectory_files:
            full_file_path = directory_path + "/" + file_name
            if (self.downsampled):
                raw = TwoArmTrajectory(full_file_path)
                trajectories.append(raw.Downsample(self.downsample))
            else:
                trajectories.append(TwoArmTrajectory(full_file_path))
        return [trajectories, trajectory_files]

    def AnalyzeTogether(self, traj, evaltype):
        if (len(self.library) == 0):
            return ['None', 'None', 'None']
        fast_dtw_dict = {}
        if (self.downsampled):
            traj = traj.Downsample(self.downsample)
        trajectory_actuals = self.__extract_raw_together__(traj, evaltype)
        for index in range(len(self.library)):
            library_traj = self.library[index]
            if (self.verbose):
                print "Comparing against library trajectory: " + self.files[index] + " [pose]"
            library_actuals = self.__extract_raw_together__(library_traj, evaltype)
            fastcost = self.evaluator.Evaluate(library_actuals, trajectory_actuals, together_pose_distance)
            if (self.verbose):
                print "Calculated cost: " + str(fastcost) + " [together]"
            fast_dtw_dict[str(fastcost)] = library_traj.code
        fast_min_code = sorted(fast_dtw_dict.keys())[0]
        fast_match = fast_dtw_dict[fast_min_code]
        return [fast_match, fast_min_code, traj.code]

    def AnalyzeSeparately(self, traj, evaltype):
        if (len(self.library) == 0):
            return [None, None, None, None, None]
        left_dtw_dict = {}
        right_dtw_dict = {}
        if (self.downsampled):
            traj = traj.Downsample(self.downsample)
        [left_actual, right_actual] = self.__extract_raw_separately__(traj, evaltype)
        for index in range(len(self.library)):
            library_traj = self.library[index]
            if (self.verbose):
                print "Comparing against library trajectory: " + self.files[index] + " [pose]"
            [left_library, right_library] = self.__extract_raw_separately__(library_traj, evaltype)
            leftcost = self.evaluator.Evaluate(left_library, left_actual, separate_pose_distance)
            if (self.verbose):
                print "Calculated cost: " + str(leftcost) + " [left]"
            left_dtw_dict[str(leftcost)] = library_traj.code
            rightcost = self.evaluator.Evaluate(right_library, right_actual, separate_pose_distance)
            if (self.verbose):
                print "Calculated cost: " + str(rightcost) + " [right]"
            right_dtw_dict[str(rightcost)] = library_traj.code
        left_min_code = sorted(left_dtw_dict.keys())[0]
        left_match = left_dtw_dict[left_min_code]
        right_min_code = sorted(right_dtw_dict.keys())[0]
        right_match = right_dtw_dict[right_min_code]
        return [left_match, left_min_code, right_match, right_min_code, traj.code]

    def __extract_raw_separately__(self, full_trajectory, field):
        raw_left = []
        raw_right = []
        for state in full_trajectory.states:
            if (field == "desired"):
                raw_left.append(state.left.desired)
                raw_right.append(state.right.desired)
            elif (field == "actual"):
                raw_left.append(state.left.actual)
                raw_right.append(state.right.actual)
            elif (field == "error"):
                raw_left.append(state.left.error)
                raw_right.append(state.right.error)
            else:
                raise AttributeError
        return [raw_left, raw_right]

    def __extract_raw_together__(self, full_trajectory, field):
        raw_combined = []
        for state in full_trajectory.states:
            if (field == "desired"):
                raw_left = state.left.desired
                raw_right = state.right.desired
                raw_combined.append(raw_left + raw_right)
            elif (field == "actual"):
                raw_left = state.left.actual
                raw_right = state.right.actual
                raw_combined.append(raw_left + raw_right)
            elif (field == "error"):
                raw_left = state.left.error
                raw_right = state.right.error
                raw_combined.append(raw_left + raw_right)
            else:
                raise AttributeError
        return raw_combined

