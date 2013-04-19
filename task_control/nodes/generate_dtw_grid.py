import math
import random
import os
from dtw import *
import subprocess
from two_arm_trajectory import *
import xml
from xml.etree.ElementTree import *

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

class MatrixGenerator:

    def __init__(self, pose_library_dir, downsampled=True, downsample=50, verbose=False):
        self.verbose = verbose
        self.evaluator = DTW()
        self.downsampled = downsampled
        self.downsample = downsample
        [self.library, self.files] = self.LoadFromDirectory(pose_library_dir)

    def GetDTWMatrix(self, example1, example2):
        example1_actuals = self.__extract_raw_together__(example1, "actual")
        example2_actuals = self.__extract_raw_together__(example2, "actual")
        [cost, matrix] = self.evaluator.Evaluate(example1_actuals, example2_actuals, together_pose_distance, True)
        print "Cost = " + str(cost)
        return matrix

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

if __name__ == '__main__':
    path = subprocess.check_output("rospack find task_control", shell=True)
    path = path.strip("\n")
    full_path = path + "/real_trajectory_library/pose/"
    new_gen = MatrixGenerator(full_path, True, 50, False)
    ex_matrix_0 = new_gen.GetDTWMatrix(new_gen.library[0],new_gen.library[0])
    ex_matrix_1 = new_gen.GetDTWMatrix(new_gen.library[0],new_gen.library[2])
    ex_matrix_2 = new_gen.GetDTWMatrix(new_gen.library[0],new_gen.library[1])
    ex_matrix_3 = new_gen.GetDTWMatrix(new_gen.library[0],new_gen.library[524])
    csv_string0 = ""
    for index in range(len(ex_matrix_0)):
        line_string = str(ex_matrix_0[index]).lstrip("[").rstrip("]")
        #print ex_matrix_1[index][0]
        #print line_string
        csv_string0 += line_string + "\n"
    csv_file0 = open("dtw0.csv", "w")
    csv_file0.write(csv_string0)
    csv_file0.close()
    csv_string1 = ""
    for index in range(len(ex_matrix_1)):
        line_string = str(ex_matrix_1[index]).lstrip("[").rstrip("]")
        #print ex_matrix_1[index][0]
        #print line_string
        csv_string1 += line_string + "\n"
    csv_file1 = open("dtw1.csv", "w")
    csv_file1.write(csv_string1)
    csv_file1.close()
    csv_string2 = ""
    for index in range(len(ex_matrix_2)):
        line_string = str(ex_matrix_2[index]).lstrip("[").rstrip("]")
        #print ex_matrix_1[index][0]
        #print line_string
        csv_string2 += line_string + "\n"
    csv_file2 = open("dtw2.csv", "w")
    csv_file2.write(csv_string2)
    csv_file2.close()
    csv_string3 = ""
    for index in range(len(ex_matrix_3)):
        line_string = str(ex_matrix_3[index]).lstrip("[").rstrip("]")
        #print ex_matrix_1[index][0]
        #print line_string
        csv_string3 += line_string + "\n"
    csv_file3 = open("dtw3.csv", "w")
    csv_file3.write(csv_string3)
    csv_file3.close()
