import math
import random
import os
from dtw import *
from trajectory_analysis import *
import xml
from xml.etree.ElementTree import *

class AnalysisTrajectory:

    def __init__(self, filename):
        self.desired_positions = []
        self.desired_velocities = []
        self.desired_accelerations = []
        self.actual_positions = []
        self.actual_velocities = []
        self.actual_accelerations = []
        self.error_positions = []
        self.error_velocities = []
        self.error_accelerations = []
        self.times = []
        self.code = ""
        self.REAL = True
        self.load(filename)

    def load(self, filename):
        traj_file = open(filename, "r")
        traj_tree = ElementTree().parse(traj_file)
        #load the code
        self.code = traj_tree[0][3].text.strip("\n")
        for entry in traj_tree[2]:
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
            velocity = chunks2[1].split(": ")[1]
            acceleration = chunks2[2].split(": ")[1]
            self.desired_positions.append(self.Str2List(position))
            self.desired_velocities.append(self.Str2List(velocity))
            self.desired_accelerations.append(self.Str2List(acceleration))
            #load the actual
            chunks3 = entry[2].text.strip("\n").split("\n")
            position = chunks3[0].split(": ")[1]
            velocity = chunks3[1].split(": ")[1]
            acceleration = chunks3[2].split(": ")[1]
            self.actual_positions.append(self.Str2List(position))
            self.actual_velocities.append(self.Str2List(velocity))
            self.actual_accelerations.append(self.Str2List(acceleration))
            #load the error
            chunks4 = entry[3].text.strip("\n").split("\n")
            position = chunks4[0].split(": ")[1]
            velocity = chunks4[1].split(": ")[1]
            acceleration = chunks4[2].split(": ")[1]
            self.error_positions.append(self.Str2List(position))
            self.error_velocities.append(self.Str2List(velocity))
            self.error_accelerations.append(self.Str2List(acceleration))
        traj_file.close()

    def Str2List(self, raw):
        raw = raw.lstrip("(").rstrip(")")
        parts = raw.split(", ")
        cleaned = []
        for part in parts:
            cleaned.append(float(part))
        return cleaned

    def AddAWGN(self, position_std_dev, velocity_std_dev):
        if (position_std_dev == 0.0 and velocity_std_dev == 0.0):
            #print "Adding noise to a trajectory [none] and [none]"
            return
        elif (position_std_dev == 0.0):
            #print "Adding noise to a trajectory [none] and [velocity]"
            self.REAL = False
            for index in range(len(self.actual_positions)):
                #Add noise to every value of actual velocity and error velocity
                for item_index in range(len(self.actual_velocities[index])):
                    noise = random.gauss(0.0, velocity_std_dev)
                    self.actual_velocities[index][item_index] = self.actual_velocities[index][item_index] + noise
                    self.error_velocities[index][item_index] = self.error_velocities[index][item_index] + (-noise)
            return
        elif (velocity_std_dev == 0.0):
            #print "Adding noise to a trajectory [position] and [none]"
            self.REAL = False
            for index in range(len(self.actual_positions)):
                #Add noise to every value of actual position and error position
                for item_index in range(len(self.actual_positions[index])):
                    noise = random.gauss(0.0, position_std_dev)
                    self.actual_positions[index][item_index] = self.actual_positions[index][item_index] + noise
                    self.error_positions[index][item_index] = self.error_positions[index][item_index] + (-noise)
            return
        #print "Adding noise to a trajectory [position] and [velocity]"
        self.REAL = False
        for index in range(len(self.actual_positions)):
            #Add noise to every value of actual position and error position
            for item_index in range(len(self.actual_positions[index])):
                noise = random.gauss(0.0, position_std_dev)
                self.actual_positions[index][item_index] = self.actual_positions[index][item_index] + noise
                self.error_positions[index][item_index] = self.error_positions[index][item_index] + (-noise)
            #Add noise to every value of actual velocity and error velocity
            for item_index in range(len(self.actual_velocities[index])):
                noise = random.gauss(0.0, velocity_std_dev)
                self.actual_velocities[index][item_index] = self.actual_velocities[index][item_index] + noise
                self.error_velocities[index][item_index] = self.error_velocities[index][item_index] + (-noise)

class PAOnlineTrajectoryAnalyzer:
    def __init__(self, library_dir, max_x, max_y, distance_fn=None):
        self.library_path = library_dir
        self.TrajectoryLength = 0
        self.evaluators = {}
        self.costs = {}
        self.library = self.LoadFromDirectory(self.library_path)
        for library_traj in self.library:
            self.evaluators[library_traj.code] = PreAllocDTW(max_x, max_y, library_traj.error_positions, distance_fn)
            self.costs[library_traj.code] = -1.0

    def LoadFromDirectory(self, directory_path):
        trajectories = []
        trajectory_files = os.listdir(directory_path)
        for file_name in trajectory_files:
            full_file_path = directory_path + "/" + file_name
            trajectories.append(AnalysisTrajectory(full_file_path))
        return trajectories

    def AddStateToTrajectory(self, new_state):
        for evaluator in self.evaluators.keys():
            self.evaluators[evaluator].Update(new_state, None)
        self.TrajectoryLength = self.TrajectoryLength + 1

    def UpdateTrajectoryDTW(self):
        new_costs = {}
        if (self.TrajectoryLength > 0):
            #Compute DTW cost for all examples in library
            for evaluator in self.evaluators.keys():
                cost = self.evaluators[evaluator].ReEvaluate()
                new_costs[evaluator] = cost
            self.costs = new_costs
        else:
            raise AttributeError

    def GetState(self):
        names = self.costs.keys()
        costs = []
        for name in names:
            costs.append(self.costs[name])
        return names, costs
        

class OnlineTrajectoryAnalyzer:

    def __init__(self, library_dir, distance_fn=None):
        self.library_path = library_dir
        self.evaluator = DTW(distance_fn)
        self.library = self.LoadFromDirectory(self.library_path)
        self.current_trajectory = []
        self.trajectory_DTW = {}
        for library_traj in self.library:
            self.trajectory_DTW[library_traj.code] = -1.0

    def LoadFromDirectory(self, directory_path):
        trajectories = []
        trajectory_files = os.listdir(directory_path)
        for file_name in trajectory_files:
            full_file_path = directory_path + "/" + file_name
            trajectories.append(AnalysisTrajectory(full_file_path))
        return trajectories

    def AddStateToTrajectory(self, new_state):
        if (len(self.current_trajectory) > 0):
            assert len(self.current_trajectory[0]) == len(new_state)
            self.current_trajectory.append(new_state)
        else:
            self.current_trajectory.append(new_state)

    def UpdateTrajectoryDTW(self):
        update_DTW = {}
        if (len(self.current_trajectory) > 0):
            #Compute DTW cost for all examples in library
            for library_traj in self.library:
                poscost = self.evaluator.Evaluate(self.current_trajectory, library_traj.error_positions)
                update_DTW[library_traj.code] = poscost
            self.trajectory_DTW = update_DTW
        else:
            raise AttributeError

    def GetState(self):
        names = self.trajectory_DTW.keys()
        costs = []
        for name in names:
            costs.append(self.trajectory_DTW[name])
        return names, costs

class TrajectoryAnalyzer:

    def __init__(self, library_dir, distance_fn=None):
        self.library_path = library_dir
        self.evaluator = DTW(distance_fn)
        self.library = self.LoadFromDirectory(self.library_path)

    def LoadFromDirectory(self, directory_path):
        trajectories = []
        trajectory_files = os.listdir(directory_path)
        for file_name in trajectory_files:
            full_file_path = directory_path + "/" + file_name
            trajectories.append(AnalysisTrajectory(full_file_path))
        return trajectories

    def AnalyzeDirectory(self, trajectory_path, fast=False, verbose=False, PSD=0.0, VSD=0.0):
        trajectories_to_analyze = self.LoadFromDirectory(trajectory_path)
        results = []
        for traj in trajectories_to_analyze:
            traj.AddAWGN(PSD, VSD)
            try:
                result = self.AnalyzeTrajectory(traj, fast)
                results.append(result)
                if (verbose):
                    print result
            except:
                print "Could not analyze trajectory"
        return results

    def GenerateDirectoryData(self, trajectory_path, save_file, fast=False, PSD=0.0, VSD=0.0):
        trajectories_to_analyze = self.LoadFromDirectory(trajectory_path)
        results = {}
        for traj in trajectories_to_analyze:
            traj.AddAWGN(PSD, VSD)
            try:
                results[traj.code] = self.GenerateTrajectoryResults(traj, fast)
            except:
                print "Could not analyze trajectory"
        library_labels = results[results.keys()[0]][0].keys()
        print library_labels
        file_string = ""
        for label in library_labels:
            file_string = file_string + "," + label
        for traj_code in results.keys():
            code_line = traj_code
            for label in library_labels:
                code_line = code_line + "," + str(results[traj_code][0][label])
            file_string = file_string + "\n" + code_line
        out_file = open(save_file, "w")
        out_file.write(file_string)
        out_file.close()
        print "Data generation complete"

    def GenerateTrajectoryData(self, traj):
        #Evaluate by position and velocity info for the actual trajectory
        actual_dtw_pos_dict = {}
        actual_dtw_vel_dict = {}
        for library_traj in self.library:
            poscost = self.evaluator.Evaluate(traj.actual_positions, library_traj.actual_positions)
            actual_dtw_pos_dict[library_traj.code] = poscost
            velcost = self.evaluator.Evaluate(traj.actual_velocities, library_traj.actual_velocities)
            actual_dtw_vel_dict[library_traj.code] = velcost
        #Evaluate by position and velocity for the error trajectory
        error_dtw_pos_dict = {}
        error_dtw_vel_dict = {}
        for library_traj in self.library:
            poscost = self.evaluator.Evaluate(traj.error_positions, library_traj.error_positions)
            error_dtw_pos_dict[library_traj.code] = poscost
            velcost = self.evaluator.Evaluate(traj.error_velocities, library_traj.error_velocities)
            error_dtw_vel_dict[library_traj.code] = velcost
        return [actual_dtw_pos_dict, actual_dtw_vel_dict, error_dtw_pos_dict, error_dtw_vel_dict]

    def GenerateTrajectoryDataFast(self, traj):
        fast_dtw_dict = {}
        for library_traj in self.library:
            fastcost = self.evaluator.Evaluate(traj.error_positions, library_traj.error_positions)
            fast_dtw_dict[library_traj.code] = fastcost 
        return [None, None, fast_dtw_dict, None]

    def GenerateTrajectoryResults(self, traj, FAST):
        results = None
        if FAST:
            results = self.GenerateTrajectoryDataFast(traj)
        else:
            results = self.GenerateTrajectoryData(traj)
        return results

    def AnalyzeTrajectory(self, traj, FAST):
        #FAST MODE - evaluate only by error position:
        if FAST:
            fast_dtw_dict = {}
            for library_traj in self.library:
                fastcost = self.evaluator.Evaluate(traj.error_positions, library_traj.error_positions)
                fast_dtw_dict[str(fastcost)] = library_traj.code
            fast_min_code = sorted(fast_dtw_dict.keys())[0]
            fast_match = fast_dtw_dict[fast_min_code]
            return ["N/A", fast_match, traj.code]
        #Evaluate by position and velocity info for the actual trajectory
        actual_dtw_pos_dict = {}
        for library_traj in self.library:
            poscost = self.evaluator.Evaluate(traj.actual_positions, library_traj.actual_positions)
            actual_dtw_pos_dict[str(poscost)] = library_traj.code
        #Evaluate by position and velocity for the error trajectory
        error_dtw_pos_dict = {}
        for library_traj in self.library:
            poscost = self.evaluator.Evaluate(traj.error_positions, library_traj.error_positions)
            error_dtw_pos_dict[str(poscost)] = library_traj.code
        #Find best-matching actual trajectory
        actual_min_pos_code = sorted(actual_dtw_pos_dict.keys())[0]
        actual_match = actual_dtw_pos_dict[actual_min_pos_code]
        #Find best-matching error trajectory
        error_min_pos_code = sorted(error_dtw_pos_dict.keys())[0]
        error_match = error_dtw_pos_dict[error_min_pos_code]
        return [actual_match, error_match, traj.code]

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
                fastcost = self.evaluator.Evaluate(traj.error_positions, library_traj.error_positions)
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
                cost = self.evaluator.Evaluate(self.library[index].error_positions, self.library[compare].error_positions)
                index_results[self.library[compare].code] = cost
            results[self.library[index].code] = index_results
        return results


