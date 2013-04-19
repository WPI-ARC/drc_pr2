from two_arm_trajectory import *
from trajectory_analyzer import *
from dtw import *
import subprocess

if __name__ == '__main__':
    path = subprocess.check_output("rospack find task_control", shell=True)
    path = path.strip("\n")
    traj_path = path + "/simulation_trajectory_library/pose/"
    t_file_1 = traj_path + "test_09|01|2013_22|44|25_pr2_trajectory.xml"
    traj_1 = TwoArmTrajectory(t_file_1)
    #Test whole trajectory
    evaluator = TwoArmTrajectoryPoseAnalyzer(traj_path, True, 50)
    match = evaluator.AnalyzeTogether(traj_1, "actual")
    print match
    
