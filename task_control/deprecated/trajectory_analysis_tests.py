from trajectory_analysis import *
from pose_trajectory_analysis import *

def checkPair(set_1, set_2):
    if (len(set_1) != len(set_2)):
        return 0
    elif (len(set_1) == 2):
        if (set_1[0] == set_2[0]):
            return 1
        else:
            return 0
    else:
        if (set_1[0] == set_2[0] and set_1[1] == set_2[1]):
            return 1
        else:
            return 0

def checkTriplet(triplet):
    am_split = triplet[0].split(" ")
    em_split = triplet[1].split(" ")
    rl_split = triplet[2].split(" ")
    actual_match = checkPair(am_split, rl_split)
    error_match = checkPair(em_split, rl_split)
    return [actual_match, error_match]

def check_real():
    library_path = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_library_pr2"
    trajectory_path = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_examples_pr2"
    analyzer = TrajectoryAnalyzer(library_path)
    print "Loaded"
    print "Running test on real PR2 data with no noise"
    results1 = analyzer.AnalyzeDirectory(trajectory_path, False, True, 0.0, 0.0)
    print "Analysis complete"
    total = len(results1)
    a_correct = 0
    e_correct = 0
    for triplet in results1:
        a_correct = a_correct + checkTriplet(triplet)[0]
        e_correct = e_correct + checkTriplet(triplet)[1]
    a_percent_correct = (float(a_correct) / float(total)) * 100.0
    e_percent_correct = (float(e_correct) / float(total)) * 100.0
    print "[Actual] Success rate:", a_percent_correct
    print "[Error] Success rate:", e_percent_correct
    print "Running test on real PR2 data with noise"
    results1 = analyzer.AnalyzeDirectory(trajectory_path, False, True, 0.03, 0.03)
    print "Analysis complete"
    total = len(results1)
    a_correct = 0
    e_correct = 0
    for triplet in results1:
        a_correct = a_correct + checkTriplet(triplet)[0]
        e_correct = e_correct + checkTriplet(triplet)[1]
    a_percent_correct = (float(a_correct) / float(total)) * 100.0
    e_percent_correct = (float(e_correct) / float(total)) * 100.0
    print "[Actual] Success rate:", a_percent_correct
    print "[Error] Success rate:", e_percent_correct

def characterize_real():
    library_path = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_library_pr2"
    trajectory_path = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_examples_pr2"
    analyzer = TrajectoryAnalyzer(library_path)
    print "Loaded"
    print "Running test on real PR2 data"
    analyzer.GenerateDirectoryData(trajectory_path, "pr2_basic.csv", False, 0.0, 0.0)
    print "Repeating test with noise added"
    analyzer.GenerateDirectoryData(trajectory_path, "pr2_noise.csv", False, 0.03, 0.03)

def test_run():
    library_path = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_library_sim"
    trajectory_path = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_examples_sim"
    analyzer = TrajectoryAnalyzer(library_path)
    print "Loaded"
    '''
    print "Running FAST MODE test"
    results1 = analyzer.AnalyzeDirectory(trajectory_path, False, True)
    print "Analysis complete"
    total = len(results1)
    a_correct = 0
    e_correct = 0
    for triplet in results1:
        a_correct = a_correct + checkTriplet(triplet)[0]
        e_correct = e_correct + checkTriplet(triplet)[1]
    a_percent_correct = (float(a_correct) / float(total)) * 100.0
    e_percent_correct = (float(e_correct) / float(total)) * 100.0
    print "[Actual] Success rate:", a_percent_correct
    print "[Error] Success rate:", e_percent_correct
    
    print "Running FAST MODE test with NOISE"
    results2 = analyzer.AnalyzeDirectory(trajectory_path, False, True, 0.03, 0.03)
    print "Analysis complete"
    total = len(results2)
    a_correct = 0
    e_correct = 0
    for triplet in results2:
        a_correct = a_correct + checkTriplet(triplet)[0]
        e_correct = e_correct + checkTriplet(triplet)[1]
    a_percent_correct = (float(a_correct) / float(total)) * 100.0
    e_percent_correct = (float(e_correct) / float(total)) * 100.0
    print "[Actual] Success rate:", a_percent_correct
    print "[Error] Success rate:", e_percent_correct
    '''
    analyzer.GenerateDirectoryData(trajectory_path, "basic.csv", False, 0.0, 0.0)
    analyzer.GenerateDirectoryData(trajectory_path, "noise.csv", False, 0.03, 0.03)

def check_codes(code_1, code_2):
    code1 = code_1.split(" ")[0]
    code2 = code_2.split(" ")[0]
    if (code1 == code2):
        return 1
    else:
        return 0

def min_check_real():
    library_path = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_library_pr2_wheel"
    analyzer = TrajectoryAnalyzer(library_path)
    print "Loaded"
    print "Running test on real PR2 data with no noise"
    results1 = analyzer.MinimalAnalyzeDirectory(library_path)
    print "Analysis complete"
    total = len(results1)
    e_correct = 0
    for pair in results1:
        e_correct = check_codes(pair)
    e_percent_correct = (float(e_correct) / float(total)) * 100.0
    print "[Error] Success rate:", e_percent_correct

def RunL1O():
    data_path = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_library_pr2_wheel"
    save_file = "LeaveOneOutWheel.csv"
    analyzer = TrajectoryAnalyzer(data_path)
    print "Running a leave-one-out test on selected data"
    results = analyzer.LeaveOneOut()
    print "Results:"
    labels = sorted(results.keys())
    print labels
    file_string = ""
    for label in labels:
        file_string = file_string + "," + label
    for traj_code in labels:
        code_line = traj_code
        for label in labels:
            code_line = code_line + "," + str(results[traj_code][label])
        file_string = file_string + "\n" + code_line
    out_file = open(save_file, "w")
    out_file.write(file_string)
    out_file.close()

def min_check_real_poses():
    library_path = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_library_pr2_wheel_poses_selected"
    analyzer = PoseTrajectoryAnalyzer(library_path)
    print "Loaded"
    print "Running test on real PR2 pose data with no noise"
    results1 = analyzer.MinimalAnalyzeDirectory(library_path)
    print "Analysis complete"
    total = len(results1)
    e_correct = 0
    for pair in results1:
        e_correct = check_codes(pair[0], pair[1])
    e_percent_correct = (float(e_correct) / float(total)) * 100.0
    print "[Error] Success rate:", e_percent_correct

def RunL1O_poses():
    data_path = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_library_pr2_wheel_poses_selected"
    save_file = "LeaveOneOutWheelPosesSelected.csv"
    analyzer = PoseTrajectoryAnalyzer(data_path)
    print "Running a leave-one-out test on selected pose data"
    results = analyzer.LeaveOneOut()
    print "Results:"
    labels = sorted(results.keys())
    print labels
    file_string = ""
    for label in labels:
        file_string = file_string + "," + label
    for traj_code in labels:
        code_line = traj_code
        for label in labels:
            code_line = code_line + "," + str(results[traj_code][label])
        file_string = file_string + "\n" + code_line
    out_file = open(save_file, "w")
    out_file.write(file_string)
    out_file.close()

if __name__ == '__main__':
    #characterize_real()
    #min_check_real()
    #RunL1O()
    min_check_real_poses()
    RunL1O_poses()
    #check_real()
    #test_run()
