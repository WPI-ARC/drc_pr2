#!/usr/bin/python

#   Python script for anaylsis of task monitor output
#   Calder Phillips-Grafflin - WPI DRC Team 2013
from trajectory_analyzer import *
import subprocess

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

def CheckAssignment(tagged, assigned):
    assert(len(tagged) == len(assigned))
    total_good = 0
    total_bad = 0
    total_correct = 0
    total_incorrect = 0
    correct_good = 0
    correct_bad = 0
    false_negative = 0
    false_positive = 0
    if (tagged == "KG"):
        total_good += 1
        if (assigned == "KG"):
            correct_good += 1
            total_correct += 1
        elif (assigned == "KB"):
            false_negative += 1
            total_incorrect += 1
        else:
            print "WTF just happened?!"
    elif (tagged == "KB"):
        total_bad += 1
        if (assigned == "KG"):
            false_positive += 1
            total_incorrect += 1
        elif (assigned == "KB"):
            correct_bad += 1
            total_correct += 1
        else:
             print "WTF just happened?!"
    else:
        print "WTF just happened?!"
    return {"tg":total_good, "tb":total_bad, "cg":correct_good, "cb":correct_bad, "fp":false_positive, "fn":false_negative, "q":True}

def RunLeaveOneOut():
    print "Running leave-one-out testing on the library"
    path = subprocess.check_output("rospack find task_control", shell=True)
    path = path.strip("\n")
    full_path = path + "/real_trajectory_library/pose/"
    pose_analyzer = TwoArmTrajectoryPoseAnalyzer(full_path, True, 50, False)
    library_stats = pose_analyzer.GetLibraryStats()
    print "Library size is " + str(library_stats['total']) + " trajectories (" + str(library_stats['KG']) + " KG, " + str(library_stats['KB']) + " KB)"
    [kg_library_results, kb_library_results] = pose_analyzer.RunExtendedLibraryTest(False, True)
    print "++++++++++\nDTW match cost statistics:\n++++++++++"
    DisplayExtendedStats(kg_library_results, kb_library_results)
    print "++++++++++\nGeneral library statistics:\n++++++++++"
    ComputeStats(kg_library_results + kb_library_results)

def DisplayExtendedStats(kg_library_results, kb_library_results):
    for result in kg_library_results:
        match_code = result[0]
        real_code = result[2]
        cost = result[1]
        if (match_code == real_code):
            print bcolors.OKGREEN + "KG match assigned with cost: " + cost + bcolors.ENDC
        else:
            print bcolors.FAIL + bcolors.BOLD + "Incorrect KB match assigned with cost: " + cost + bcolors.ENDC
    for result in kb_library_results:
        match_code = result[0]
        real_code = result[2]
        cost = result[1]
        if (match_code == real_code):
            print bcolors.OKGREEN + "KB match assigned with cost: " + cost + bcolors.ENDC
        else:
            print bcolors.FAIL + bcolors.BOLD + "Incorrect KG match assigned with cost: " + cost + bcolors.ENDC

def ProcessChunk(element):
    #print element
    tagged = element[0]
    assigned = element[2]
    return CheckAssignment(tagged, assigned)

def ComputeStats(trajectory_chunks):
    #Analyze the quality trajectories
    combined_stats = []
    total_good = 0
    total_bad = 0
    total_correct_good = 0
    total_correct_bad = 0
    total_false_positive = 0
    total_false_negative = 0
    total_correct = 0
    total_incorrect = 0
    #Collect data
    for element in trajectory_chunks:
        new_stats = ProcessChunk(element)
        #print new_stats
        if (new_stats['q']):
            combined_stats.append(new_stats)
            new_correct = new_stats['cg'] + new_stats['cb']
            new_incorrect = new_stats['fp'] + new_stats['fn']
            total_correct += new_correct
            total_incorrect += new_incorrect
            total_good += new_stats['tg']
            total_bad += new_stats['tb']
            total_correct_good += new_stats['cg']
            total_correct_bad += new_stats['cb']
            total_false_positive += new_stats['fp']
            total_false_negative += new_stats['fn']
    #Display stats for quality trajectories
    print "Total trajectories: " + str(total_good + total_bad)
    print "Total good trajectories: " + str(total_good)
    print "Total bad trajectories: " + str(total_bad)
    print "----------"
    print "Total correctly identified: " + str(total_correct)
    print "Total incorrectly identified: " + str(total_incorrect)
    print "Total false positives: " + str(total_false_positive)
    print "Total false negatives: " + str(total_false_negative)
    print "----------"
    print "Overall percent identified correctly: " + str((float(total_correct) / float(total_good + total_bad)) * 100.0)
    print "Overall percent identified incorrectly: " + str((float(total_incorrect) / float(total_good + total_bad)) * 100.0)
    print "Overall percent false positives: " + str((float(total_false_positive) / float(total_good + total_bad)) * 100.0)
    print "Overall percent false negatives: " + str((float(total_false_negative) / float(total_good + total_bad)) * 100.0)
    print "Percent good identified correctly: " + str((float(total_correct_good) / float(total_good)) * 100.0)
    print "Percent bad identified correctly: " + str((float(total_correct_bad) / float(total_bad)) * 100.0)
    print "Percent good identified incorrectly: " + str((float(total_false_negative) / float(total_good)) * 100.0)
    print "Percent bad identified incorrectly: " + str((float(total_false_positive) / float(total_bad)) * 100.0)

if __name__ == '__main__':
    RunLeaveOneOut()
