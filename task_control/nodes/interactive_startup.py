#!/usr/bin/python

#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team

import roslib; roslib.load_manifest('task_control')
import rospy
import math
import time
import subprocess
import os

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

if __name__ == '__main__':
    print bcolors.BOLD + bcolors.CYAN + "[TASK CONTROL] Running interactive task controller startup...\n"
    print "This allows you to interactively set the control parameters of the task control system:\n"
    print "\"task_control/safety\" - sets the maximim allowed joint distance between the starting configuration and the begining of a commanded trajectory"
    print "\"task_control/threshold\" - sets the cost threshold over which a trajectory will automatically be identified as MISALIGN"
    print "\"task_control/mode\" - set between REAL and SIMULATION modes, which controls which trajectory libraries are used"
    print "\"task_control/enforce\" - determines if the task controller will terminate trajectory execution at the expected completion time"
    print "\"task_control/testing\" - determines if the task controller will compute statistics on assessment correctness (this requires ground truth knowledge of executed trajectories to be meaningful)"
    print "\"task_control/interactive\" - determines if the task controller will prompt for manual tagging of trajectories after they are executed"
    print "---------" + bcolors.ENDC
    print bcolors.OKBLUE + "\nThere are three basic configurations of parameters and values:\n"
    print bcolors.BOLD + "Simulation:" + bcolors.ENDC + bcolors.OKBLUE
    print "\"task_control/safety\" = 50.0 (float)"
    print "\"task_control/threshold\" = -1.0 (float)"
    print "\"task_control/mode\" = simulation (float)"
    print "\"task_control/enforce\" = true (float)"
    print "\"task_control/testing\" = false (float)"
    print "\"task_control/interactive\" = false (float)\n"
    print bcolors.BOLD + "Real robot (training):" + bcolors.ENDC + bcolors.OKBLUE
    print "\"task_control/safety\" = 50.0 (float)"
    print "\"task_control/threshold\" = -1.0 (float)"
    print "\"task_control/mode\" = real (string)"
    print "\"task_control/enforce\" = false (boolean)"
    print "\"task_control/testing\" = true (boolean)"
    print "\"task_control/interactive\" = false (boolean)\n"
    print bcolors.BOLD + "Real robot (testing):" + bcolors.ENDC + bcolors.OKBLUE
    print "\"task_control/safety\" = 50.0 (float)"
    print "\"task_control/threshold\" = -1.0 (float)"
    print "\"task_control/mode\" = real (string)"
    print "\"task_control/enforce\" = false (boolean)"
    print "\"task_control/testing\" = true (boolean)"
    print "\"task_control/interactive\" = true (boolean)\n"
    print bcolors.BOLD + "Real robot (operation):" + bcolors.ENDC + bcolors.OKBLUE
    print "\"task_control/safety\" = 5.0 (float)"
    print "\"task_control/threshold\" = 15.0 (float)"
    print "\"task_control/mode\" = real (string)"
    print "\"task_control/enforce\" = true (boolean)"
    print "\"task_control/testing\" = false (boolean)"
    print "\"task_control/interactive\" = false (boolean)" + bcolors.ENDC
    #Interactively ask the user about parameters to set
    print bcolors.CYAN + bcolors.BOLD + "\nSelect a configuration to load:\n" + bcolors.ENDC + bcolors.CYAN
    print "1) Simulation - you MUST select this when using Gazebo!"
    print "2) Real robot training - select when observing the robot to build new trajectory libraries"
    print "3) Real robot testing - select when you want debug information about trajectory assessment correctness (this assumes you are tagging your trajectories!)"
    print "4) Real robot operation - select this when you are running the robot for real"
    print "5) Custom"
    choice = raw_input(bcolors.BOLD + bcolors.QUESTION + "\nEnter your selection (1,2,3,4,5): " + bcolors.ENDC)
    print "You chose " + str(int(choice))
