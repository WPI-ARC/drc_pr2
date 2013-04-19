#!/usr/bin/python

#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team
#
#   Tests the online_dtw system with a known trajectory
#

import roslib; roslib.load_manifest('arm_plan_pkg')
import rospy
import time
from trajectory_analysis import *

from std_msgs.msg import String
from arm_plan_pkg.msg import *

class DTWTester:

    def __init__(self):
        rospy.init_node('test_arm_online_dtw_test', anonymous=False)
        print "Running test of online DTW system"
        test_trajectory = AnalysisTrajectory("/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_library_pr2_selected/30|11|2012_01|21|36_trajectory.xml")
        #Setup the publishers
        self.state_pub = rospy.Publisher("test_arm_online_dtw/update", DTWUpdate)
        control = True
        for state in test_trajectory.error_positions:
            raw_input("press enter to continue...")
            self.state_pub.publish(state)
        print "Test finished"

if __name__ == '__main__':
    DTWTester()
