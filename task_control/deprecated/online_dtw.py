#!/usr/bin/python

#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team
#
#   Uses Dynamic Time Warping to match the provided trajectory
#   against the set library of example trajectories. This is
#   performed 'online' as new states are recieved.
#

import roslib; roslib.load_manifest('arm_plan_pkg')
import rospy
import time
from trajectory_analysis import *

from std_msgs.msg import String
from arm_plan_pkg.msg import *

class DTWManager:

    def __init__(self, node_prefix, library_path):
        rospy.init_node(node_prefix + '_online_dtw', anonymous=False)
        print "Loading " + node_prefix + "_online_dtw node..."
        #Setup underlying DTW evaluator
        self.evaluator = PAOnlineTrajectoryAnalyzer(library_path, 400, 400, None)
        print "...Loaded PreAllocatedTrajectoryAnalyzer"
        #Setup the publishers
        self.state_pub = rospy.Publisher(node_prefix + "_online_dtw/matches", DTWMatch)
        print "Loaded match publisher"
        #Setup the subscriber callback
        self.handler = rospy.Subscriber(node_prefix + "_online_dtw/update", DTWUpdate, self.handler)
        print "Loaded update subscriber"
        rate = rospy.Rate(rospy.get_param('~hz', 10))
        while not rospy.is_shutdown():
            [names, costs] = self.evaluator.GetState()
            self.state_pub.publish(names, costs)
            rate.sleep()

    def handler(self, message):
        print "Recieved message"
        self.evaluator.AddStateToTrajectory(message.data)
        self.evaluator.UpdateTrajectoryDTW()
        print "Done analysis"

if __name__ == '__main__':
    node_name = "test_arm"
    library_path = "/home/calderpg/Dropbox/DARPA_Research_Work/drc/arm_plan_pkg/trajectory_library_pr2_selected"
    DTWManager(node_name, library_path)
