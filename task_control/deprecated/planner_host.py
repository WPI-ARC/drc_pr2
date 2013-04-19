#!/usr/bin/python
import time
import random
import roslib; roslib.load_manifest('arm_plan_pkg')
import rospy
import math

from std_msgs.msg import String
from arm_plan_pkg.srv import *

class TestManager:
    def __init__(self, name, planner_type, timeout):
        rospy.init_node(name, anonymous=False)
        self.timeout = timeout
        #Allocate some empty status variables
        self.latest_feedback = None
        self.latest_status = None
        self.latest_result = None
        #Setup the service callback
        self.test_handler = rospy.Service("test_plan", test_plan, self.TestHandler)
        self.exec_handler = rospy.Service("exec_plan", exec_plan, self.ExecHandler)
        print "Init-ed a planner host with planner " + planner_type + " and timeout of:", timeout
        rate = rospy.Rate(rospy.get_param('~hz', 60))
        while not rospy.is_shutdown():
            rate.sleep()

    def TestHandler(self, call):
        print "TEST_PLAN service request with request:", call.request
        random_delay = random.uniform(0.0, 15.0)
        time.sleep(random_delay)
        return 1

    def ExecHandler(self, call):
        print "EXEC_PLAN service request with request:", call.request
        random_delay = random.uniform(0.0, 25.0)
        time.sleep(random_delay)
        return 1

if __name__ == '__main__':
    name = "arm_planner_host"
    planner_type = "CBiRRT2"
    timeout = -1.0
    TestManager(name, planner_type, timeout)
