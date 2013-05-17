#!/usr/bin/python

import roslib; roslib.load_manifest('task_control')
import rospy
import math
from std_msgs.msg import String
import actionlib
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *

class HeadPointer:

    def __init__(self):
        rospy.init_node('head_pointer', anonymous=True)
        #Set up actionlib clients
        self.head_client = actionlib.SimpleActionClient('head_traj_controller/point_head_action', PointHeadAction)
        self.head_client.wait_for_server()
        #Setup the message components
        headgoal = PointHeadGoal()
        point = PointStamped()
        point.header.frame_id = "base_link"
        #Set position values
        point.point.x = 1.41;
        point.point.y = 0.0;
        point.point.z = 1.0;
        headgoal.target = point;
        headgoal.pointing_frame = "high_def_link";
        headgoal.pointing_axis.x = 1;
        headgoal.pointing_axis.y = 0;
        headgoal.pointing_axis.z = 0;
        self.head_client.send_goal(headgoal)
        self.head_client.wait_for_result()

if __name__ == '__main__':
    HeadPointer()
