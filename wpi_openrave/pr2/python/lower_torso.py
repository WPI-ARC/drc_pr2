#!/usr/bin/env python

## ROS ##
import roslib; 
roslib.load_manifest('std_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('std_srvs')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('trajectory_msgs')
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *

rospy.init_node('lower_torso', anonymous=False)

torsoClient = actionlib.SimpleActionClient("torso_controller/position_joint_action",SingleJointPositionAction)
torsoGoal = SingleJointPositionGoal()
torsoGoal.position = 0.0
torsoGoal.min_duration = rospy.Duration(2.0)
torsoGoal.max_velocity = 1.0
torsoClient.wait_for_server()
torsoClient.send_goal(torsoGoal)
torsoClient.wait_for_result()
