#import easy to use xml parser called minidom:
from xml.dom.minidom import parseString

import roslib;
roslib.load_manifest('tf')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('trajectory_msgs')
import rospy
import math
import random
import time

from std_msgs.msg import String
from tf import *
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *

import numpy

 
#open the xml file for reading:
file0 = open('movetraj0.txt','r')
file1 = open('movetraj1.txt','r')
file2 = open('movetraj2.txt','r')
file3 = open('movetraj3.txt','r')

data0 = file0.read()
data1 = file1.read()
data2 = file2.read()
data3 = file3.read()

file0.close()
file1.close()
file2.close()
file3.close()

dom0 = parseString(data0)
dom1 = parseString(data1)
dom2 = parseString(data2)
dom3 = parseString(data3)

str0 = dom0.getElementsByTagName('data')[0].toxml().replace('<data count="3">\n','').replace(' </data>','').replace(' ',',')

str1 = dom1.getElementsByTagName('data')[0].toxml().replace('<data count="50">\n','').replace(' </data>','').replace(' ',',')

str2 = dom2.getElementsByTagName('data')[0].toxml().replace('<data count="44">\n','').replace(' </data>','').replace(' ',',')

str3 = dom3.getElementsByTagName('data')[0].toxml().replace('<data count="3">\n','').replace(' </data>','').replace(' ',',')

array0 = eval('[' + str0  + ']')
array1 = eval('[' + str1  + ']')
array2 = eval('[' + str2  + ']')
array3 = eval('[' + str3  + ']')

l_traj = JointTrajectory()
r_traj = JointTrajectory()


l_traj.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]

r_traj.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]

# Initialize the points array
l_traj.points = []
r_traj.points = []

#print array0



for j in range(0, 67, 33):
    
    l_current_point = JointTrajectoryPoint()
    r_current_point = JointTrajectoryPoint()

    l_current_point.positions = array0[j:j+7]
    r_current_point.positions = array0[j+8:j+15]

    l_current_point.velocities = array0[j+16:j+23]
    r_current_point.velocities = array0[j+24:j+31]

    l_current_point.time_from_start = rospy.Duration(array0[j+32])
    r_current_point.time_from_start = rospy.Duration(array0[j+32])
        
    l_traj.points.append(l_current_point)
    r_traj.points.append(r_current_point)

print l_traj    

rospy.init_node('my_trajectory_manager',anonymous=False)

l_goal = JointTrajectoryGoal()
l_goal.trajectory = l_traj
l_arm_client = actionlib.SimpleActionClient("l_arm_controller/joint_trajectory_action", JointTrajectoryAction)
l_arm_client.wait_for_server()
l_arm_client.send_goal(l_goal)

# print l_traj
# print r_traj

r_goal = JointTrajectoryGoal()
r_goal.trajectory = r_traj
r_arm_client = actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action", JointTrajectoryAction)
r_arm_client.wait_for_server()
r_arm_client.send_goal(r_goal)

r_arm_client.wait_for_result()
l_arm_client.wait_for_result()


    

    


