# Bener Suay 2012 December
# benersuay@wpi.edu
#
# This script reads a native openrave trajectory xml file and executes it with the PR2
#
# To execute it with the PR2 in Gazebo:
#
# 1. keep movetraj0,1,2,3.txt and this script in the same folder
# 2. roslaunch pr2_gazebo pr2_empty_world.launch
# 3. python <this-script>.py
#
# To execute it with the real robot:
#
# 1. export $ROS_MASTER_URI=http://<your-robot's-ip>:11311
# 2. export $ROS_HOSTNAME=<your-computer's-ip-on-the-network>
# 3. python <this-script>.py
# 
# All calls to print are there for debugging purposes, you can remove them if they're bothering you.

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

from copy import deepcopy
import sys
 
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

dom_array = [parseString(data0), parseString(data1), parseString(data2), parseString(data3)]

rave2ros_pr2_l_arm_joint_dict = {'15': 'l_shoulder_pan_joint', '16': 'l_shoulder_lift_joint', '17': 'l_upper_arm_roll_joint', '18': 'l_elbow_flex_joint', '19': 'l_forearm_roll_joint', '20': 'l_wrist_flex_joint', '21': 'l_wrist_roll_joint'}

rave2ros_pr2_r_arm_joint_dict = {'27': 'r_shoulder_pan_joint', '28': 'r_shoulder_lift_joint', '29': 'r_upper_arm_roll_joint', '30': 'r_elbow_flex_joint', '31': 'r_forearm_roll_joint', '32': 'r_wrist_flex_joint', '33': 'r_wrist_roll_joint'}

rospy.init_node('my_trajectory_manager',anonymous=False)

for d in range(len(dom_array)):

    l_arm_joint_offsets = {}
    r_arm_joint_offsets = {}

    useLeftArm = True
    useRightArm = True

    l_arm_active_joint_names = []
    r_arm_active_joint_names = []

    d_length = len(dom_array[d].getElementsByTagName('data')[0].firstChild.data.split(' '))
    # last element is always empty because of openrave syntax
    #d_length = d_length -1

    data_count = int(dom_array[d].getElementsByTagName('data')[0].getAttribute('count'))
    data = dom_array[d].getElementsByTagName('data')[0].firstChild.data.split(' ')

    for t in range(len(dom_array[d].getElementsByTagName('group'))):

        # Get the value of the name attribute of this group tag
        name = dom_array[d].getElementsByTagName('group')[t].getAttribute('name')

        # It may be separated by space, make it a list
        name = name.split(' ')

        if( name[0] == 'deltatime'):
            deltatime_offset = int(dom_array[d].getElementsByTagName('group')[t].getAttribute('offset'))
            deltatime_dof = int(dom_array[d].getElementsByTagName('group')[t].getAttribute('dof'))

        elif( name[0] == 'joint_velocities'):
            robot_name = name[1]
            # Check robot name, sometimes we create artificial robots like crank, valve, or driving wheel. 
            # In this script though we're only interested in the PR2 joint velocities.
            if(robot_name == 'pr2'):
                vel_offset = int(dom_array[d].getElementsByTagName('group')[t].getAttribute('offset'))
                vel_dof = int(dom_array[d].getElementsByTagName('group')[t].getAttribute('dof'))

        elif( name[0] == 'joint_values'):
            robot_name = name[1]
            # Are these joint values defined for the pr2?
            # In this script we're only interested in the pr2 joint values
            if(robot_name == 'pr2'):
                jval_offset = int(dom_array[d].getElementsByTagName('group')[t].getAttribute('offset'))
                jval_dof = int(dom_array[d].getElementsByTagName('group')[t].getAttribute('dof'))

                # The following block populates the two unpopulated dictionnaries we created earlier
                # E.g. {'0': '15', ..., '9': '29', ...}
                # We dynamically create / populate this because we only want to get the value of a joint
                # if and only if it's: a) one of the joints we're interested and b) it's in the data
                for j in range(2,len(name)):
                    if(name[j] in rave2ros_pr2_l_arm_joint_dict.keys()):
                        for c in range(data_count):
                            ind = (c*(deltatime_offset+1))+(j-2)
                            l_arm_joint_offsets[str(ind)] = name[j]

                        l_arm_active_joint_names.append(rave2ros_pr2_l_arm_joint_dict[name[j]])
                        useLeftArm = True
                    elif(name[j] in rave2ros_pr2_r_arm_joint_dict.keys()):
                        for c in range(data_count):
                            ind = (c*(deltatime_offset+1))+(j-2)
                            r_arm_joint_offsets[str(ind)] = name[j]

                        r_arm_active_joint_names.append(rave2ros_pr2_r_arm_joint_dict[name[j]])
                        useRightArm = True


    # We're done with indexing, now let's harvest the data
    if(useLeftArm):
        l_traj = JointTrajectory()
        l_traj.joint_names = l_arm_active_joint_names
    if(useRightArm):
        r_traj = JointTrajectory()
        r_traj.joint_names = r_arm_active_joint_names

    # One of the following variables may not exist. Let's try and return error if we fail.
    try:
        # Deltaoffset is always the last data point, which gives us the length of one single chunk of data.
        # We know that there are datacount number of chunks.
        for c in range(data_count):

            if(useLeftArm):
                l_current_point = JointTrajectoryPoint()
                l_current_point.time_from_start = rospy.Duration(float(data[c+deltatime_offset]))
                
            if(useRightArm):
                r_current_point = JointTrajectoryPoint()
                r_current_point.time_from_start = rospy.Duration(float(data[c+deltatime_offset]))
               
            # Get the indices
            i = c*(deltatime_offset+1)

            i_vel = i + vel_offset
            f_vel = i_vel + vel_dof

            i_jval = i + jval_offset
            f_jval = i_jval + jval_dof


            # Start reading data based on indices
            # Read joint positions 
            l_p_buffer = []
            r_p_buffer = []

            for p in range(i_jval,f_jval):
                if(str(p) in l_arm_joint_offsets.keys()):
                    l_p_buffer.append(float(data[p]))
                elif(str(p) in r_arm_joint_offsets.keys()):
                    r_p_buffer.append(float(data[p]))

            if(useLeftArm):
                l_current_point.positions = deepcopy(l_p_buffer)
            if(useRightArm):
                r_current_point.positions = deepcopy(r_p_buffer)

            l_v_buffer = []
            r_v_buffer = []
            for v in range(i_vel,f_vel):
                if(str(v) in l_arm_joint_offsets.keys()):
                    l_v_buffer.append(float(data[v])*2)
                elif(str(v) in r_arm_joint_offsets.keys()):
                    r_v_buffer.append(float(data[v])*2)

            if(useLeftArm):
                l_current_point.velocities = deepcopy(l_v_buffer)
                l_traj.points.append(l_current_point)
            if(useRightArm):
                r_current_point.velocities = deepcopy(r_v_buffer)
                r_traj.points.append(r_current_point)

    except NameError, e:
        print "Error - one of the necessary variables is not found:"
        print e


    #print l_traj 
    #print r_traj
    
    l_goal = JointTrajectoryGoal()
    l_goal.trajectory = l_traj
    l_arm_client = actionlib.SimpleActionClient("l_arm_controller/joint_trajectory_action", JointTrajectoryAction)
    l_arm_client.wait_for_server()
    l_arm_client.send_goal(l_goal)

    r_goal = JointTrajectoryGoal()
    r_goal.trajectory = r_traj
    r_arm_client = actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action", JointTrajectoryAction)
    r_arm_client.wait_for_server()
    r_arm_client.send_goal(r_goal)
    
    r_arm_client.wait_for_result()
    l_arm_client.wait_for_result()
    
    print "Press enter to continue..."
    sys.stdin.readline()
    
        
    
