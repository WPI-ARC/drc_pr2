#!/usr/bin/env python
# Bener Suay 2013 January
# benersuay@wpi.edu

#import easy to use xml parser called minidom:
from xml.dom.minidom import parseString

import roslib;
roslib.load_manifest('wpi_arm_plan_pkg')

import rospy
from math import *
import random
import time

from std_msgs.msg import String
from tf import *
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from task_control.msg import *
from task_control.srv import *

import numpy

from copy import deepcopy
import sys

l_arm_active_joint_names = ['l_shoulder_pan_joint','l_shoulder_lift_joint','l_upper_arm_roll_joint','l_elbow_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']

r_arm_active_joint_names = ['r_shoulder_pan_joint','r_shoulder_lift_joint','r_upper_arm_roll_joint','r_elbow_flex_joint','r_forearm_roll_joint','r_wrist_flex_joint','r_wrist_roll_joint']

# Input arguments:
# fname: filename to read and play the trajectory from
# num_sample: how many points do we want to play from this trajectory. 
# Note that this starts from the last point, so if num_sample is 1, we will only play the last point.
# If num_sample is less than 1, then we play every point on the trajectory.
def getPointFromTrajFile(fname,num_sample=-1,playAtTime=1.0):
    debug = False # this makes the script print out some of what it reads from file
    #open the xml file for reading:
    #print "Reading - playing: " + fname
    trajFile = open(fname,'r')

    data = trajFile.read()

    trajFile.close()

    dom = parseString(data) # Get the document object model

    rave2ros_pr2_l_arm_joint_dict = {'15': 'l_shoulder_pan_joint', '16': 'l_shoulder_lift_joint', '17': 'l_upper_arm_roll_joint', '18': 'l_elbow_flex_joint', '19': 'l_forearm_roll_joint', '20': 'l_wrist_flex_joint', '21': 'l_wrist_roll_joint'}

    rave2ros_pr2_r_arm_joint_dict = {'27': 'r_shoulder_pan_joint', '28': 'r_shoulder_lift_joint', '29': 'r_upper_arm_roll_joint', '30': 'r_elbow_flex_joint', '31': 'r_forearm_roll_joint', '32': 'r_wrist_flex_joint', '33': 'r_wrist_roll_joint'}

    # Also note 
    # 12: torso_lift_joint
    # 23: l_gripper_l_finger_joint
    # 34: r_gripper_l_finger_joint

    # These dictionnaries will keep the information about which elements in the trajectory array correspond to left arm joints, right arm joints, torso and gripper joints
    l_arm_joint_offsets = {}
    r_arm_joint_offsets = {}
    l_gripper_joint_offsets = {}
    r_gripper_joint_offsets = {}
    torso_joint_offsets = {}

    useLeftArm = False
    useRightArm = False
    useLeftGripper = False
    useRightGripper = False
    useTorso = False

    l_arm_active_joint_names = ['l_shoulder_pan_joint','l_shoulder_lift_joint','l_upper_arm_roll_joint','l_elbow_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    r_arm_active_joint_names = ['r_shoulder_pan_joint','r_shoulder_lift_joint','r_upper_arm_roll_joint','r_elbow_flex_joint','r_forearm_roll_joint','r_wrist_flex_joint','r_wrist_roll_joint']
    

    d_length = len(dom.getElementsByTagName('data')[0].firstChild.data.split(' '))
    # last element is always empty because of openrave syntax
    #d_length = d_length -1

    data_count = int(dom.getElementsByTagName('data')[0].getAttribute('count'))
    data = dom.getElementsByTagName('data')[0].firstChild.data.split(' ')
    stepsize = int(len(data)/data_count) # In "data" array we have joint angles, velocities and dTime. These values are repeated data_count times.

    if(debug):
        print "stepsize"
        print stepsize

    for t in range(len(dom.getElementsByTagName('group'))):

        # Get the value of the name attribute of this group tag
        name = dom.getElementsByTagName('group')[t].getAttribute('name')

        # It may be separated by space, make it a list
        name = name.split(' ')

        if( name[0] == 'deltatime'):
            deltatime_offset = int(dom.getElementsByTagName('group')[t].getAttribute('offset'))
            deltatime_dof = int(dom.getElementsByTagName('group')[t].getAttribute('dof'))
            if(debug):
                print "deltatime_offset"
                print deltatime_offset
                print "deltatime_dof"
                print deltatime_dof

        elif( name[0] == 'joint_velocities'):
            robot_name = name[1]
            # Check robot name, sometimes we create artificial robots like crank, valve, or driving wheel. 
            # In this script though we're only interested in the PR2 joint velocities.
            if(robot_name == 'pr2'):
                vel_offset = int(dom.getElementsByTagName('group')[t].getAttribute('offset'))
                vel_dof = int(dom.getElementsByTagName('group')[t].getAttribute('dof'))
                if(debug):
                    print "vel_offset"
                    print vel_offset
                    print "vel_dof"
                    print vel_dof

        elif( name[0] == 'joint_values'):
            robot_name = name[1]
            # Are these joint values defined for the pr2?
            # In this script we're only interested in the pr2 joint values
            if(robot_name == 'pr2'):
                jval_offset = int(dom.getElementsByTagName('group')[t].getAttribute('offset'))
                jval_dof = int(dom.getElementsByTagName('group')[t].getAttribute('dof'))

                if(debug):
                    print "jval_offset"
                    print jval_offset
                    print "jval_dof"
                    print jval_dof

                # The following block populates the two unpopulated dictionnaries we created earlier
                # E.g. {'15': '15', '21': '21', '17': '17', '16': '16', '19': '19', '173': '15', '100': '21', '178': '20', '20': '20', '99': '20', '98': '19', '179': '21', '18': '18', '177': '19', '176': '18', '175': '17', '174': '16', '95': '16', '94': '15', '97': '18', '96': '17'}
                # In the example dictionnary, 15th, 173th, and 94th elements of the "data" array belong to the value of joint 15.
                # We dynamically create / populate this because we only want to get the value of a joint
                # if and only if it's: a) one of the joints we're interested and b) it's in the data
                for j in range(2,len(name)):
                    if(name[j] in rave2ros_pr2_l_arm_joint_dict.keys()):
                        for c in range(data_count):
                            # ind: Where, in that big array of data, will be the angle value of joint "name[j]"?
                            # Note: the reasone we're adding (j-2) is because jval_offset gives us the index of the first joint's value.
                            # index should be found as follows:
                            # stepsize=(len(data)/data_count) : how long is one block of data (vals, vels, dTime, altogether)?
                            # jval_offset : offset from the beginning of one block of data for the first joint's ang value
                            # j-2 : gives us the offset for the specific joint angle we're interested
                            # 
                            # In short, we are looking for:
                            #
                            # ind = (c*stepsize)+(jval_offset)+(j-2)
                            ind = (c*stepsize)+(jval_offset)+(j-2)                            
                            l_arm_joint_offsets[str(ind)] = name[j]
                        useLeftArm = True
                    elif(name[j] in rave2ros_pr2_r_arm_joint_dict.keys()):
                        for c in range(data_count):
                            ind = (c*stepsize)+(jval_offset)+(j-2)
                            r_arm_joint_offsets[str(ind)] = name[j]
                        useRightArm = True
                    elif(name[j] == 'torso_lift_join'):
                        for c in range(data_count):
                            ind = (c*stepsize)+(jval_offset)+(j-2)
                            torso_joint_offsets[str(ind)] = name[j]
                        useTorso = True
                    elif(name[j] == 'l_gripper_l_finger_joint'):
                        for c in range(data_count):
                            ind = (c*stepsize)+(jval_offset)+(j-2)
                            l_gripper_joint_offsets[str(ind)] = name[j]
                        useLeftGripper = True
                    elif(name[j] == 'r_gripper_l_finger_joint'):
                        for c in range(data_count):
                            ind = (c*stepsize)+(jval_offset)+(j-2)
                            r_gripper_joint_offsets[str(ind)] = name[j]
                        useRightGripper = True

    if(debug):
        print "l_arm_joint_offsets"
        print l_arm_joint_offsets
        print "r_arm_joint_offsets"
        print r_arm_joint_offsets

    # One of the following variables may not exist. Let's try and return error if we fail.
    try:
        # Deltaoffset is always the last data point, which gives us the length of one single chunk of data.
        # We know that there are datacount number of chunks.

        if((num_sample < 1) or (num_sample > int(data_count))): # play full trajectory
            start_from = 0
            end_at = data_count
            skip_this_many_points = 0
        elif(num_sample == 1): # play the last point
            start_from = data_count - 1
            end_at = data_count
            skip_this_many_points = 0
        elif(num_sample == 2): # play the first and the last points
            start_from = 0
            end_at = data_count
            skip_this_many_points = data_count - 1
        else:
            start_from = data_count - (num_sample*int(floor(int(data_count)/num_sample)))
            end_at = data_count
            skip_this_many_points = int(floor(int(data_count)/num_sample))
            
        increment_by = 1 + skip_this_many_points

        # print "num_sample"
        # print num_sample
        # print "start_from"
        # print start_from
        # print "end_at"
        # print end_at
        # print "skip_this_many_points"
        # print skip_this_many_points
        # print "increment_by"
        # print increment_by

        for c in range(start_from,end_at,increment_by):

            if(useLeftArm):
                l_current_point = JointTrajectoryPoint()
                #l_current_point.time_from_start = rospy.Duration(float(data[c*deltatime_offset]))
                l_current_point.time_from_start = rospy.Duration(playAtTime)
            if(useRightArm):
                r_current_point = JointTrajectoryPoint()
                #r_current_point.time_from_start = rospy.Duration(float(data[c*deltatime_offset]))
                r_current_point.time_from_start = rospy.Duration(playAtTime)

            # Get the indices for this subset of data
            i = c*(deltatime_offset+deltatime_dof)

            i_vel = i + vel_offset
            f_vel = i_vel + vel_dof

            i_jval = i + jval_offset
            f_jval = i_jval + jval_dof


            # Start reading data based on indices
            # Read joint positions 

            # Emptry position buffers for arm trajectories
            l_p_buffer = []
            r_p_buffer = []

            # Fill in position buffers
            for p in range(i_jval,f_jval):
                if(str(p) in l_arm_joint_offsets.keys()):
                    l_p_buffer.append(float(data[p]))
                elif(str(p) in r_arm_joint_offsets.keys()):
                    r_p_buffer.append(float(data[p]))

            # Empty velocity buffers for arm trajectories
            l_v_buffer = []
            r_v_buffer = []

            # Fill in velocity buffers
            for v in range(i_vel,f_vel):
                if(str(v) in l_arm_joint_offsets.keys()):
                    #l_v_buffer.append(float(data[v]))
                    l_v_buffer.append(0.0)
                elif(str(v) in r_arm_joint_offsets.keys()):
                    #r_v_buffer.append(float(data[v]))
                    r_v_buffer.append(0.0)

            # Then the arms
            if(useLeftArm):
                l_current_point.positions = deepcopy(l_p_buffer)
                l_current_point.velocities = deepcopy(l_v_buffer)
            if(useRightArm):
                r_current_point.positions = deepcopy(r_p_buffer)
                r_current_point.velocities = deepcopy(r_v_buffer)

        return [l_current_point, r_current_point]
                    
    except NameError, e:
        print "Error - one of the necessary variables is not found:"
        print e

    # print "Finished playing trajectory file: " + fname
    return 1

def getCompleteTraj(fname,num_sample=-1):
    debug = False # this makes the script print out some of what it reads from file
    #open the xml file for reading:
    # print "Reading - playing: " + fname
    trajFile = open(fname,'r')

    data = trajFile.read()

    trajFile.close()

    dom = parseString(data) # Get the document object model

    rave2ros_pr2_l_arm_joint_dict = {'15': 'l_shoulder_pan_joint', '16': 'l_shoulder_lift_joint', '17': 'l_upper_arm_roll_joint', '18': 'l_elbow_flex_joint', '19': 'l_forearm_roll_joint', '20': 'l_wrist_flex_joint', '21': 'l_wrist_roll_joint'}

    rave2ros_pr2_r_arm_joint_dict = {'27': 'r_shoulder_pan_joint', '28': 'r_shoulder_lift_joint', '29': 'r_upper_arm_roll_joint', '30': 'r_elbow_flex_joint', '31': 'r_forearm_roll_joint', '32': 'r_wrist_flex_joint', '33': 'r_wrist_roll_joint'}

    # Also note 
    # 12: torso_lift_joint
    # 23: l_gripper_l_finger_joint
    # 34: r_gripper_l_finger_joint

    # These dictionnaries will keep the information about which elements in the trajectory array correspond to left arm joints, right arm joints, torso and gripper joints
    l_arm_joint_offsets = {}
    r_arm_joint_offsets = {}
    l_gripper_joint_offsets = {}
    r_gripper_joint_offsets = {}
    torso_joint_offsets = {}

    useLeftArm = False
    useRightArm = False
    useLeftGripper = False
    useRightGripper = False
    useTorso = False

    l_arm_active_joint_names = ['l_shoulder_pan_joint','l_shoulder_lift_joint','l_upper_arm_roll_joint','l_elbow_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    r_arm_active_joint_names = ['r_shoulder_pan_joint','r_shoulder_lift_joint','r_upper_arm_roll_joint','r_elbow_flex_joint','r_forearm_roll_joint','r_wrist_flex_joint','r_wrist_roll_joint']
    

    d_length = len(dom.getElementsByTagName('data')[0].firstChild.data.split(' '))
    # last element is always empty because of openrave syntax
    #d_length = d_length -1

    data_count = int(dom.getElementsByTagName('data')[0].getAttribute('count'))
    data = dom.getElementsByTagName('data')[0].firstChild.data.split(' ')
    stepsize = int(len(data)/data_count) # In "data" array we have joint angles, velocities and dTime. These values are repeated data_count times.

    if(debug):
        print "stepsize"
        print stepsize

    for t in range(len(dom.getElementsByTagName('group'))):

        # Get the value of the name attribute of this group tag
        name = dom.getElementsByTagName('group')[t].getAttribute('name')

        # It may be separated by space, make it a list
        name = name.split(' ')

        if( name[0] == 'deltatime'):
            deltatime_offset = int(dom.getElementsByTagName('group')[t].getAttribute('offset'))
            deltatime_dof = int(dom.getElementsByTagName('group')[t].getAttribute('dof'))
            if(debug):
                print "deltatime_offset"
                print deltatime_offset
                print "deltatime_dof"
                print deltatime_dof

        elif( name[0] == 'joint_velocities'):
            robot_name = name[1]
            # Check robot name, sometimes we create artificial robots like crank, valve, or driving wheel. 
            # In this script though we're only interested in the PR2 joint velocities.
            if(robot_name == 'pr2'):
                vel_offset = int(dom.getElementsByTagName('group')[t].getAttribute('offset'))
                vel_dof = int(dom.getElementsByTagName('group')[t].getAttribute('dof'))
                if(debug):
                    print "vel_offset"
                    print vel_offset
                    print "vel_dof"
                    print vel_dof

        elif( name[0] == 'joint_values'):
            robot_name = name[1]
            # Are these joint values defined for the pr2?
            # In this script we're only interested in the pr2 joint values
            if(robot_name == 'pr2'):
                jval_offset = int(dom.getElementsByTagName('group')[t].getAttribute('offset'))
                jval_dof = int(dom.getElementsByTagName('group')[t].getAttribute('dof'))

                if(debug):
                    print "jval_offset"
                    print jval_offset
                    print "jval_dof"
                    print jval_dof

                # The following block populates the two unpopulated dictionnaries we created earlier
                # E.g. {'15': '15', '21': '21', '17': '17', '16': '16', '19': '19', '173': '15', '100': '21', '178': '20', '20': '20', '99': '20', '98': '19', '179': '21', '18': '18', '177': '19', '176': '18', '175': '17', '174': '16', '95': '16', '94': '15', '97': '18', '96': '17'}
                # In the example dictionnary, 15th, 173th, and 94th elements of the "data" array belong to the value of joint 15.
                # We dynamically create / populate this because we only want to get the value of a joint
                # if and only if it's: a) one of the joints we're interested and b) it's in the data

                if(debug):
                    for c in range(2):
                        print data[(c*stepsize)+jval_offset:(c*stepsize)+jval_offset+jval_dof]

                for j in range(2,len(name)):
                    if(name[j] in rave2ros_pr2_l_arm_joint_dict.keys()):
                        for c in range(data_count):
                            # ind: Where, in that big array of data, will be the angle value of joint " [j]"?
                            # Note: the reasone we're adding (j-2) is because jval_offset gives us the index of the first joint's value.
                            # index should be found as follows:
                            # stepsize=(len(data)/data_count) : how long is one block of data (vals, vels, dTime, altogether)?
                            # jval_offset : offset from the beginning of one block of data for the first joint's ang value
                            # j-2 : gives us the offset for the specific joint angle we're interested
                            # 
                            # In short, we are looking for:
                            #
                            # ind = (c*stepsize)+(jval_offset)+(j-2)
                            ind = (c*stepsize)+(jval_offset)+(j-2)                            
                            l_arm_joint_offsets[str(ind)] = name[j]
                        useLeftArm = True
                    elif(name[j] in rave2ros_pr2_r_arm_joint_dict.keys()):
                        for c in range(data_count):
                            ind = (c*stepsize)+(jval_offset)+(j-2)
                            r_arm_joint_offsets[str(ind)] = name[j]
                        useRightArm = True
                    elif(name[j] == 'torso_lift_join'):
                        for c in range(data_count):
                            ind = (c*stepsize)+(jval_offset)+(j-2)
                            torso_joint_offsets[str(ind)] = name[j]
                        useTorso = True
                    elif(name[j] == 'l_gripper_l_finger_joint'):
                        for c in range(data_count):
                            ind = (c*stepsize)+(jval_offset)+(j-2)
                            l_gripper_joint_offsets[str(ind)] = name[j]
                        useLeftGripper = True
                    elif(name[j] == 'r_gripper_l_finger_joint'):
                        for c in range(data_count):
                            ind = (c*stepsize)+(jval_offset)+(j-2)
                            r_gripper_joint_offsets[str(ind)] = name[j]
                        useRightGripper = True

    if(debug):
        print "l_arm_joint_offsets"
        print l_arm_joint_offsets
        print "r_arm_joint_offsets"
        print r_arm_joint_offsets

    # One of the following variables may not exist. Let's try and return error if we fail.
    try:
        # Deltaoffset is always the last data point, which gives us the length of one single chunk of data.
        # We know that there are datacount number of chunks.

        if((num_sample < 1) or (num_sample > int(data_count))): # play full trajectory
            start_from = 0
            end_at = data_count
            skip_this_many_points = 0
        elif(num_sample == 1): # play the last point
            start_from = data_count - 1
            end_at = data_count
            skip_this_many_points = 0
        elif(num_sample == 2): # play the first and the last points
            start_from = 0
            end_at = data_count
            skip_this_many_points = data_count - 1
        else:
            start_from = data_count - (num_sample*int(floor(int(data_count)/num_sample)))
            end_at = data_count
            skip_this_many_points = int(floor(int(data_count)/num_sample))
            
        increment_by = 1 + skip_this_many_points

        # print "num_sample"
        # print num_sample
        # print "start_from"
        # print start_from
        # print "end_at"
        # print end_at
        # print "skip_this_many_points"
        # print skip_this_many_points
        # print "increment_by"
        # print increment_by

        if(useLeftArm):
            l_traj = JointTrajectory()
            l_traj.joint_names = l_arm_active_joint_names
        if(useRightArm):
            r_traj = JointTrajectory()
            r_traj.joint_names = r_arm_active_joint_names
        
        tc = 1 # time_counter
        deltaT = float(6.0/data_count)
        for c in range(start_from,end_at,increment_by):
            #if(debug):
            # print "time_from_start"
            # print tc*deltaT
            if(useLeftArm):
                l_current_point = JointTrajectoryPoint()
                #l_current_point.time_from_start = rospy.Duration(float(data[c*deltatime_offset]))
                l_current_point.time_from_start = rospy.Duration(2.0+(tc*deltaT))
            if(useRightArm):
                r_current_point = JointTrajectoryPoint()
                #r_current_point.time_from_start = rospy.Duration(float(data[c*deltatime_offset]))
                r_current_point.time_from_start = rospy.Duration(2.0+(tc*deltaT))
            if(useTorso):
                torso_goal = SingleJointPositionGoal()
                torso_goal.min_duration = rospy.Duration(float(data[c*deltatime_offset]))
            if(useLeftGripper):
                l_gripper_goal = Pr2GripperCommandGoal()
                l_gripper_goal.command.max_effort = -1.0
            if(useRightGripper):
                r_gripper_goal = Pr2GripperCommandGoal()
                r_gripper_goal.command.max_effort = -1.0


            tc = tc + 1
            # Get the indices for this subset of data
            #i = c*(deltatime_offset+deltatime_dof)
            i = c*stepsize

            i_vel = i + vel_offset
            f_vel = i_vel + vel_dof

            i_jval = i + jval_offset
            f_jval = i_jval + jval_dof


            # Start reading data based on indices
            # Read joint positions 

            # Emptry position buffers for arm trajectories
            l_p_buffer = []
            r_p_buffer = []

            # Fill in position buffers
            for p in range(i_jval,f_jval):
                if(str(p) in l_arm_joint_offsets.keys()):
                    l_p_buffer.append(float(data[p]))
                elif(str(p) in r_arm_joint_offsets.keys()):
                    r_p_buffer.append(float(data[p]))
                elif(str(p) in torso_joint_offsets.keys()):
                    torso_goal.position = float(data[p])
                elif(str(p) in l_gripper_joint_offsets.keys()):
                    l_gipper_goal.command.position = float(data[p])
                elif(str(p) in r_gripper_joint_offsets.keys()):
                    r_gipper_goal.command.position = float(data[p])

            if(debug):
                print l_p_buffer
                print r_p_buffer
            # Empty velocity buffers for arm trajectories
            l_v_buffer = []
            r_v_buffer = []

            # Fill in velocity buffers
            for v in range(i_vel,f_vel):
                if(str(v) in l_arm_joint_offsets.keys()):
                    #l_v_buffer.append(float(data[v]))
                    l_v_buffer.append(0.0)
                elif(str(v) in r_arm_joint_offsets.keys()):
                    #r_v_buffer.append(float(data[v]))
                    r_v_buffer.append(0.0)
                elif(str(v) in torso_joint_offsets.keys()):
                    torso_goal.max_velocity = float(data[v])
                # Note that there is no velocity attribute for gripper goals

            # Then the arms
            if(useLeftArm):
                l_current_point.positions = deepcopy(l_p_buffer)
                l_current_point.velocities = deepcopy(l_v_buffer)
                l_traj.points.append(l_current_point)
                l_goal = JointTrajectoryGoal()
                l_goal.trajectory = l_traj
            if(useRightArm):
                r_current_point.positions = deepcopy(r_p_buffer)
                r_current_point.velocities = deepcopy(r_v_buffer)
                r_traj.points.append(r_current_point)
                r_goal = JointTrajectoryGoal()
                r_goal.trajectory = r_traj

        return [l_traj, r_traj]
                    
    except NameError, e:
        print "Error - one of the necessary variables is not found:"
        print e

    # print "Finished playing trajectory file: " + fname
    return 1

def compilePrepTrajectory(arm):
    t = JointTrajectory()

    #if(arm == 'leftArm'):
    #    t.joint_names = l_arm_active_joint_names
    #    i = 0
    
    #if(arm == 'rightArm'):
    #    t.joint_names = r_arm_active_joint_names
    #    i = 1

    #init = getPointFromTrajFile('movetraj0.txt',1,1.0)[i]
    #approach1 = getPointFromTrajFile('movetraj1.txt',1,2.0)[i]
    #approach2 = getPointFromTrajFile('movetraj2.txt',1,3.0)[i]
    #t.points.append(init)
    #t.points.append(approach1)
    #t.points.append(approach2)
    
    if(arm == 'leftArm'):
        return getCompleteTraj('movetraj2.txt')[0]
    
    if(arm == 'rightArm'):
        return getCompleteTraj('movetraj2.txt')[1]

    return t

def compileMainTrajectory(arm):
    if(arm == 'leftArm'):
        return getCompleteTraj('movetraj3.txt')[0]
    
    if(arm == 'rightArm'):
        return getCompleteTraj('movetraj3.txt')[1]

def compileReverseTrajectory(arm):
    if(arm == 'leftArm'):
        tr3 = reverseTrajectory('movetraj3.txt',arm)
        return tr3

    if(arm == 'rightArm'):
        tr3 = reverseTrajectory('movetraj3.txt',arm)
        return tr3

def compilePostTrajectory(arm):
    if(arm == 'leftArm'):
        tr5 = getCompleteTraj('movetraj5.txt')[0]
        return tr5
    if(arm == 'rightArm'):
        tr5 = getCompleteTraj('movetraj5.txt')[1]
        return tr5

def concatenateTrajectories(t1,t2):
    last_t = t1.points[len(t1.points)-1].time_from_start.to_sec()
    for p in range(len(t2.points)):
            t2.points[p].time_from_start = rospy.Duration(t2.points[p].time_from_start.to_sec()+6.0)
            t1.points.append(t2.points[p])
    return t1

def reverseTrajectory(tname,arm):
    if(arm == 'leftArm'):
        t_fwd = JointTrajectory()
        t_bwd = JointTrajectory()
        t_fwd = getCompleteTraj(tname)[0]
        for p in range(len(t_fwd.points)-1,-1,-1):
            t_bwd.points.append(t_fwd.points[p])

        for p in range(len(t_fwd.points)):
            t_bwd.points[p].time_from_start = rospy.Duration(float(p*(6.0/len(t_fwd.points))))
            

        t_bwd.joint_names = t_fwd.joint_names
        return t_bwd

    if(arm == 'rightArm'):
        t_fwd = JointTrajectory()
        t_bwd = JointTrajectory()
        t_fwd = getCompleteTraj(tname)[1]
        for p in range(len(t_fwd.points)-1,-1,-1):
            t_bwd.points.append(t_fwd.points[p])

        for p in range(len(t_fwd.points)):
            t_bwd.points[p].time_from_start = rospy.Duration(float(p*(6.0/len(t_fwd.points))))

        t_bwd.joint_names = t_fwd.joint_names
        return t_bwd

def pr2_rave2task_control(traj_code="test"):

    rospy.wait_for_service("task_control/Trajectory_Request")
    traj_client = rospy.ServiceProxy("task_control/Trajectory_Request", PR2ManagedJointTrajectory)

    close_gripper_goal = Pr2GripperCommandGoal()
    close_gripper_goal.command.position = 0.00
    close_gripper_goal.command.max_effort = -1.0
    open_gripper_goal = Pr2GripperCommandGoal()
    open_gripper_goal.command.position = 0.09
    open_gripper_goal.command.max_effort = -1.0

    trajectory_to_execute = PR2ManagedJointTrajectory._request_class()    
    trajectory_to_execute.left_arm_command.prep_trajectory = compilePrepTrajectory('leftArm')
    trajectory_to_execute.left_arm_command.main_trajectory = compileMainTrajectory('leftArm')
    trajectory_to_execute.left_arm_command.return_trajectory = compileReverseTrajectory('leftArm')
    trajectory_to_execute.left_arm_command.post_trajectory = compilePostTrajectory('leftArm')
    trajectory_to_execute.right_arm_command.prep_trajectory = compilePrepTrajectory('rightArm')
    trajectory_to_execute.right_arm_command.main_trajectory = compileMainTrajectory('rightArm')
    trajectory_to_execute.right_arm_command.return_trajectory = compileReverseTrajectory('rightArm')
    trajectory_to_execute.right_arm_command.post_trajectory = compilePostTrajectory('rightArm')

    #Set gripper commands
    trajectory_to_execute.left_gripper_commands = [open_gripper_goal, close_gripper_goal, open_gripper_goal, open_gripper_goal, open_gripper_goal]
    trajectory_to_execute.right_gripper_commands = [open_gripper_goal, close_gripper_goal, open_gripper_goal, open_gripper_goal, open_gripper_goal]

    #Set header info
    trajectory_to_execute.code = traj_code
    trajectory_to_execute.filepath = "test"

    #Send the trajectory to the trajectory controller
    print "sent trajectories to the client. waiting for a result..."
    result = traj_client(trajectory_to_execute)
    print "got a result:"
    print result
    return result

def main():
    # Initialize ROS node
    rospy.init_node('pr2_rave2task_control',anonymous=False)        
    pr2_rave2task_control()
    return 1

if __name__ == "__main__":
    main()
