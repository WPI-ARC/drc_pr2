#!/usr/bin/python

#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team

import roslib; roslib.load_manifest('task_control')
import rospy
import math
import random
import time
import os
import sys
import xml
from xml.etree.ElementTree import *

from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

class TrajectoryState:

    def __init__(self, desired_state, actual_state, state_error, time_secs, time_nsecs, typename):
        self.desired = desired_state
        self.actual = actual_state
        self.error = state_error
        self.trajectory_type = typename
        self.secs = time_secs
        self.nsecs = time_nsecs

    def __str__(self):
        string_representation = "desired: " + str(self.desired) + "\n"
        string_representation += "actual: " + str(self.actual) + "\n"
        string_representation += "error: " + str(self.error) + "\n"
        string_representation += "secs: " + str(self.secs) + "\n"
        string_representation += "nsecs: " + str(self.nsecs) + "\n"
        return string_representation

class TwoArmTrajectoryState:

    def __init__(self, left_state, right_state, typename):
        self.left = left_state
        self.right = right_state
        self.trajectory_type = typename

    def __str__(self):
        string_representation = "<state>\n<left>\n" + str(self.left) + "</left>\n"
        string_representation += "<right>\n" + str(self.right) + "</right>\n</state>\n"
        return string_representation

class TwoArmTrajectory:

    def __init__(self, filename=None, typename=None, traj_code=None):
        if (filename != None):
            self.states = []
            self.__load__(filename)
        else:
            self.trajectory_type = typename
            self.states = []
            self.code = traj_code

    def __load__(self, filepath):
        traj_file = open(filepath, "r")
        traj_tree = ElementTree().parse(traj_file)
        #load the code
        self.trajectory_type = traj_tree[0][1].text.strip("\n")
        self.code = traj_tree[0][2].text.strip("\n")
        for entry in traj_tree[1]:
            #load each state at a time
            left_state_chunk = entry[0].text.strip("\n")
            right_state_chunk = entry[1].text.strip("\n")
            #convert into object representation
            left_arm_state = self.__load_state_from_text__(left_state_chunk, self.trajectory_type)
            right_arm_state = self.__load_state_from_text__(right_state_chunk, self.trajectory_type)
            #add to the trajectory
            new_traj_state = TwoArmTrajectoryState(left_arm_state, right_arm_state, self.trajectory_type)
            self.states.append(new_traj_state)
        traj_file.close()

    def __str2list__(self, raw):
        try:
            raw = raw.lstrip("(").rstrip(")")
            raw = raw.lstrip("[").rstrip("]")
            parts = raw.split(", ")
            cleaned = []
            for part in parts:
                cleaned.append(float(part))
            return cleaned
        except:
            return None

    def __load_state_from_text__(self, statetext, statetype):
        text_line = statetext.split("\n")
        desired_state = self.__str2list__(text_line[0].split(": ")[1])
        actual_state = self.__str2list__(text_line[1].split(": ")[1])
        error_state = self.__str2list__(text_line[2].split(": ")[1])
        secs = (text_line[3].split(": "))[1]
        nsecs = (text_line[4].split(": "))[1]
        new_state = TrajectoryState(desired_state, actual_state, error_state, secs, nsecs, statetype)
        return new_state
    
    def BuildFromRawRos(self, left_arm_states, right_arm_states, trajtype, trajcode):
        temp_left = []
        temp_right = []
        #Convert left arm states
        for raw_state in left_arm_states:
            cleaned_state = self.__raw2cleaned__(raw_state, trajtype)
            temp_left.append(cleaned_state)
        #Convert right arm states
        for raw_state in right_arm_states:
            cleaned_state = self.__raw2cleaned__(raw_state, trajtype)
            temp_right.append(cleaned_state)
        #Set trajectory info
        self.trajectory_type = trajtype
        self.code = trajcode
        #Calc trajectory length
        length = min(len(temp_left), len(temp_right))
        #Set states
        for i in range(length):
            self.states.append(TwoArmTrajectoryState(temp_left[i], temp_right[i], trajtype))

    def __raw2cleaned__(self, raw_state, trajtype):
        if (trajtype == "JOINT"):
            return self.__raw_joint2cleaned__(raw_state)
        elif (trajtype == "POSE"):
            return self.__raw_pose2cleaned__(raw_state)
        else:
            raise AttributeError

    def __extract_posestamped__(self, raw_posestamped):
        real_pose = raw_posestamped.pose
        return [real_pose.position.x, real_pose.position.y, real_pose.position.z, real_pose.orientation.x, real_pose.orientation.y, real_pose.orientation.z, real_pose.orientation.w]

    def __raw_pose2cleaned__(self, raw_state):
        raw_secs = raw_state.header.stamp.secs
        raw_nsecs = raw_state.header.stamp.nsecs
        raw_desired = None
        raw_actual = self.__extract_posestamped__(raw_state)
        raw_error = None
        new_state = TrajectoryState(raw_desired, raw_actual, raw_error, raw_secs, raw_nsecs, "JOINT")
        return new_state

    def __raw_joint2cleaned__(self, raw_state):
        raw_secs = raw_state.header.stamp.secs
        raw_nsecs = raw_state.header.stamp.nsecs
        raw_desired = raw_state.desired.positions
        raw_actual = raw_state.actual.positions
        raw_error = raw_state.error.positions
        new_state = TrajectoryState(raw_desired, raw_actual, raw_error, raw_secs, raw_nsecs, "JOINT")
        return new_state

    def Write(self, filepath):
        XML_string = "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n<trajectory>\n<info>\n"
        XML_string += "<length>\n" + str(len(self.states)) + "\n</length>\n<type>\n" + self.trajectory_type + "\n</type>\n<code>\n" + self.code + "\n</code>\n</info>\n<states>\n"
        for state in self.states:
            XML_string += str(state)
        XML_string = XML_string + "</states>\n</trajectory>"
        xml_file = open(filepath, "w")
        xml_file.write(XML_string)
        xml_file.close()

    def Downsample(self, new_length=None, fraction=None):
        #Downsample the trajectory to the length provided OR the fraction of the length
        downsampled = TwoArmTrajectory(None, self.trajectory_type, self.code)
        if (new_length != None):
            current_length = len(self.states)
            if (new_length <= current_length):
                interval = float(current_length) / float(new_length)
                for i in range(new_length):
                    index = int(interval * i)
                    downsampled.states.append(self.states[index])
                return downsampled
            else:
                return self
        elif (fraction != None):
            current_length = len(self.states)
            assert(fraction <= 1.0)
            new_length = float(current_length) * fraction
            interval = float(current_length) / float(new_length)
            for i in range(new_length):
                index = int(interval * i)
                downsampled.states.append(self.states[index])
            return downsampled
        else:
            raise AttributeError

    def __array_to_csv__(self, array_to_convert):
        raw_string = str(array_to_convert)
        raw_string = raw_string.strip("[").strip("]")
        raw_string = raw_string.strip("(").strip(")")
        return raw_string

    def ConvertToCSV(self, target_filepath):
        #Converts the xml-formatted trajectory to csv for export
        csv_string = ""
        for state in self.states:
            csv_string += "left,desired," + self.__array_to_csv__(state.left.desired) + "\n"
            csv_string += "left,actual," + self.__array_to_csv__(state.left.actual) + "\n"
            csv_string += "left,error," + self.__array_to_csv__(state.left.error) + "\n"
            csv_string += "right,desired," + self.__array_to_csv__(state.right.desired) + "\n"
            csv_string += "right,actual," + self.__array_to_csv__(state.right.actual) + "\n"
            csv_string += "right,error," + self.__array_to_csv__(state.right.error) + "\n"
        csv_file.rstrip("\n")
        csv_file = open(target_filepath, "w")
        csv_file.write(csv_string)
        csv_file.close()
        return target_filepath
