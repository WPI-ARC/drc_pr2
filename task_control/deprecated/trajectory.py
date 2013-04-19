import time
import roslib; roslib.load_manifest('arm_plan_pkg')
import rospy
import math
import xml
from xml.etree.ElementTree import *

from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *

class Trajectory:

    def __init__(self):
        self.states = []
        self.ideal = []
        self.result = []
        self.feedback = []
        self.status = []
        self.LIVE = True

    def Start(self):
        if not self.LIVE:
            raise AttributeError
        self.start_time = time.clock()

    def Stop(self, code):
        self.stop_time = time.clock()
        self.duration = self.stop_time - self.start_time
        self.length = len(self.states)
        self.code = code
        self.LIVE = False

    def cleanup(self, state):
        state_string = "<state>\n<info>\n"
        state_string = state_string + "secs: " + str(state.header.stamp.secs) + "\n"
        state_string = state_string + "nsecs: " + str(state.header.stamp.nsecs) + "\n"
        state_string = state_string + "</info>\n<desired>\n"
        state_string = state_string + "positions: " + str(state.desired.positions) + "\n"
        state_string = state_string + "velocities: " + str(state.desired.velocities) + "\n"
        state_string = state_string + "accelerations: " + str(state.desired.velocities) + "\n"
        state_string = state_string + "</desired>\n<actual>\n"
        state_string = state_string + "positions: " + str(state.actual.positions) + "\n"
        state_string = state_string + "velocities: " + str(state.actual.velocities) + "\n"
        state_string = state_string + "accelerations: " + str(state.actual.velocities) + "\n"
        state_string = state_string + "</actual>\n<error>\n"
        state_string = state_string + "positions: " + str(state.error.positions) + "\n"
        state_string = state_string + "velocities: " + str(state.error.velocities) + "\n"
        state_string = state_string + "accelerations: " + str(state.error.velocities) + "\n"
        state_string = state_string + "</error>\n</state>\n"
        return state_string

    def output(self, point):
        state_string = "<state>\n<info>\n"
        state_string = state_string + "secs: " + str(point.time_from_start.secs) + "\n"
        state_string = state_string + "nsecs: " + str(point.time_from_start.nsecs) + "\n"
        state_string = state_string + "</info>\n<command>\n"
        state_string = state_string + "positions: " + str(point.positions) + "\n"
        state_string = state_string + "velocities: " + str(point.velocities) + "\n"
        state_string = state_string + "accelerations: " + str(point.velocities) + "\n"
        state_string = state_string + "</command>\n</state>\n"
        return state_string

    def ParseOut(self, filename):
        XML_string = "<trajectory>\n<info>\n"
        XML_string = XML_string + "<joints>\n" + str(self.ideal.joint_names) + "\n</joints>\n<time>\n" + str(self.duration) + "\n</time>\n<length>\n" + str(self.length) + "\n</length>\n<code>\n" + self.code + "\n</code>\n</info>\n"
        XML_string = XML_string + "<commanded>\n"
        for point in self.ideal.points:
            XML_string = XML_string + self.output(point)
        XML_string = XML_string + "</commanded>\n<executed>\n"
        for state in self.states:
            XML_string = XML_string + self.cleanup(state)
        XML_string = XML_string + "</executed>\n</trajectory>"
        xml_file = open(filename, "w")
        xml_file.write(XML_string)
        xml_file.close()
