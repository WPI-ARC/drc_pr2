#!/usr/bin/python

#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team
#
#   Manages execution of a given trajectory using a PR2
#   Each trajectory is added to the library of trajectories
#   for use with Dynamic Time Warping to classify and identify
#   new trajectories as encountered in execution.
#

import roslib; roslib.load_manifest('arm_plan_pkg')
import rospy
import math
import random
import time
from trajectory import *
from dtw import *

from std_msgs.msg import String
from tf import *
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from arm_plan_pkg.srv import *

class TestManager:
    def __init__(self, iterations):
        rospy.init_node('wheel_test_manager', anonymous=False)
        #Setup the service clients
        rospy.wait_for_service("arm_plan_pkg/Trajectory_Request")
        self.traj_client = rospy.ServiceProxy("arm_plan_pkg/Trajectory_Request", TrajectoryRequest)
        #self.valv_client = rospy.ServiceClient
        print "Service host up"
        #Buld library
        print "--- Building example library ---"
        library = self.BuildLibrary(iterations)
        print "*** Example library built ***"
        
    def PrepTrajectory(self):
        trajrequest = JointTrajectory()
        trajrequest.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        start_point = JointTrajectoryPoint()
        start_point.positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        start_point.time_from_start = rospy.Duration(1.0)
        end_point = JointTrajectoryPoint()
        end_point.positions = [0.36940511785382696, 0.12610419934920733, 0.8965900769167513, -1.239355266992639, 2.5358967504421166, -1.1679996900193927, -0.8058902197034163]
        end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        end_point.time_from_start = rospy.Duration(2.0)
        trajrequest.points = [start_point, end_point]
        return trajrequest

    def GenForwardTrajectory(self):
        trajrequest = JointTrajectory()
        trajrequest.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        point_1 = JointTrajectoryPoint()
        point_1.positions = [0.36940511785382696, 0.12610419934920733, 0.8965900769167513, -1.239355266992639, 2.5358967504421166, -1.1679996900193927, -0.8058902197034163]
        point_1.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point_1.time_from_start = rospy.Duration(0.5)
        point_2 = JointTrajectoryPoint()
        point_2.positions = [0.22763480217338128, -0.12471974765550076, 0.8981936251804501, -0.8778628006679704, 2.511948056444584, -1.1341062412618088, 0.17484434826154094]
        point_2.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point_2.time_from_start = rospy.Duration(1.0)
        point_3 = JointTrajectoryPoint()
        point_3.positions = [-0.04562540863524378, -0.27715979908972804, 0.8996368186177791, -0.46309471255416346, 2.599065769392128, -1.098211382179644, 1.1238174045537943]
        point_3.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point_3.time_from_start = rospy.Duration(1.5)
        point_4 = JointTrajectoryPoint()
        point_4.positions = [-0.030950937363057318, -0.2729300529345054, 0.8621137892472258, -0.47091233096887397, 2.593454601957924, -1.1111770403693226, 1.1182482627810466]
        point_4.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point_4.time_from_start = rospy.Duration(2.0)
        trajrequest.points = [point_1, point_2, point_3, point_4]
        return trajrequest

    def GenBackwardTrajectory(self):
        trajrequest = JointTrajectory()
        trajrequest.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        point_4 = JointTrajectoryPoint()
        point_4.positions = [0.36940511785382696, 0.12610419934920733, 0.8965900769167513, -1.239355266992639, 2.5358967504421166, -1.1679996900193927, -0.8058902197034163]
        point_4.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point_4.time_from_start = rospy.Duration(0.5)
        point_3 = JointTrajectoryPoint()
        point_3.positions = [0.22763480217338128, -0.12471974765550076, 0.8981936251804501, -0.8778628006679704, 2.511948056444584, -1.1341062412618088, 0.17484434826154094]
        point_3.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point_3.time_from_start = rospy.Duration(1.0)
        point_2 = JointTrajectoryPoint()
        point_2.positions = [-0.04562540863524378, -0.27715979908972804, 0.8996368186177791, -0.46309471255416346, 2.599065769392128, -1.098211382179644, 1.1238174045537943]
        point_2.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point_2.time_from_start = rospy.Duration(1.5)
        point_1 = JointTrajectoryPoint()
        point_1.positions = [-0.030950937363057318, -0.2729300529345054, 0.8621137892472258, -0.47091233096887397, 2.593454601957924, -1.1111770403693226, 1.1182482627810466]
        point_1.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        point_1.time_from_start = rospy.Duration(2.0)
        trajrequest.points = [point_1, point_2, point_3, point_4]
        return trajrequest

    def BuildLibrary(self, times):
        #Set up the arm
        trajrequest = self.PrepTrajectory()
        #Send the trajectory to the trajectory controller
        result = self.traj_client(trajrequest, "setup")
        #Generate a series of trajectories
        codes = []
        for iteration in range(times):
            #Assemble a test
            print "Running an example trajectory set"
            #Assemble a trajectory
            test_name = "Wheel forward example" + str(iteration)
            trajrequest = self.GenForwardTrajectory()
            #Send the trajectory to the trajectory controller
            result = self.traj_client(trajrequest, test_name)
            #Save the return codes
            codes.append(result)
            #Wait a second for everything to stabilize
            time.sleep(0.5)
            #Assemble a trajectory
            test_name = "Wheel backward example" + str(iteration)
            trajrequest = self.GenBackwardTrajectory()
            #Send the trajectory to the trajectory controller
            result = self.traj_client(trajrequest, test_name)
            #Save the return codes
            codes.append(result)
            #Wait a second for everything to stabilize
            time.sleep(0.5)
        #Return the return codes
        return codes

if __name__ == '__main__':
    test = TestManager(10)
