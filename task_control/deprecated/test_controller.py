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
    def __init__(self, arm_name):
        rospy.init_node(arm_name + 'test_manager', anonymous=False)
        if arm_name == "l_":
            self.arm = "left"
        else:
            self.arm = "right"
        print "Loading " + self.arm + " arm test manager"
        
        #Setup the service clients
        rospy.wait_for_service("arm_plan_pkg/Trajectory_Request")
        self.traj_client = rospy.ServiceProxy("arm_plan_pkg/Trajectory_Request", TrajectoryRequest)
        #self.valv_client = rospy.ServiceClient
        print "Service host up"
        #Buld library
        print "--- Building example library ---"
        library = self.BuildLibrary(20)
        print "*** Example library built ***"
        #Run test
        print "--- Starting test run ---"
        codes = self.RunTests(20)
        print "*** Test run complete ***"
        '''
        print "--- Loading trajectory library ---"
        self.examples = []
        for code in library:
            self.examples.append(AnalysisTrajectory(code.result))
        print "*** Trajectory library loaded ***"
        print "--- Analyzing test results ---"
        self.trajectories = []
        for code in codes:
            self.trajectories.append(AnalysisTrajectory(code.result))
        self.evaluator = DTW()
        results = []
        for traj in self.trajectories:
            results.append(self.RunAnalysis(traj))
        correct = 0
        total = len(results)
        for pair in results:
            if (pair[0] == pair[1]):
                correct = correct + 1
        percent = (float(correct) / float(total)) * 100.0
        print "Percent of trajectories identified correctly: " + str(percent)
        print "*** Analysis complete ***"

    def RunAnalysis(self, traj):
        print "Loaded trajectory: " + traj.code
        #Evaluate by position and velocity info
        dtw_pos_dict = {}
        dtw_vel_dict = {}
        for other_traj in self.examples:
            poscost = self.evaluator.Evaluate(traj.actual_positions, other_traj.actual_positions)
            dtw_pos_dict[str(poscost)] = other_traj.code
            velcost = self.evaluator.Evaluate(traj.actual_velocities, other_traj.actual_velocities)
            dtw_vel_dict[str(velcost)] = other_traj.code
        #Find best-matching trajectory
        min_pos_code = sorted(dtw_pos_dict.keys())[0]
        min_vel_code = sorted(dtw_vel_dict.keys())[0]
        match = "None"
        if (float(min_pos_code) < float(min_vel_code)):
            match = dtw_pos_dict[min_pos_code]
            print "Best match from [position] trajectory is: " + match
        elif (float(min_vel_code) < float (min_pos_code)):
            match = dtw_vel_dict[min_vel_code]
            print "Best match from [velocity] trajectory is: " + match
        else:
            match = dtw_pos_dict[min_pos_code]
            print "Best match from both trajectories is: " + match
        actual = "Traj1"
        identified = "Traj1"
        if "Traj2" in traj.code:
            actual = "Traj2"
        if "Traj2" in match:
            identified = "Traj2"
        return [actual, identified]
        '''

    def GenTrajectory1(self):
        trajrequest = JointTrajectory()
        trajrequest.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        start_point = JointTrajectoryPoint()
        start_point.positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        start_point.time_from_start = rospy.Duration(1.0)
        end_point = JointTrajectoryPoint()
        end_point.positions = [-0.3,0.2,-0.1,-1.2,1.5,-0.3,0.5]
        end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        end_point.time_from_start = rospy.Duration(2.0)
        trajrequest.points = [start_point, end_point]
        return trajrequest

    def GenTrajectory2(self):
        trajrequest = JointTrajectory()
        trajrequest.joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        start_point = JointTrajectoryPoint()
        start_point.positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        start_point.time_from_start = rospy.Duration(1.0)
        end_point = JointTrajectoryPoint()
        end_point.positions = [-0.4,0.3,-0.2,-1.3,1.6,-0.4,0.6]
        end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        end_point.time_from_start = rospy.Duration(2.0)
        trajrequest.points = [start_point, end_point]
        return trajrequest

    def BuildLibrary(self, times):
        #Generate a series of trajectories
        codes = []
        for iteration in range(times):
            #Assemble a test
            print "Running an example trajectory"
            #Assemble a trajectory
            test_name = "Traj1 Wiimote Example" + str(iteration)
            trajrequest = self.GenTrajectory1()
            if ((iteration % 2) == 1):
                trajrequest = self.GenTrajectory2()
                test_name = "Traj2 Wiimote Example" + str(iteration)
            #Assemble a valve command
            #Set valve params
            #Send the trajectory to the trajectory controller
            result = self.traj_client(trajrequest, test_name)
            #Save the return codes
            codes.append(result)
            #Wait a second for everything to stabilize
            time.sleep(1.0)
        #Return the return codes
        return codes

    def RunTests(self, times):
        #Generate a series of trajectories
        codes = []
        for iteration in range(times):
            #Assemble a test
            print "Running a test trajectory"
            #Assemble a trajectory
            test_name = "Traj1 Wiimote Test" + str(iteration)
            trajrequest = self.GenTrajectory1()
            ctrl = random.uniform(0.0,1.0)
            if (ctrl > 0.5):
                trajrequest = self.GenTrajectory2()
                test_name = "Traj2 Wiimote Test" + str(iteration)
            #Assemble a valve command
            #Set valve params
            #Send the trajectory to the trajectory controller
            result = self.traj_client(trajrequest, test_name)
            #Save the return codes
            codes.append(result)
            #Wait a second for everything to stabilize
            time.sleep(1.0)
        #Return the return codes
        return codes

if __name__ == '__main__':
    test = TestManager("r_")
