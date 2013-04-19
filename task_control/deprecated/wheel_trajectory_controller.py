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
from feedback_wheel.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from arm_plan_pkg.srv import *

class WheelTrajectoryManager:
    def __init__(self, arm_name):
        rospy.init_node(arm_name + 'wheel_trajectory_manager', anonymous=False)
        if arm_name == "l_":
            self.arm = "left"
        else:
            self.arm = "right"
        print "Loading " + self.arm + " arm test manager"
        #Setup the service clients
        rospy.wait_for_service("arm_plan_pkg/Trajectory_Request")
        self.traj_client = rospy.ServiceProxy("arm_plan_pkg/Trajectory_Request", TrajectoryRequest)
        print "Service client up"
        #Setup the wheel controller
        self.wheel_pub = rospy.Publisher("feedback_wheel/braking_force", BrakeForce)
        self.wheel_pub.publish(0.0)
        print "Wheel controller loaded"
        #Setup the actionlib clients
        self.arm_client = actionlib.SimpleActionClient(arm_name + "arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.arm_client.wait_for_server()
        self.gripper_client = actionlib.SimpleActionClient(arm_name + "gripper_controller/gripper_action", Pr2GripperCommandAction)
        self.gripper_client.wait_for_server()
        print "Actionlib clients ready"
        #Build library
        print "--- Building example library ---"
        library = self.BuildLibrary(4)
        print "*** Example library built ***"
        print "Complete...!"
        

    def GenStartToGrasp(self):
        #Generate the trajectory from the safe start position to the grasp start point
        trajrequest = JointTrajectory()
        trajrequest.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        start_point = JointTrajectoryPoint()
        start_point.positions = [0.7193539497176639, -0.19950165967983868, 0.5779650369197873, -1.4741733608567211, 1.354890623259473, -1.4270451066123635, -3.198271700299716]
        start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        start_point.time_from_start = rospy.Duration(1.0)
        end_point = JointTrajectoryPoint()
        end_point.positions = [0.5542868745599052, -0.14341522566158524, 1.2089612786852895, -1.477937399352693, 1.0123780214252047, -0.8743512945872154, -2.949183132729538]
        end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        end_point.time_from_start = rospy.Duration(2.0)
        trajrequest.points = [start_point, end_point]
        return trajrequest

    def GenAbort(self):
        trajrequest = JointTrajectory()
        trajrequest.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        start_point = JointTrajectoryPoint()
        start_point.positions = [0.06828828361325467, -0.5611449559513824, 0.44599301481737097, -1.4563665633565472, 1.9413443907642556, -2.136588575596549, -3.8924130115686446]
        start_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        start_point.time_from_start = rospy.Duration(1.0)
        end_point = JointTrajectoryPoint()
        end_point.positions = [0.7193539497176639, -0.19950165967983868, 0.5779650369197873, -1.4741733608567211, 1.354890623259473, -1.4270451066123635, -3.198271700299716]
        end_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        end_point.time_from_start = rospy.Duration(2.0)
        trajrequest.points = [start_point, end_point]
        return trajrequest

    def GenTest(self, aligned):
        trajrequest = JointTrajectory()
        trajrequest.joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        raw_points = [[0.4882932071437446, -0.14375360535400306, 1.1462625415746641, -1.368780282969513, 0.9681828469949764, -1.0293300679819708, -2.936304492380058],[0.48389915642382436, -0.1472219972012857, 1.1440175740054856, -1.3583567917498989, 0.9681828469949764, -1.0453848594987207, -2.9330413233725885],[0.23269210583215738, -0.31378940079395695, 1.1190022210917834, -0.9712399095103458, 1.094578731981951, -1.6529869286895569, -2.755916509647139],[-0.07008286075555459, -0.38772536358725035, 1.1186815114390436, -0.6903847294263019, 1.5557935562387697, -2.0472212537119816, -2.4652334144617494],[-0.07257005927626414, -0.3875561737410414, 1.118040092133564, -0.6890817930238502, 1.5655118668464638, -2.0494837175571607, -2.4625358614155743]]
        if (aligned == False):
            raw_points[0] = self.GenerateMisalignedStart(raw_points[0])
        actual_points = []
        start_time = 0.5
        for point in raw_points:
            new_point = JointTrajectoryPoint()
            new_point.positions = point
            new_point.velocities = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
            new_point.time_from_start = rospy.Duration(start_time)
            start_time = start_time + 2.0
            actual_points.append(new_point)
        trajrequest.points = actual_points
        return trajrequest

    def GenerateMisalignedStart(self, current_start):
        new_start = list(current_start)
        #Add error to shoulder pan
        error = random.uniform(-0.02,0.02)
        new_start[0] = new_start[0] + error
        #Add error to shoulder lift
        error = random.uniform(-0.04,0.04)
        new_start[1] = new_start[1] + error
        #Add error to arm roll
        error = random.uniform(-0.06,0.06)
        new_start[2] = new_start[2] + error
        #Add error to elbow flex
        error = random.uniform(-0.08,0.08)
        new_start[3] = new_start[3] + error
        #Add error to forearm roll
        error = random.uniform(-0.10,0.10)
        new_start[4] = new_start[4] + error
        #Add error to wrist flex
        error = random.uniform(-0.12,0.12)
        new_start[5] = new_start[5] + error
        #Add error to wrist roll
        error = random.uniform(-0.14,0.14)
        new_start[6] = new_start[6] + error
        #return
        return new_start

    def RunTest(self, test_name, gripper_closure, trajectory_to_execute):
        result = None
        #Open gripper
        print "Opening gripper..."
        gopengoal = Pr2GripperCommandGoal()
        gopengoal.command.position = 0.09
        gopengoal.command.max_effort = -1.0
        self.gripper_client.send_goal(gopengoal)
        self.gripper_client.wait_for_result()
        #Send the arm to the prep position
        print "Moving the arm to the grasping position..."
        prep_trajectory = self.GenStartToGrasp()
        prepgoal = JointTrajectoryGoal()
        prepgoal.trajectory = prep_trajectory
        self.arm_client.send_goal(prepgoal)
        self.arm_client.wait_for_result()
        #Close gripper
        print "Closing the gripper..."
        gclosegoal = Pr2GripperCommandGoal()
        gclosegoal.command.position = gripper_closure
        gclosegoal.command.max_effort = -1.0
        self.gripper_client.send_goal(gclosegoal)
        time.sleep(2.0)
        #Assemble test trajectory
        if (trajectory_to_execute != None):
            #'''
            print "Running the turning trajectory..."
            #Send the trajectory to the trajectory controller
            result = self.traj_client(trajectory_to_execute, test_name)
            #'''
        #Open gripper
        print "Opening the gripper..."
        gopengoal = Pr2GripperCommandGoal()
        gopengoal.command.position = 0.09
        gopengoal.command.max_effort = -1.0
        self.gripper_client.send_goal(gopengoal)
        self.gripper_client.wait_for_result()
        #Send the arm back to the start
        print "Reseting the arm..."
        post_trajectory = self.GenAbort()
        postgoal = JointTrajectoryGoal()
        postgoal.trajectory = post_trajectory
        self.arm_client.send_goal(postgoal)
        self.arm_client.wait_for_result()
        return result

    def ParseTestCode(self, test_code):
        parsed = [0.0, "set at 0 degrees", 0.0, True]
        #Set braking force
        if ("NF" in test_code):
            parsed[0] = (0.0)
        elif ("MF" in test_code):
            parsed[0] = (-0.5)
        elif ("HF" in test_code):
            parsed[0] = (-1.0)
        else:
            parsed[0] = (0.0)
        #Set wheel stop
        if ("NS" in test_code):
            parsed[1] = "set at 0 degrees"
        elif ("PS" in test_code):
            parsed[1] = "set at 45 degrees"
        elif ("AS" in test_code):
            parsed[1] = "set at 90 degrees"
        else:
            parsed[1] = "set at 0 degrees"
        #Set gripper closure
        if ("GG" in test_code):
            parsed[2] = 0.0
        elif ("BG" in test_code):
            parsed[2] = 0.035 #Not quite closed around wheel
        else:
            parsed[2] = 0.0
        #Set alignment
        if ("FA" in test_code):
            parsed[3] = True
        elif ("BA" in test_code):
            parsed[3] = False
        else:
            parsed[3] = True
        #return
        return parsed

    def BuildLibrary(self, times):
        #Generate a series of trajectories
        codes = []
        test_suite = ["NF-NS-GG-FA","MF-NS-GG-FA","HF-NS-GG-FA","NF-PS-GG-FA","MF-PS-GG-FA","HF-PS-GG-FA","NF-AS-GG-FA","NF-NS-BG-FA","MF-NS-BG-FA","HF-NS-BG-FA","NF-PS-BG-FA","MF-PS-BG-FA","HF-PS-BG-FA","NF-AS-BG-FA","NF-NS-GG-BA","MF-NS-GG-BA","HF-NS-GG-BA","NF-PS-GG-BA","MF-PS-GG-BA","HF-PS-GG-BA","NF-AS-GG-BA","NF-NS-BG-BA","MF-NS-BG-BA","HF-NS-BG-BA","NF-PS-BG-BA","MF-PS-BG-BA","HF-PS-BG-BA","NF-AS-BG-BA"]
        min_test_suite = ["NF-NS-GG-FA", "NF-NS-BG-FA", "NF-NS-GG-BA", "NF-NS-BG-BA"]
        for test_code in min_test_suite:
            print "Running a new test code: " + test_code
            parsed = self.ParseTestCode(test_code)
            for iteration in range(times):
                #Assemble a test
                print "Running a wheel turning trajectory"
                #Assemble a trajectory
                print "Prepare test environment...!"
                print parsed[1]
                raw_input("Press enter to confirm setup is ready...")
                self.wheel_pub.publish(parsed[0])
                test_name = test_code + " trajectory example" + str(iteration)
                print "Running : " + test_name
                #Generate trajectory matching the alignment code
                full_traj = self.GenTest(parsed[3])
                #Run the finished trajectory
                self.RunTest(test_name, parsed[2], full_traj)
                #Wait a second for everything to stabilize
                time.sleep(1.0)
            #Close gripper
            print "Closing the gripper..."
            gclosegoal = Pr2GripperCommandGoal()
            gclosegoal.command.position = 0.0
            gclosegoal.command.max_effort = -1.0
            self.gripper_client.send_goal(gclosegoal)
            time.sleep(2.0)
        #Return the return codes
        return codes

if __name__ == '__main__':
    test = WheelTrajectoryManager("l_")
