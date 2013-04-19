#!/usr/bin/python

#   Calder Phillips-Grafflin -- WPI/Drexel Darpa Robotics Challenge Team
#
#   Manages execution of a given trajectory using a PR2
#   Each trajectory is added to the library of trajectories
#   for use with Dynamic Time Warping to classify and identify
#   new trajectories as encountered in execution.
#

import roslib; roslib.load_manifest('task_control')
import rospy
import math
import time
import subprocess
from two_arm_trajectory import *
from trajectory_analyzer import *
from evaluator import *
from joint_to_pose_converter import *
import os

from std_msgs.msg import String
from tf import *
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from task_control.msg import *
from task_control.srv import *

class PR2TrajectoryController:
    def __init__(self, library_path, error_threshold):
        self.library_path = library_path
        rospy.init_node('pr2_trajectory_controller', anonymous=False)
        print "Loading PR2 (both arms) trajectory controller"
        self.evaluator = Evaluator(library_path, False, error_threshold, True, 50)
        print "Loaded evaluator"
        self.converter = JointToPoseConverter()
        print "Loaded FK converter"
        self.running = False
        #Allocate some empty status variables
        self.left_arm_states = []
        self.right_arm_states = []
        #Setup the publishers
        self.state_pub = rospy.Publisher("task_control/TaskState", String)
        print "Loaded state publisher"
        #Setup the service callback
        self.RequestHandler = rospy.Service("task_control/Trajectory_Request", PR2ManagedJointTrajectory, self.RequestHandler)
        print "ManagedJointTrajectory service host up"
        #Load arm actionlib clients
        self.left_arm_client = actionlib.SimpleActionClient("l_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.left_arm_client.wait_for_server()
        self.right_arm_client = actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.right_arm_client.wait_for_server()
        print "Arm actionlib clients ready"
        #Load gripper actionlib clients
        self.left_gripper_client = actionlib.SimpleActionClient("l_gripper_controller/gripper_action", Pr2GripperCommandAction)
        self.left_gripper_client.wait_for_server()
        self.right_gripper_client = actionlib.SimpleActionClient("r_gripper_controller/gripper_action", Pr2GripperCommandAction)
        self.right_gripper_client.wait_for_server()
        print "Gripper actionlib clients ready"
        #Setup the subscribers
        rospy.Subscriber("l_arm_controller/state", JointTrajectoryControllerState, self.left_arm_state_cb)
        rospy.Subscriber("r_arm_controller/state", JointTrajectoryControllerState, self.right_arm_state_cb)
        #Spin slowly
        print "Joint state subscribers ready"
        print "Task controller setup complete...running..."
        rate = rospy.Rate(rospy.get_param('~hz', 10))
        while not rospy.is_shutdown():
            if (self.running):
                self.state_pub.publish("ACTIVE")
            else:
                self.state_pub.publish("IDLE")
            rate.sleep()

    def RequestHandler(self, request):
        print "Executing a trajectory..."
        #Populate & setup for the test
        traj_file = self.library_path + "/joints/" + request.code + time.strftime("_%d|%m|%Y_%H|%M|%S_pr2_trajectory.xml")
        self.left_arm_states = []
        self.right_arm_states = []
        assert(len(request.left_gripper_commands) == 4)
        assert(len(request.right_gripper_commands) == 4)
        '''
        Run the prep trajectory without storing state
        '''
        #Run the prep gripper goal
        self.left_gripper_client.send_goal(request.left_gripper_commands[0])
        self.right_gripper_client.send_goal(request.right_gripper_commands[0])
        self.left_gripper_client.wait_for_result()
        self.right_gripper_client.wait_for_result()
        #Run the prep trajectory [unmanaged]
        left_goal = JointTrajectoryGoal()
        left_goal.trajectory = request.left_arm_command.prep_trajectory
        right_goal = JointTrajectoryGoal()
        right_goal.trajectory = request.right_arm_command.prep_trajectory
        self.left_arm_client.send_goal(left_goal)
        self.right_arm_client.send_goal(right_goal)
        self.left_arm_client.wait_for_result()
        self.right_arm_client.wait_for_result()
        '''
        Run the main trajectory and store state to XML
        '''
        #Run the main gripper goal
        self.left_gripper_client.send_goal(request.left_gripper_commands[1])
        self.right_gripper_client.send_goal(request.right_gripper_commands[1])
        time.sleep(2.0)
        #Run the main trajectory [managed]
        '''
        Assemble the actionlib call, send it, and wait for it to complete
        During this process, the manager logs all data output from the controller
        '''
        left_goal = JointTrajectoryGoal()
        left_goal.trajectory = request.left_arm_command.main_trajectory
        right_goal = JointTrajectoryGoal()
        right_goal.trajectory = request.right_arm_command.main_trajectory
        self.running = True
        self.left_arm_client.send_goal(left_goal)
        self.right_arm_client.send_goal(right_goal)
        self.left_arm_client.wait_for_result()
        self.right_arm_client.wait_for_result()
        self.running = False
        '''
        Runs the post trajectory without storing state
        '''
        #Run the post gripper goal
        self.left_gripper_client.send_goal(request.left_gripper_commands[2])
        self.right_gripper_client.send_goal(request.right_gripper_commands[2])
        self.left_gripper_client.wait_for_result()
        self.right_gripper_client.wait_for_result()
        #Run the post trajectory [unmanaged]
        left_goal = JointTrajectoryGoal()
        left_goal.trajectory = request.left_arm_command.post_trajectory
        right_goal = JointTrajectoryGoal()
        right_goal.trajectory = request.right_arm_command.post_trajectory
        self.left_arm_client.send_goal(left_goal)
        self.right_arm_client.send_goal(right_goal)
        self.left_arm_client.wait_for_result()
        self.right_arm_client.wait_for_result()
        #Run the ending gripper goal
        self.left_gripper_client.send_goal(request.left_gripper_commands[3])
        self.right_gripper_client.send_goal(request.right_gripper_commands[3])
        self.left_gripper_client.wait_for_result()
        self.right_gripper_client.wait_for_result()
        #Store & save the trajectory
        print "Trajectory execution complete..."
        current_trajectory = self.StoreTrajectory(request.code)
        current_trajectory.Write(traj_file)
        #Analyze it
        error_code = self.evaluate_trajectory(traj_file)
        #Return the identified error code
        print "Trajectory evaluation complete with code [" + error_code + "]"
        return error_code

    def StoreTrajectory(self, trajectory_code):
        new_traj = TwoArmTrajectory(None, "JOINT", trajectory_code)
        new_traj.BuildFromRawRos(self.left_arm_states, self.right_arm_states, "JOINT", trajectory_code)
        return new_traj

    def evaluate_trajectory(self, joint_trajectory_file):
        #First, convert the joint-space trajectory to pose-space
        filename = self.library_path + "/pose/" + joint_trajectory_file.split("/")[-1]
        print "Converting joint trajectory to pose trajectory"
        #print "Converting joint to pose trajectory with filename: " + filename
        pose_trajectory_file = self.converter.ConvertTwoArmTrajectoryFile(joint_trajectory_file, filename)
        #Evaluate the provided trajectory wrt the trajectory library
        success_code = self.evaluator.EvaluatePoseOnly(pose_trajectory_file)
        #Return the identified error code
        return success_code

    def left_arm_state_cb(self, message):
        if self.running:
            self.left_arm_states.append(message)

    def right_arm_state_cb(self, message):
        if self.running:
            self.right_arm_states.append(message)

if __name__ == '__main__':
    path = subprocess.check_output("rospack find task_control", shell=True)
    path = path.strip("\n")
    PR2TrajectoryController(path + "/real_trajectory_library", -1.0)
