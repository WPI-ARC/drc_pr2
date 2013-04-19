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
from copy import deepcopy
from two_arm_trajectory import *
from trajectory_analyzer import *
from pose_evaluator import *
from joint_to_pose_converter import *
import os

from std_msgs.msg import String
from tf import *
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *
from task_control.msg import *
from task_control.srv import *

class PR2TrajectoryController:
    def __init__(self, library_path, error_threshold, safety_bounds, testing, enforce, interactive, ground_truth, gripper_threshold):
        self.testing = testing
        self.enforce = enforce
        self.interactive = interactive
        self.ground_truth = ground_truth
        self.gripper_threshold = gripper_threshold
        self.safety_bounds = safety_bounds
        self.library_path = library_path
        self.log_file_name = library_path + "/task_log.txt"
        rospy.init_node('pr2_trajectory_controller', anonymous=False)
        print bcolors.HEADER
        print "[STARTUP] Loading PR2 (both arms) trajectory controller"
        self.evaluator = Evaluator(library_path, True, error_threshold, True, 50, True, testing, False)
        print "[STARTUP] Loaded evaluator"
        library_stats = self.evaluator.GetLibraryStats()
        print bcolors.BOLD + "[STARTUP] Library size is " + str(library_stats['total']) + " trajectories (" + str(library_stats['KG']) + " KG, " + str(library_stats['KB']) + " KB)" + bcolors.ENDC + bcolors.HEADER
        self.running = False
        #Allocate some empty status variables
        self.left_arm_states = []
        self.right_arm_states = []
        self.latest_left = None
        self.latest_right = None
        self.latest_left_gripper = None
        self.latest_right_gripper = None
        #Setup the publishers
        self.state_pub = rospy.Publisher("task_control/TaskState", String)
        print "[STARTUP] Loaded state publisher"
        #Setup the service callback
        self.RequestHandler = rospy.Service("task_control/Trajectory_Request", PR2ManagedJointTrajectory, self.RequestHandler)
        print "[STARTUP] ManagedJointTrajectory service host up"
        #Load arm actionlib clients
        self.left_arm_client = actionlib.SimpleActionClient("l_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.left_arm_client.wait_for_server()
        self.right_arm_client = actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action", JointTrajectoryAction)
        self.right_arm_client.wait_for_server()
        print "[STARTUP] Arm actionlib clients ready"
        #Load gripper actionlib clients
        self.left_gripper_client = actionlib.SimpleActionClient("l_gripper_controller/gripper_action", Pr2GripperCommandAction)
        self.left_gripper_client.wait_for_server()
        self.right_gripper_client = actionlib.SimpleActionClient("r_gripper_controller/gripper_action", Pr2GripperCommandAction)
        self.right_gripper_client.wait_for_server()
        print "[STARTUP] Gripper actionlib clients ready"
        #Setup the subscribers
        rospy.Subscriber("l_arm_cart_controller/state/pose", PoseStamped, self.left_arm_state_cb)
        rospy.Subscriber("r_arm_cart_controller/state/pose", PoseStamped, self.right_arm_state_cb)
        rospy.Subscriber("l_arm_controller/state", JointTrajectoryControllerState, self.left_arm_joint_state_cb)
        rospy.Subscriber("r_arm_controller/state", JointTrajectoryControllerState, self.right_arm_joint_state_cb)
        #Spin slowly
        print "[STARTUP] Joint state subscribers ready"
        rospy.Subscriber("l_gripper_controller/state", JointControllerState, self.left_gripper_state_cb)
        rospy.Subscriber("r_gripper_controller/state", JointControllerState, self.right_gripper_state_cb)
        print "[STARTUP] Gripper state subscribers ready"        
        rospy.on_shutdown(self.cleanup)
        print bcolors.OKGREEN + bcolors.BOLD + "[STARTUP] Task controller running..."
        print bcolors.ENDC
        rate = rospy.Rate(rospy.get_param('~hz', 10))
        while not rospy.is_shutdown():
            if (self.running):
                self.state_pub.publish("ACTIVE")
            else:
                self.state_pub.publish("IDLE")
            rate.sleep()

    def cleanup(self):
        self.evaluator.cleanup()

    def __calc_cost_between_states__(self, start_left, start_right, end_left, end_right):
        left_dist = euclidean_distance(start_left, end_left)
        if (abs(left_dist) > abs(self.safety_bounds)):
            print bcolors.FAIL + bcolors.BOLD + "[SAFETY] FAIL Joint state distance (left): " + str(left_dist) + bcolors.ENDC
            return False
        print bcolors.WARNING + "[SAFETY] PASS Joint state distance (left): " + str(left_dist) + bcolors.ENDC
        right_dist = euclidean_distance(start_right, end_right)
        if (abs(right_dist) > abs(self.safety_bounds)):
            print bcolors.FAIL + bcolors.BOLD + "[SAFETY] FAIL Joint state distance (right): " + str(right_dist) + bcolors.ENDC
            return False
        print bcolors.WARNING + "[SAFETY] PASS Joint state distance (right): " + str(right_dist) + bcolors.ENDC
        #Safe trajectory
        return True

    def __sanitize__(self, request):
        #Checks to make sure that the first configuration
        #of the trajectory is within safety bounds
        #Check the start of the prep trajectory
        #Check LEFT
        start_left = deepcopy(self.latest_left)
        start_right = deepcopy(self.latest_right)
        start_prep_left = request.left_arm_command.prep_trajectory.points[0].positions
        start_prep_right = request.right_arm_command.prep_trajectory.points[0].positions
        s2p_possible = self.__calc_cost_between_states__(start_left, start_right, start_prep_left, start_prep_right)
        if (not s2p_possible):
            print bcolors.FAIL + bcolors.BOLD + "[SAFETY] ABORT Current to Prep distance too high" + bcolors.ENDC
            return False
        #Check the prep to main step
        end_prep_left = request.left_arm_command.prep_trajectory.points[-1].positions
        end_prep_right = request.right_arm_command.prep_trajectory.points[-1].positions
        start_main_left = request.left_arm_command.main_trajectory.points[0].positions
        start_main_right = request.right_arm_command.main_trajectory.points[0].positions
        p2m_possible = self.__calc_cost_between_states__(end_prep_left, end_prep_right, start_main_left, start_main_right)
        if (not p2m_possible):
            print bcolors.FAIL + bcolors.BOLD + "[SAFETY] ABORT Prep to Main distance too high" + bcolors.ENDC
            return False
        #Check the main to post step
        end_main_left = request.left_arm_command.main_trajectory.points[-1].positions
        end_main_right = request.right_arm_command.main_trajectory.points[-1].positions
        start_post_left = request.left_arm_command.post_trajectory.points[0].positions
        start_post_right = request.right_arm_command.post_trajectory.points[0].positions
        m2p_possible = self.__calc_cost_between_states__(end_main_left, end_main_right, start_post_left, start_post_right)
        if (not m2p_possible):
            print bcolors.FAIL + bcolors.BOLD + "[SAFETY] ABORT Main to Post distance too high" + bcolors.ENDC
            return False
        #Safe trajectory
        return True
    
    def RequestHandler(self, request):
        print bcolors.OKBLUE + "----------\n[STATUS] Executing a trajectory..." + bcolors.ENDC
        if (self.__sanitize__(request) == False):
            print bcolors.FAIL + bcolors.BOLD + "[STATUS] Trajectory evaluation complete with code [UNSAFE TRAJECTORY]" + bcolors.ENDC
            log_file = open(self.log_file_name, "a")
            log_file.write("Trajectory: " + request.code + " evaluation failure due to safety check failure\n")
            log_file.close()
            return "UNSAFE TRAJECTORY"
        assert(len(request.left_gripper_commands) == 5)
        assert(len(request.right_gripper_commands) == 5)
        '''
        Run the prep trajectory without storing state
        '''
        if (self.testing):
            print bcolors.CYAN + "[DEBUG] Running prep trajectory" + bcolors.ENDC
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
        waiting_time = float(request.left_arm_command.prep_trajectory.points[-1].time_from_start.secs) + (float(request.left_arm_command.prep_trajectory.points[-1].time_from_start.nsecs) / 1000000000)
        if (self.testing):
            print bcolors.CYAN + "[DEBUG] Estimated execution time: " + str(waiting_time) + bcolors.ENDC
        if (self.enforce):
            time.sleep(waiting_time + 1.0)
            self.left_arm_client.cancel_goal()
            self.right_arm_client.cancel_goal()
            print bcolors.CYAN + "[DEBUG] Enforced timeout" + bcolors.ENDC
        else:
            self.left_arm_client.wait_for_result()
            self.right_arm_client.wait_for_result()
        '''
        Runs the core of the trajectory with the ability to cycle
        '''
        error_codes = []
        for i in range(3):
            error_codes.append(self.ExecuteCoreTrajectory(request))
        '''
        Runs the post trajectory without storing state
        '''
        if (self.testing):
            print bcolors.CYAN + "[DEBUG] Running post trajectory" + bcolors.ENDC
        #Run the post gripper goal
        self.left_gripper_client.send_goal(request.left_gripper_commands[3])
        self.right_gripper_client.send_goal(request.right_gripper_commands[3])
        self.left_gripper_client.wait_for_result()
        self.right_gripper_client.wait_for_result()
        #Run the post trajectory [unmanaged]
        left_goal = JointTrajectoryGoal()
        left_goal.trajectory = request.left_arm_command.post_trajectory
        right_goal = JointTrajectoryGoal()
        right_goal.trajectory = request.right_arm_command.post_trajectory
        self.left_arm_client.send_goal(left_goal)
        self.right_arm_client.send_goal(right_goal)
        waiting_time = float(request.left_arm_command.post_trajectory.points[-1].time_from_start.secs) + (float(request.left_arm_command.post_trajectory.points[-1].time_from_start.nsecs) / 1000000000)
        if (self.testing):
            print bcolors.CYAN + "[DEBUG] Estimated execution time: " + str(waiting_time) + bcolors.ENDC
        if (self.enforce):
            time.sleep(waiting_time + 1.0)
            self.left_arm_client.cancel_goal()
            self.right_arm_client.cancel_goal()
            print bcolors.CYAN + "[DEBUG] Enforced timeout" + bcolors.ENDC
        else:
            self.left_arm_client.wait_for_result()
            self.right_arm_client.wait_for_result()
        #Run the ending gripper goal
        self.left_gripper_client.send_goal(request.left_gripper_commands[4])
        self.right_gripper_client.send_goal(request.right_gripper_commands[4])
        self.left_gripper_client.wait_for_result()
        self.right_gripper_client.wait_for_result()
        print bcolors.OKGREEN + bcolors.BOLD + "[STATUS] Full trajectory execution complete" + bcolors.ENDC
        results_string = str(error_codes)
        return results_string
    
    def ExecuteCoreTrajectory(self, request):
        #Populate & setup for the test
        self.left_arm_states = []
        self.right_arm_states = []
        '''
        Run the main trajectory and store state to XML
        '''
        if (self.testing):
            print bcolors.CYAN + "[DEBUG] Running main trajectory" + bcolors.ENDC
        #Run the main gripper goal
        self.left_gripper_client.send_goal(request.left_gripper_commands[1])
        self.right_gripper_client.send_goal(request.right_gripper_commands[1])
        time.sleep(2.0)
        #Run the main trajectory [managed]
        '''
        Assemble the actionlib call, send it, and wait for it to complete
        During this process, the manager logs all data output from the controller
        '''
        left_start = deepcopy(self.latest_left_gripper)
        right_start = deepcopy(self.latest_right_gripper)
        left_goal = JointTrajectoryGoal()
        left_goal.trajectory = request.left_arm_command.main_trajectory
        right_goal = JointTrajectoryGoal()
        right_goal.trajectory = request.right_arm_command.main_trajectory
        self.running = True
        self.left_arm_client.send_goal(left_goal)
        self.right_arm_client.send_goal(right_goal)
        waiting_time = float(request.left_arm_command.main_trajectory.points[-1].time_from_start.secs) + (float(request.left_arm_command.main_trajectory.points[-1].time_from_start.nsecs) / 1000000000)
        if (self.testing):
            print bcolors.CYAN + "[DEBUG] Estimated execution time: " + str(waiting_time) + bcolors.ENDC
        if (self.enforce):
            time.sleep(waiting_time + 1.0)
            self.left_arm_client.cancel_goal()
            self.right_arm_client.cancel_goal()
            print bcolors.CYAN + "[DEBUG] Enforced timeout" + bcolors.ENDC
        else:
            self.left_arm_client.wait_for_result()
            self.right_arm_client.wait_for_result()
        left_end = deepcopy(self.latest_left_gripper)
        right_end = deepcopy(self.latest_right_gripper)
        self.running = False
        if (self.testing):
            print bcolors.CYAN + "[DEBUG] Running return trajectory" + bcolors.ENDC
        #Run the return gripper goal
        self.left_gripper_client.send_goal(request.left_gripper_commands[2])
        self.right_gripper_client.send_goal(request.right_gripper_commands[2])
        self.left_gripper_client.wait_for_result()
        self.right_gripper_client.wait_for_result()
        #Run the return trajectory [managed]
        '''
        Assemble the actionlib call, send it, and wait for it to complete
        During this process, the manager logs all data output from the controller
        '''
        left_goal = JointTrajectoryGoal()
        left_goal.trajectory = request.left_arm_command.return_trajectory
        right_goal = JointTrajectoryGoal()
        right_goal.trajectory = request.right_arm_command.return_trajectory
        self.left_arm_client.send_goal(left_goal)
        self.right_arm_client.send_goal(right_goal)
        waiting_time = float(request.left_arm_command.main_trajectory.points[-1].time_from_start.secs) + (float(request.left_arm_command.main_trajectory.points[-1].time_from_start.nsecs) / 1000000000)
        if (self.testing):
            print bcolors.CYAN + "[DEBUG] Estimated execution time: " + str(waiting_time) + bcolors.ENDC
        if (self.enforce):
            time.sleep(waiting_time + 1.0)
            self.left_arm_client.cancel_goal()
            self.right_arm_client.cancel_goal()
            print bcolors.CYAN + "[DEBUG] Enforced timeout" + bcolors.ENDC
        else:
            self.left_arm_client.wait_for_result()
            self.right_arm_client.wait_for_result()
        #Store & save the trajectory
        print bcolors.OKBLUE + "[STATUS] Trajectory execution complete..." + bcolors.ENDC
        traj_code = request.code
        if (self.ground_truth):
            traj_code = self.AssessGroundTruth(left_start, right_start, left_end, right_end)
            print bcolors.CYAN + "[DEBUG] Assessed ground truth: " + traj_code + bcolors.ENDC
        if (self.interactive):
            traj_code = raw_input(bcolors.QUESTION + "[INTERACTIVE] Enter trajectory type (KB or KG): " + bcolors.ENDC)
        traj_file = self.library_path + "/pose/" + traj_code + time.strftime("_%d|%m|%Y_%H|%M|%S_pr2_trajectory.xml")
        current_trajectory = self.StoreTrajectory(traj_code)
        current_trajectory.Write(traj_file)
        #Analyze it
        error_code = self.evaluate_trajectory(traj_file)
        #Return the identified error code
        print bcolors.OKGREEN + "[STATUS] Trajectory evaluation complete with code [" + str(error_code) + "]" + bcolors.ENDC
        return error_code

    def AssessGroundTruth(self, left_start, right_start, left_end, right_end):
        left_grasp = self.EvaluateGrasp(left_start, left_end)
        right_grasp = self.EvaluateGrasp(right_start, right_end)
        if (left_grasp == "GOOD" and right_grasp == "GOOD"):
            return "KG"
        else:
            return "KB"

    def StoreTrajectory(self, trajectory_code):
        new_traj = TwoArmTrajectory(None, "POSE", trajectory_code)
        new_traj.BuildFromRawRos(self.left_arm_states, self.right_arm_states, "POSE", trajectory_code)
        return new_traj

    def __gen_log_string__(self, results_dict):
        log_string = "trajectory: [" + results_dict['pose file'] + "]|best match trajectory: [" + results_dict['match'] + "]|match cost: [" + results_dict['cost'] + "]|assigned code: [" + results_dict['code'] + "]\n"
        return log_string

    def evaluate_trajectory(self, pose_trajectory_file):
        #Evaluate the provided trajectory wrt the trajectory library
        success_dict = self.evaluator.Evaluate(pose_trajectory_file)
        print "[INFO] Best match trajectory: " + success_dict['match']
        print "[INFO] Best match cost: " + success_dict['cost']
        print "[INFO] Characterized error code: " + success_dict['code']
        #Log the data
        log_file = open(self.log_file_name, "a")
        log_file.write(self.__gen_log_string__(success_dict))
        log_file.close()
        #Return the identified error code
        success_code = success_dict['code']
        correctness = success_dict['correct']
        return [success_code, correctness]

    def EvaluateGrasp(self, start_gripper, end_gripper):
        print bcolors.WARNING + bcolors.BOLD + "[GROUND TRUTH DEBUG] Gripper started at: " + str(start_gripper) + " and ended at: " + str(end_gripper) + bcolors.ENDC
        if (start_gripper < self.gripper_threshold):
            return "BAD"
        if (end_gripper < self.gripper_threshold):
            return "FAILED"
        return "GOOD"

    def left_gripper_state_cb(self, message):
        self.latest_left_gripper = message.process_value

    def right_gripper_state_cb(self, message):
        self.latest_right_gripper = message.process_value

    def left_arm_joint_state_cb(self, message):
        self.latest_left = message.actual.positions

    def right_arm_joint_state_cb(self, message):
        self.latest_right = message.actual.positions

    def left_arm_state_cb(self, message):
        if self.running:
            self.left_arm_states.append(message)

    def right_arm_state_cb(self, message):
        if self.running:
            self.right_arm_states.append(message)

if __name__ == '__main__':
    path = subprocess.check_output("rospack find task_control", shell=True)
    path = path.strip("\n")
    library = rospy.get_param("task_control/mode", "simulation")
    if (library != "simulation" and library != "real"):
        library = "simulation"
    testing = rospy.get_param("task_control/testing", False)
    if (testing != False and testing != True):
        testing = False
    enforce = rospy.get_param("task_control/enforce", False)
    if (enforce != False and enforce != True):
        enforce = False
    interactive = rospy.get_param("task_control/interactive", False)
    if (interactive != False and interactive != True):
        interactive = False
    ground_truth = rospy.get_param("task_control/ground_truth", False)
    if (ground_truth != False and ground_truth != True):
        ground_truth = False
    safety = rospy.get_param("task_control/safety", "0.3")
    safety = float(safety)
    gripper_threshold = rospy.get_param("task_control/gripper_threshold", "0.008")
    gripper_threshold = float(gripper_threshold)
    threshold = rospy.get_param("task_control/threshold", "-1.0")
    threshold = float(threshold)
    full_path = path + "/" + library + "_trajectory_library"
    if (testing):
        print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting task controller in TESTING mode" + bcolors.ENDC
    else:
        print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting task controller in ACTUAL mode"
    if (library == "real"):
        print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting task controller in REAL mode" + bcolors.ENDC
    else:
        print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting task controller in SIMULATION mode"
    if (ground_truth):
        print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting task controller in GROUND TRUTH mode" + bcolors.ENDC
        print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting task controller with gripper threshold: " + str(gripper_threshold) + bcolors.ENDC
    if (interactive):
        print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting task controller in INTERACTIVE mode" + bcolors.ENDC
    else:
        print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting task controller in NORMAL mode"
    if (enforce):
        print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting task controller in ENFORCE mode" + bcolors.ENDC
    else:
        print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting task controller in LAISSEZ-FAIRE mode"
    print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting task controller with safety: " + str(safety) + bcolors.ENDC
    print bcolors.HEADER + bcolors.BOLD + "[STARTUP] Starting task controller with threshold: " + str(threshold) + bcolors.ENDC
    PR2TrajectoryController(full_path, threshold, safety, testing, enforce, interactive, ground_truth, gripper_threshold)
