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
import time
import trajectory
import os

from std_msgs.msg import String
from tf import *
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from arm_plan_pkg.srv import *

class TrajectoryManager:
    def __init__(self, arm_name):
        rospy.init_node(arm_name + 'trajectory_manager', anonymous=False)
        if arm_name == "l_arm_":
            self.arm = "left"
        else:
            self.arm = "right"
        print "Loading " + self.arm + " arm trajectory test manager"
        self.running = False
        #Allocate some empty status variables
        self.latest_feedback = []
        self.latest_status = []
        self.latest_result = []
        self.latest_state = []
        self.planned_trajectory = None
        self.current_trajectory = None
        #State options = 'IDLE' 'ACTIVE' 'ERROR' 'FAIL'
        self.current_state = 'IDLE'
        #Error options = 'NONE' 'PASS' 'MISS' 'SLIP'
        self.current_error = 'NONE'
        #Setup the publishers
        self.state_pub = rospy.Publisher("arm_plan_pkg/TaskState", String)
        self.error_pub = rospy.Publisher("arm_plan_pkg/TaskError", String)
        #self.trajectory_pub = rospy.Publisher(arm_name + "controller/joint_trajectory_action/goal", JointTrajectoryActionGoal)
        #self.cancel_pub = rospy.Publisher(arm_name + "controller/joint_trajectory_action/cancel", GoalID)
        print "Loaded all publishers"
        #Setup the service callback
        self.RequestHandler = rospy.Service("arm_plan_pkg/Trajectory_Request", TrajectoryRequest, self.RequestHandler)
        self.RecordHandler = rospy.Service("arm_plan_pkg/Trajectory_Record", TrajectoryRecord, self.RecordHandler)
        print "Service host up"
        #Load actionlib client
        self.client = actionlib.SimpleActionClient(arm_name + "controller/joint_trajectory_action", JointTrajectoryAction)
        self.client.wait_for_server()
        print "Actionlib client ready"
        #Setup the subscribers
        rospy.Subscriber(arm_name + "controller/state", JointTrajectoryControllerState, self.state_cb)
        rospy.Subscriber(arm_name + "controller/joint_trajectory_action/feedback", JointTrajectoryActionFeedback, self.feedback_cb)
        rospy.Subscriber(arm_name + "controller/joint_trajectory_action/status", GoalStatusArray, self.status_cb)
        rospy.Subscriber(arm_name + "controller/joint_trajectory_action/result", JointTrajectoryActionResult, self.result_cb)
        #Spin slowly
        print "Loaded all subscribers"
        rate = rospy.Rate(rospy.get_param('~hz', 30))
        while not rospy.is_shutdown():
            self.state_pub.publish(self.current_state)
            self.error_pub.publish(self.current_error)
            rate.sleep()

    def RecordHandler(self, request):
        #Populate & setup for the test
        traj_file = os.getcwd() + "/" + time.strftime("%d|%m|%Y_%H|%M|%S_trajectory.xml")
        self.latest_feedback = []
        self.latest_status = []
        self.latest_result = []
        self.latest_state = []
        self.planned_trajectory = request.command
        self.current_trajectory = trajectory.Trajectory()
        #Run the test
        '''
        Record trajectory states for the time given in the request
        '''
        goal = JointTrajectoryGoal()
        goal.trajectory = self.planned_trajectory
        self.running = True
        self.current_trajectory.Start()
        #Wait for the trajectory to complete
        time.sleep(request.delay)
        self.running = False
        self.current_trajectory.Stop(str(request.code))
        #End the test
        self.current_trajectory.ideal = self.planned_trajectory
        self.current_trajectory.states = self.latest_state
        self.current_trajectory.feedback = self.latest_feedback
        self.current_trajectory.result = self.latest_result
        self.current_trajectory.status = self.latest_status
        self.current_error = self.CharacterizeError()
        self.current_trajectory.ParseOut(traj_file)
        return traj_file

    def RequestHandler(self, request):
        #Populate & setup for the test
        traj_file = os.getcwd() + "/" + time.strftime("%d|%m|%Y_%H|%M|%S_trajectory.xml")
        self.latest_feedback = []
        self.latest_status = []
        self.latest_result = []
        self.latest_state = []
        self.planned_trajectory = request.command
        self.current_trajectory = trajectory.Trajectory()
        #Run the test
        '''
        Assemble the actionlib call, send it, and wait for it to complete
        During this process, the manager logs all data output from the controller
        '''
        goal = JointTrajectoryGoal()
        goal.trajectory = self.planned_trajectory
        self.running = True
        self.current_trajectory.Start()
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        self.running = False
        self.current_trajectory.Stop(str(request.code))
        #End the test
        self.current_trajectory.ideal = self.planned_trajectory
        self.current_trajectory.states = self.latest_state
        self.current_trajectory.feedback = self.latest_feedback
        self.current_trajectory.result = self.latest_result
        self.current_trajectory.status = self.latest_status
        self.current_error = self.CharacterizeError()
        self.current_trajectory.ParseOut(traj_file)
        return traj_file

    def CharacterizeError(self):
        return 'NONE'

    def state_cb(self, message):
        if self.running:
            self.latest_state.append(message)

    def feedback_cb(self, message):
        if self.running:
            self.latest_feedback.append(message)

    def status_cb(self, message):
        if self.running:
            self.latest_status.append(message)

    def result_cb(self, message):
        if self.running:
            self.latest_result.append(message)

if __name__ == '__main__':
    TrajectoryManager("l_arm_")
