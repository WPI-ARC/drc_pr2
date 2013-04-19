#!/usr/bin/env python
#
# Bener Suay, January 2013
# benersuay@wpi.edu
#

## OPENRAVE ##
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

## SYSTEM - FILE OPS ##
import sys
import os
from threading import Lock

## ROS ##
import roslib; 
roslib.load_manifest('std_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('move_base_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('std_srvs')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('trajectory_msgs')
roslib.load_manifest('gui')
roslib.load_manifest('tf')
#roslib.load_manifest('simple_robot_control')

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *
from gui.srv import *
from tf import *
import tf
#from simple_robot_control import *

## Constraint Based Manipulation ##
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *

## RAVE2ROS ##
from pr2_rave2task_control import *
from pr2_rave2ros_traj_play_from_file import *

class TrajectoryPlanner:
    def __init__(self,args=None):
        #self.mode = "openrave_only"
        self.mode = "openrave_with_ros"
        #self.which_planner = "birrt"
        self.which_planner = "cbirrt"
        RaveInitialize()
        RaveSetDebugLevel(3)
        self.env = Environment()
        self.env.Load('../../models/driving_wheel.robot2.xml')
          
        self.robot = self.env.ReadRobotURI('robots/pr2-beta-static.zae')
         
        self.jointDict = {'bl_caster_rotation_joint' : 0, 'bl_caster_l_wheel_joint' : 1, 'bl_caster_r_wheel_joint' : 2, 'br_caster_rotation_joint' : 3, 'br_caster_l_wheel_joint' : 4, 'br_caster_r_wheel_joint' : 5, 'fl_caster_rotation_joint' : 6, 'fl_caster_l_wheel_joint' : 7, 'fl_caster_r_wheel_joint' : 8, 'fr_caster_rotation_joint' : 9, 'fr_caster_l_wheel_joint' : 10, 'fr_caster_r_wheel_joint' : 11, 'torso_lift_joint' : 12, 'head_pan_joint' : 13, 'head_tilt_joint' : 14, 'l_shoulder_pan_joint' : 15, 'l_shoulder_lift_joint' : 16, 'l_upper_arm_roll_joint' : 17, 'l_elbow_flex_joint' : 18, 'l_forearm_roll_joint' : 19, 'l_wrist_flex_joint' : 20, 'l_wrist_roll_joint' : 21, 'l_gripper_l_finger_joint' : 22, 'l_gripper_motor_slider_joint' : 23, 'l_gripper_motor_screw_joint' : 24, 'l_gripper_joint' : 25, 'laser_tilt_mount_joint' : 26, 'r_shoulder_pan_joint' : 27, 'r_shoulder_lift_joint' : 28, 'r_upper_arm_roll_joint' : 29, 'r_elbow_flex_joint' : 30, 'r_forearm_roll_joint' : 31, 'r_wrist_flex_joint' : 32, 'r_wrist_roll_joint' : 33, 'r_gripper_l_finger_joint' : 34, 'r_gripper_motor_slider_joint' : 35, 'r_gripper_motor_screw_joint' : 36, 'r_gripper_joint' : 37, 'torso_lift_motor_screw_joint' : 38}

        self.lArmJointDict = {'l_shoulder_pan_joint' : 15, 'l_shoulder_lift_joint' : 16, 'l_upper_arm_roll_joint' : 17, 'l_elbow_flex_joint' : 18, 'l_forearm_roll_joint' : 19, 'l_wrist_flex_joint' : 20, 'l_wrist_roll_joint' : 21}
        
        self.rArmJointDict = {'r_shoulder_pan_joint' : 27, 'r_shoulder_lift_joint' : 28, 'r_upper_arm_roll_joint' : 29, 'r_elbow_flex_joint' : 30, 'r_forearm_roll_joint' : 31, 'r_wrist_flex_joint' : 32, 'r_wrist_roll_joint' : 33}
        
        self.jointDictReverseLookup = {0 : 'bl_caster_rotation_joint', 1 : 'bl_caster_l_wheel_joint', 2 :  'bl_caster_r_wheel_joint', 3 : 'br_caster_rotation_joint', 4 : 'br_caster_l_wheel_joint', 5 : 'br_caster_r_wheel_joint', 6 : 'fl_caster_rotation_joint', 7 : 'fl_caster_l_wheel_joint', 8 : 'fl_caster_r_wheel_joint', 9 : 'fr_caster_rotation_joint', 10 : 'fr_caster_l_wheel_joint', 11 : 'fr_caster_r_wheel_joint', 12 : 'torso_lift_joint', 13 : 'head_pan_joint', 14 : 'head_tilt_joint', 15 : 'l_shoulder_pan_joint', 16 : 'l_shoulder_lift_joint', 17 : 'l_upper_arm_roll_joint', 18 : 'l_elbow_flex_joint', 19 : 'l_forearm_roll_joint', 20 : 'l_wrist_flex_joint', 21 : 'l_wrist_roll_joint', 22 : 'l_gripper_l_finger_joint', 23 : 'l_gripper_motor_slider_joint', 24 : 'l_gripper_motor_screw_joint', 25 : 'l_gripper_joint', 26 : 'laser_tilt_mount_joint', 27 : 'r_shoulder_pan_joint', 28 : 'r_shoulder_lift_joint', 29 : 'r_upper_arm_roll_joint', 30 : 'r_elbow_flex_joint', 31 : 'r_forearm_roll_joint', 32 : 'r_wrist_flex_joint', 33 : 'r_wrist_roll_joint', 34 : 'r_gripper_l_finger_joint', 35 : 'r_gripper_motor_slider_joint', 36 : 'r_gripper_motor_screw_joint', 37 : 'r_gripper_joint', 38 : 'torso_lift_motor_screw_joint'}
         
        self.crankid = self.env.GetRobots()[0]
        self.env.Add(self.robot)
         
        self.probs_cbirrt = RaveCreateModule(self.env,'CBiRRT')
        self.probs_crankmover = RaveCreateModule(self.env,'CBiRRT')

        try:
             
            self.env.AddModule(self.probs_cbirrt,'pr2')
            self.env.AddModule(self.probs_crankmover,'crank')
             
        except openrave_exception, e:
            print e

        self.wristPitch = 0.4
        
        # printself.mode
        
        if(self.mode == "openrave_only"):
            self.env.SetViewer('qtcoin')
            self.moveArmsToInitPosition()
            self.openrave_only_set_wheel_pose()
            self.generate_points()
            self.arm_planner()

        self.planning_lock = Lock() # This will be set to False when we're using the trajectory planner
        
        
        if(self.mode == "openrave_with_ros"):
            #self.env.SetViewer('qtcoin')
            # print"initializing ros nodes"
            rospy.init_node('openrave', anonymous=False)

            # This subscriber gets the PR2's joint values 
            #rospy.Subscriber("joint_states", JointState, self.robot_joints_callback)

            # This service gets called when the wheel pose is updated by a human user in RViz (pose is in torso_lift_link)
            rospy.Service("update_valve_pose", updateValvePos, self.handle_update_valve_pose)

            # Call this service to make the head look infront of the robot
            rospy.Service("openrave_pr2_look_ahead",Empty,self.handle_look_ahead)

            self.base_vel_cmd_pub = rospy.Publisher("/base_controller/command", Twist)

            self.listener = TransformListener()

            rospy.on_shutdown(self.myhook)

            print "Robot ready to plan, waiting for a valve pose update..."

            rospy.spin()
        
    def myhook(self):
        print "shutdown time!"

    def handle_look_ahead(self,req):
        headGoal = PointHeadGoal()
        p = PointStamped()
        p.header.frame_id = "torso_lift_link"
        p.point.x = 1.0
        p.point.y = 0
        p.point.z = 0
        headGoal.target = p
        headGoal.pointing_frame = "head_mount_kinect_rgb_optical_frame"
        headGoal.min_duration = rospy.Duration(0.5)
        headGoal.max_velocity = 1.0
        headClient = actionlib.SimpleActionClient("head_traj_controller/point_head_action",PointHeadAction)
        headClient.send_goal(headGoal)
        headClient.wait_for_result()
        return []

    def rotate_base(self,difference):
        radians = atan(difference[1]/difference[0])
        # print"radians"
        # printradians

        if(abs(radians) <= 0.05):
           return 1 # good alignment
        else:
            speed = 0.15
            if radians < 0.0:
                speed = -1.0 * speed
                radians = -1.0 * radians

            velocity = Twist()
            velocity.linear.x = velocity.linear.y = velocity.linear.z = 0
            velocity.angular.x = velocity.angular.y = 0
            velocity.angular.z = speed      

            rate = rospy.Rate(10.0)

            found = False
            while not found:
                # print"waiting"
                if self.listener.frameExists("/base_footprint") and self.listener.frameExists("/odom_combined"):
                    t = self.listener.getLatestCommonTime("/base_footprint", "/odom_combined")
                    (trans_start,rot_start) = (trans_end,rot_end) = self.listener.lookupTransform("/base_footprint", "/odom_combined", t)
                    #print position, quaternion
                    #print "Found the transformation, moving on!"
                    found = True
                rate.sleep()

            #keep moving until rotation reached - since robot only rotates around z axis one can compare the w component of the quaternions to half the angle
            angle_turned = 0.0
            rot_start_inv = tf.transformations.quaternion_inverse(rot_start)
            while  angle_turned < radians:            
                self.base_vel_cmd_pub.publish(velocity)
                rospy.sleep(0.05)
                found = False
                while not found:
                    #print "waiting"
                    if self.listener.frameExists("/base_footprint") and self.listener.frameExists("/odom_combined"):
                        t = self.listener.getLatestCommonTime("/base_footprint", "/odom_combined")
                        (trans_end,rot_end) = self.listener.lookupTransform("/base_footprint", "/odom_combined", t)
                        #print "Found the transformation, moving on!"
                        found = True
                euler_angles = tf.transformations.euler_from_quaternion(tf.transformations.quaternion_multiply(rot_start_inv, rot_end))
                angle_turned = abs( euler_angles[2])
                # print"angle_turned"
                # printangle_turned
            return 2 # alignment might not be good, we don't know yet.

    def approach_base(self,difference):        
        distance = difference[0]-0.6
        # print"distance"
        # printdistance
        if((distance > -0.1) and (distance < 0.1)):
            return 1 # approach good.
        else:
            speed = 0.15      
            velocity = Twist()
            if distance < 0.0:
                speed = -1.0 * speed
                distance = -1.0 * distance

            velocity.linear.x = speed
            velocity.angular.x = velocity.angular.y = velocity.angular.z = velocity.linear.z = 0
            # #get current pose
            rate = rospy.Rate(10.0)

            found = False
            while not found:
                #print "waiting"
                if self.listener.frameExists("/base_footprint") and self.listener.frameExists("/odom_combined"):
                    t = self.listener.getLatestCommonTime("/base_footprint", "/odom_combined")
                    (trans_start,rot_start) = (trans_end,rot_end) = self.listener.lookupTransform("/base_footprint", "/odom_combined", t)
                    #print position, quaternion
                    #print "Found the transformation, moving on!"
                    found = True
                rate.sleep()


            # #keep moving until distance travelled
            while numpy.linalg.norm(numpy.array(trans_start) - trans_end) < distance:
                self.base_vel_cmd_pub.publish(velocity)
                rospy.sleep(0.05)
                found = False
                while not found:
                    #print "waiting"
                    if self.listener.frameExists("/base_footprint") and self.listener.frameExists("/odom_combined"):
                        t = self.listener.getLatestCommonTime("/base_footprint", "/odom_combined")
                        (trans_end,rot_end) = self.listener.lookupTransform("/base_footprint", "/odom_combined", t)
                        #print "Found the transformation, moving on!"
                        found = True
                rate.sleep()
            return 2 # There may still be some distance to go, we don't know yet.

    def openrave_only_set_wheel_pose(self):
         
        T0_tll = self.robot.GetLinks()[16].GetTransform() # Transform from the origin to torso_lift_link frame
          
        Ttll_wheel = MakeTransform(rotationMatrixFromQuat([-0.0434053950012,0.205337584019,0.0024010904599,0.977725327015]),matrix([0.561164438725,0.00322353374213,0.101688474417])) # Transform from torso_lift_link to racing wheel's frame

        # Wheel's model is defined rotated, the following transform will make it look straight up
        wheelStraightUp = MakeTransform(matrix(dot(rodrigues([0,0,-pi/2]),rodrigues([pi/2,0,0]))),transpose(matrix([0,0,0])))

        wheelStraightUp = dot(wheelStraightUp,MakeTransform(rodrigues([-0.4,0,0]),transpose(matrix([0,0,0])))) # For some reason we need to correct the 3D sketchup model for the real life wheel angle in the lab. This is the offset.

        wheelAtCorrectLocation = dot(T0_tll,Ttll_wheel) # Wheel's pose in world coordinates, adjusted for torso lift link transform
        wheelAtCorrectLocation = dot(wheelAtCorrectLocation,wheelStraightUp) # Rotate the model so that it looks straight up
         
        self.crankid.SetTransform(array(wheelAtCorrectLocation))
          

    def handle_update_valve_pose(self,req):
        wheel = req.valve
        
        # print"wheel pose.position"
        # printwheel.pose.position
        
        r = req.radius

        # print"Radius"
        # printr
         
        T0_tll = self.robot.GetLinks()[16].GetTransform() # Transform from the origin to torso_lift_link frame
         
        Ttll_wheel = MakeTransform(rotationMatrixFromQuat([wheel.pose.orientation.w,wheel.pose.orientation.x,wheel.pose.orientation.y,wheel.pose.orientation.z]),matrix([wheel.pose.position.x,wheel.pose.position.y,wheel.pose.position.z])) # Transform from torso_lift_link to racing wheel's frame

        # Wheel's model is defined rotated, the following transform will make it look straight up
        wheelStraightUp = MakeTransform(matrix(dot(rodrigues([0,0,-pi/2]),rodrigues([pi/2,0,0]))),transpose(matrix([0,0,0])))

        wheelStraightUp = dot(wheelStraightUp,MakeTransform(rodrigues([-0.4,0,0]),transpose(matrix([0,0,0])))) # For some reason we need to correct the 3D sketchup model for the real life wheel angle in the lab. This is the offset.

        wheelAtCorrectLocation = dot(T0_tll,Ttll_wheel) # Wheel's pose in world coordinates, adjusted for torso lift link transform
        wheelAtCorrectLocation = dot(wheelAtCorrectLocation,wheelStraightUp) # Rotate the model so that it looks straight up
         
        self.crankid.SetTransform(array(wheelAtCorrectLocation))
        
        res = updateValvePosResponse()

        cx = wheel.pose.position.x # current x value for the valve in torso_lift_link
        cy = wheel.pose.position.y # current y value for the valve in torso_lift_link
        # print"cx"
        # printcx
        # print"cy"
        # printcy
        d = [cx,cy] # delta
        # print"delta"
        # printd

        self.moveArmsToInitPosition()
        self.planning_lock.acquire()
        rot_res = self.rotate_base(d)
        if( rot_res == 1):
            appr_res = self.approach_base(d)
            if(appr_res == 1):
                # Now we know where the valve is located, let's run the test points
                if(self.generate_points() == 1):
                    # print"generate_points succeeded"
                    if(self.arm_planner() == 1):
                        # print"arm planner succeeded"
                        # Finally finish by playing the trajectories and return the code
                        task_control_response = pr2_rave2task_control(req.id) # This function packs movetraj files in 3 packages and sends them over to task_control.                
                        res.success_code = task_control_response.result # Just return whatever the task control is returning back to GUI
                    else:
                        res.success_code = "arm planner failed"
                else:
                    res.success_code = 'generate_points failed'
            else:
                # print"approach is not good enough. align again."
                res.success_code = 'retry'
        elif(rot_res == 2):
            # print"rotation is not good enough. align again."
            res.success_code = 'retry'
        self.planning_lock.release()
        print 'End of pipeline - result:'
        print res.success_code
        return res

    def moveArmsToInitPosition(self):

        self.robot.SetDOFValues([-0.85,-0.1,0.114,-1.9,0.0,-1.00,-1.00,0.5,0.85, -0.1,0.114,-1.9,0.0,-1.00,1.00,0.5], [27,28,29,30,31,32,33,34,15,16,17,18,19,20,21,22])
        #self.SetRobotDOFValuesInSyncWithROS(self.robot,[-0.85,-0.1,0.114,-1.9,0.0,-1.00,-1.00,0.09],[27,28,29,30,31,32,33,34])
        #self.SetRobotDOFValuesInSyncWithROS(self.robot,[0.85, -0.1,0.114,-1.9,0.0,-1.00,1.00,0.09],[15,16,17,18,19,20,21,22])
        return
        if(self.mode == "openrave_with_ros"):
            self.SetRobotDOFValuesInSyncWithROS(self.robot,[-0.85,-0.1,0.114,-1.9,0.0,-1.00,-1.00,0.09],[27,28,29,30,31,32,33,34])
            self.SetRobotDOFValuesInSyncWithROS(self.robot,[0.85, -0.1,0.114,-1.9,0.0,-1.00,1.00,0.09],[15,16,17,18,19,20,21,22])
        
        if(self.mode == "openrave_only"):
            self.robot.SetDOFValues([-0.85,-0.1,0.114,-1.9,0.0,-1.00,-1.00,0.5,0.85, -0.1,0.114,-1.9,0.0,-1.00,1.00,0.5], [27,28,29,30,31,32,33,34,15,16,17,18,19,20,21,22])
        
    def robot_joints_callback(self,robotJoints):
            jointNames = []
            jointIndices = []
            jointValues = []
            for n in range(len(robotJoints.name)): # Cycle through the names 
                if robotJoints.name[n] in self.jointDict: # If you find the same joint in openRAVE
                    ind = self.jointDict[robotJoints.name[n]] # Get the index
                    # Fill in the list
                    jointNames.append(robotJoints.name[n])
                    jointIndices.append(ind)
                    jointValues.append(robotJoints.position[n])
            # Update the joint values in openRAVE
            self.planning_lock.acquire()
            self.robot.SetDOFValues(jointValues, jointIndices)
            self.planning_lock.release()

    def SetRobotDOFValuesInSyncWithROS(self,robot,vals,inds):
        # printvals
        # printinds
        
        # First move joints in OpenRAVE
         
        robot.SetDOFValues(vals,inds)
         
        # Then move joints in ROS
        #
        # It is, however, trickier than it looks. ROS needs all 7 joints to run the action service (some of which might not be set to a value in "vals" list). That's why We need to:
        # 1. Figure out which joints are set in vals.
        # 2. Get the current values for the joints that are not enlisted in vals and inds (so that we can pass them as they are to Gazebo).
        # 3. Split the joints to right and left arms.
        # 4. Find if there is a request for the gripper (this is handled differently in ROS).
        # 5. Send commands to right action services (left arm, left gripper, right arm, right gripper).
        #
        # Let's-a go!
        
        # Get new trajectory variables for both arms
        lTraj = JointTrajectory()
        lTraj.points = []

        rTraj = JointTrajectory()
        rTraj.points = []

        lInds=[15,16,17,18,19,20,21]
        lVals=[]
        lNames=['l_shoulder_pan_joint','l_shoulder_lift_joint','l_upper_arm_roll_joint','l_elbow_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
        lCurrentPoint = JointTrajectoryPoint()

        rInds=[27,28,29,30,31,32,33]
        rVals=[]
        rNames=['r_shoulder_pan_joint','r_shoulder_lift_joint','r_upper_arm_roll_joint','r_elbow_flex_joint','r_forearm_roll_joint','r_wrist_flex_joint','r_wrist_roll_joint']
        rCurrentPoint = JointTrajectoryPoint()

        # These are used as flags to decide which action service to wait for before we continue with the rest of the code
        waitForRArm = False
        waitForLArm = False

        # Create a new dictionnary for the incoming joints as keys with incoming joint values as values
        requestedDict = {}
        for j in range(len(inds)):
            requestedDict[self.jointDictReverseLookup[inds[j]]] = vals[j]

        # now requestedDict looks like "joint_something : some_angle_value, joint_some_other_thing : some_angle_value, ... "

        # Here we'll split incoming indices into left / right arm indices
        # All we need to do is to look up for the ones we need and fill in the blanks if they are not there
        # Let's start with the left arm
        for j in range(len(lInds)):
            jName = self.jointDictReverseLookup[lInds[j]]
            if jName in requestedDict: # If this joint's value is passed in, then use it
                lVals.append(requestedDict[jName])
            else: # else, get the current value for that joint from OpenRAVE model
                lVals.append(self.robot.GetDOFValues([lInds[j]])[0]) # Note, GetDOFValues returns a list, we will only have one value in there, that's why we have a [0] in the end.

        # Let's do the same for the right arm
        for j in range(len(rInds)):
            jName = self.jointDictReverseLookup[rInds[j]]
            if jName in requestedDict:
                rVals.append(requestedDict[jName])
            else:
                rVals.append(self.robot.GetDOFValues([rInds[j]])[0])

        # Now we have two full arrays for both arms with either requested or current values inserted
        # Let's take care of the grippers (if they are requested)
        # Right gripper first
        if 'r_gripper_l_finger_joint' in requestedDict:
            rGripperGoal = Pr2GripperCommandGoal()
            rGripperClient = actionlib.SimpleActionClient("r_gripper_controller/gripper_action",Pr2GripperCommandAction)
            rGripperGoal.command.position = requestedDict['r_gripper_l_finger_joint']
            rGripperGoal.command.max_effort = -1.0 # Do not limit effort
            rGripperClient.wait_for_server()
            rGripperClient.send_goal(rGripperGoal)
            rGripperClient.wait_for_result()

    
        # Now let's do the same thing for the left gripper
        if 'l_gripper_l_finger_joint' in requestedDict:
            lGripperGoal = Pr2GripperCommandGoal()
            lGripperClient = actionlib.SimpleActionClient("l_gripper_controller/gripper_action",Pr2GripperCommandAction)
            lGripperGoal.command.position = requestedDict['l_gripper_l_finger_joint']
            lGripperGoal.command.max_effort = -1.0
            lGripperClient.wait_for_server()
            lGripperClient.send_goal(lGripperGoal)
            lGripperClient.wait_for_result()  

        # Now let's move the torso (a.k.a. torso lift link)
        if 'torso_lift_joint' in requestedDict:
            torsoClient = actionlib.SimpleActionClient("torso_controller/position_joint_action",SingleJointPositionAction)
            torsoGoal = SingleJointPositionGoal()
            torsoGoal.position = requestedDict['torso_lift_joint']
            torsoGoal.min_duration = rospy.Duration(2.0)
            torsoGoal.max_velocity = 1.0
            torsoClient.wait_for_server()
            torsoClient.send_goal(torsoGoal)
            torsoClient.wait_for_result()
    
        # Now let's move the arms, right arm first
        if (rNames != [] and rVals != [] and rInds != []):
            rTraj.joint_names = rNames
            rCurrentPoint.positions = rVals
            rCurrentPoint.velocities = [0.2]*len(rVals) # Note that this value should be fiddled with
            rCurrentPoint.time_from_start = rospy.Duration(2.0)
            rTraj.points.append(rCurrentPoint)
            rGoal = JointTrajectoryGoal()
            rGoal.trajectory = rTraj
            rArmClient = actionlib.SimpleActionClient("r_arm_controller/joint_trajectory_action", JointTrajectoryAction)
            rArmClient.wait_for_server()
            rArmClient.send_goal(rGoal)
            waitForRArm = True

        # And then the left arm
        if(lNames != [] and lVals != [] and lInds != []):
            lTraj.joint_names = lNames
            lCurrentPoint.positions = lVals
            lCurrentPoint.velocities = [0.2]*len(lVals)
            lCurrentPoint.time_from_start = rospy.Duration(2.0)
            lTraj.points.append(lCurrentPoint)
            lGoal = JointTrajectoryGoal()
            lGoal.trajectory = lTraj
            lArmClient = actionlib.SimpleActionClient("l_arm_controller/joint_trajectory_action", JointTrajectoryAction)
            lArmClient.wait_for_server()
            lArmClient.send_goal(lGoal)
            waitForLArm = True

        if waitForRArm:
            rArmClient.wait_for_result()
        
        if waitForLArm:
            lArmClient.wait_for_result()

        #print lTraj
        #print rTraj

    def is_valid(self,config,pname):
        if config == '':
                # print"Hey: No IK Solution found! Move the robot!"
                return 0
        else:
            # printstr2num(config)
            # printtype(str2num(config))
            self.robot.SetActiveDOFValues(str2num(config))
            #sys.stdin.readline()
            if(self.env.CheckCollision(self.robot,self.crankid)):
                print"Point "+pname+" is in collision."
                return 0
            else:
                print"Point "+pname+" is valid"
                # print"------"
                return 1

    def generate_points(self):
        # print"Testing points"
        self.notPlanning = False # We are planning: this will stop robot model and wheel model being updated            

          
        manips = self.robot.GetManipulators()
        crankmanip = self.crankid.GetManipulators()

        # print"Getting Loaded Problems"
        
        self.probs = self.env.GetLoadedProblems()
        # printself.probs[0] # --> self.probs_cbirrt
        # printself.probs[1] # --> self.probs_crankmover
          
        self.robot.SetActiveDOFs([15,16,17,18,19,20,21,27,28,29,30,31,32,33])
    
        self.initconfig = self.robot.GetActiveDOFValues()

        # print"initconfig"
        # printself.initconfig
        # printtype(self.initconfig)

        manip = self.robot.SetActiveManipulator('leftarm') # set the manipulator to leftarm
        self.T0_LH_INIT = manip.GetEndEffectorTransform() # end of manipulator 7

        manip = self.robot.SetActiveManipulator('rightarm') # set the manipulator to leftarm       
        self.T0_RH_INIT = manip.GetEndEffectorTransform() # end of manipulator 5
        
        self.robot.SetActiveDOFs([15,16,17,18,19,20,21,27,28,29,30,31,32,33])

        #print "Press enter to continue 1..."
        #sys.stdin.readline()

        links = self.robot.GetLinks()

        self.crankjointind = 0

        
        self.jointtm = self.probs[0].SendCommand('GetJointTransform name crank jointind '+str(self.crankjointind))

        maniptm = self.crankid.GetManipulators()[0].GetTransform()

        self.jointtm = self.jointtm.replace(" ",",")

        self.jointtm = eval('['+self.jointtm+']')

        rhanddofs = [34] # Right gripper links here
        rhandclosevals = [0.035] # What are the closed joint values for the right hand's links?--> Obtained experimentally
        rhandopenvals = [0.0548] # What are the open joint values for the right hand's links? --> Obtained experimentally
        lhanddofs = [22] # Left gripper links here
        lhandclosevals = [0.035] # Same as rhandclosevals for the left gripper --> Obtained experimentally
        lhandopenvals = [0.0548] # Same as rhandopenvals for the left gripper --> Obtained experimentally

        ##############################################################
        ## THIS IS WHERE THE FIRST BLOCK ENDS IN THE MATLAB VERSION ##
        ##############################################################

        self.robot.SetActiveDOFs([15,16,17,18,19,20,21,27,28,29,30,31,32,33])

        # Rotate the driving wheel's manipulator's coordinate frame 90 degrees around its Z-axis
        temp =  dot(maniptm,MakeTransform(matrix(rodrigues([0,0,pi/2])),transpose(matrix([0,0,0]))))

        # Rotate the new coordinate frame -90 degrees around its X-axis
        temp = dot(temp,MakeTransform(matrix(rodrigues([-pi/2,0,0])),transpose(matrix([0,0,0]))))

        # Rotate the new coordinate frame -30 degrees around its Y-axis
        temp = dot(temp,MakeTransform(matrix(rodrigues([0,-pi/8,0])),transpose(matrix([0,0,0]))))

        # Translate the new coordinate frame -0.11 meters on its Z-axis and assign it to Left Hand
        self.T0_LH1_APPROACH_1 = dot(temp,MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([0.1,0.1,-0.2]))))
        self.T0_LH1_APPROACH_2 = dot(temp,MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([0.03,0.0,-0.15]))))
        self.T0_LH1_ENTRY = dot(temp,MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([-0.01,0.01,-0.14]))))
       

        # Rotate the driving wheel's manipulator's coordinate frame 90 degrees around its Z-axis
        temp =  dot(maniptm,MakeTransform(matrix(rodrigues([0,0,pi/2])),transpose(matrix([0,0,0]))))

        # Rotate the new coordinate frame +90 degrees around its X-axis
        temp = dot(temp,MakeTransform(matrix(rodrigues([pi/2,0,0])),transpose(matrix([0,0,0]))))

        # Rotate the new coordinate frame 30 degrees around its Y-axis
        temp = dot(temp,MakeTransform(matrix(rodrigues([0,-pi/8,0])),transpose(matrix([0,0,0]))))

        # Translate the new coordinate frame -0.11 meters on its Z-axis and assign it to Left Hand
        self.T0_RH1_APPROACH_1 = dot(temp,MakeTransform(matrix(rodrigues([-pi/4,0,0])),transpose(matrix([0.1,-0.1,-0.2]))))
        self.T0_RH1_APPROACH_2 = dot(temp,MakeTransform(matrix(rodrigues([-pi/4,0,0])),transpose(matrix([0.03,0.0,-0.15]))))
        self.T0_RH1_ENTRY = dot(temp,MakeTransform(matrix(rodrigues([-pi/4,0,0])),transpose(matrix([-0.01,-0.01,-0.12]))))
        


        ################################################################
        ## THIS IS WHERE THE SECOND BLOCK ENDS IN THE MATLAB VERSION  ##
        ## APPROACH-1 IK IS OVER - NOW GO TO APPROACH-2               ##
        ################################################################
        
        arg1 = str(GetRot(self.T0_LH1_APPROACH_2)).strip("[]")+str(GetTrans(self.T0_LH1_APPROACH_2)).strip("[]")
        arg1 = arg1.replace("\n"," ")

        arg2 = str(GetRot(self.T0_RH1_APPROACH_2)).strip("[]")+str(GetTrans(self.T0_RH1_APPROACH_2)).strip("[]")
        arg2 = arg2.replace("\n"," ")        


        ####################################################################
        ## APPROACH-2 IK IS OVER - NOW GO TO STARTIK WHERE WE WILL GRASP  ##
        ####################################################################

        
        arg1 = str(GetRot(self.T0_LH1_ENTRY)).strip("[]")+str(GetTrans(self.T0_LH1_ENTRY)).strip("[]")
        arg1 = arg1.replace("\n"," ")

        arg2 = str(GetRot(self.T0_RH1_ENTRY)).strip("[]")+str(GetTrans(self.T0_RH1_ENTRY)).strip("[]")
        arg2 = arg2.replace("\n"," ")        
         
        self.startik = self.probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 7 '+arg1+' maniptm 5 '+arg2)
        # print"Got self.startik:"

        if(not self.is_valid(self.startik,"startik")):
            # print"Hey: No IK Solution found! Move the robot!"
            # self.notPlanning = True
            return []
         
        ##############################################################
        ## END OF STARTIK TSR DEFINITION AND TRAJECTORY EXEC. BLOCK ##
        ##############################################################

        # Left hand is evaluated first, so need to make right hand relative to crank
        cranklinks = self.crankid.GetLinks();

        self.T0_crankcrank = self.crankid.GetManipulators()[0].GetTransform()

        T0_w0L = eye(4)
        T0_w0R = MakeTransform(rodrigues([0,self.wristPitch,0]),matrix([[self.jointtm[9]],[self.jointtm[10]],[self.jointtm[11]]]))

        crank_rot = pi/4
        TSRChainMimicDOF = 1

        Tcrank_rot = MakeTransform(matrix(rodrigues([crank_rot,0,0])),transpose(matrix([0,0,0]))) # For the right gripper
        Tcrank_rot2 = MakeTransform(matrix(rodrigues([0,0,crank_rot])),transpose(matrix([0,0,0]))); # For the left gripper

        T0_cranknew = dot(T0_w0R,Tcrank_rot)

        temp = dot(linalg.inv(T0_w0R),self.T0_RH1_ENTRY)
        T0_RH2 = dot(T0_cranknew,temp);

        temp = dot(linalg.inv(self.T0_crankcrank),self.T0_LH1_ENTRY)
        temp = dot(Tcrank_rot2,temp)
        T0_LH2 = dot(self.T0_crankcrank,temp)

        arg1 = str(GetRot(T0_LH2)).strip("[]")+str(GetTrans(T0_LH2)).strip("[]")
        arg2 = str(GetRot(T0_RH2)).strip("[]")+str(GetTrans(T0_RH2)).strip("[]")
        
        self.crankid.SetDOFValues([crank_rot],[self.crankjointind])

        # Get the goal inverse kinematics
        
        self.goalik = self.probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 7 '+arg1+' maniptm 5 '+arg2)

        # print"Got self.goalik:"
        if(not self.is_valid(self.goalik,"goalik")):
            # print"Hey: No IK Solution found! Move the robot!"
            # self.notPlanning = True
            return []

        self.crankid.SetDOFValues([crank_rot],[self.crankjointind])
        
        #########################################################
        # This is the end of the second block in Matlab version #
        #########################################################
        self.crankid.SetDOFValues([self.crankjointind])
        #self.notPlanning = True
        return 1

    def compareConfigs(self,q0,q1):
        for j in range(len(q0)):
            diff= abs(q0[j]-q1[j])
            if(diff >0.000001):
                print "Joint "+str(j)+" is not the same: "+str(diff)

    def plan_and_save_trajectory(self,goalconfig,fname,useTsrChainString=False,setPsample=False,TsrChainString=''):
        #self.notPlanning = False
        try:
            if(self.which_planner == "birrt"):
                planner = RaveCreatePlanner(self.env,'birrt')
                params = Planner.PlannerParameters()
                params.SetRobotActiveJoints(self.robot)
                params.SetGoalConfig(goalconfig)
                planner.InitPlan(self.robot,params)
                outputtraj = RaveCreateTrajectory(self.env,'')
                answer=planner.PlanPath(outputtraj)
                # print"birrt planner:"
                # printanswer
                self.robot.GetController().SetPath(outputtraj)
                self.robot.WaitForController(0)
                # print"outputtraj serialize"
                trajfile = open(fname,'w')
                trajfile.write(outputtraj.serialize(0))
                trajfile.close()

            if(self.which_planner == "cbirrt"):    
                if(setPsample):
                    command_str = 'RunCBiRRT psample 0.2 '
                else:
                    command_str = 'RunCBiRRT '

                #print "Robot config at the beginning of: "+fname
#                q11=self.robot.GetConfigurationValues()
                if(useTsrChainString):     
                    answer = self.probs[0].SendCommand(command_str+'jointgoals %d %s %s'%(len(goalconfig),Serialize1DMatrix(matrix(goalconfig)),TsrChainString))
                else:
                    answer = self.probs[0].SendCommand(command_str+'jointgoals %d %s'%(len(goalconfig),Serialize1DMatrix(matrix(goalconfig))))
                #print "Robot config at the end of: "+fname                
               

                # print"runcbirrt answer"
                # printanswer
                if (answer == '0'):
                    return 0

                # Rename the file so that we can keep the data w/o overwriting it with a new trajectory
                try:
                    os.rename("cmovetraj.txt",fname)
                    # print"Executing "+fname
                    try:
                        if(self.mode == "openrave_only"):
                            #
                            answer = self.probs[0].SendCommand('traj '+fname);
                            self.robot.WaitForController(0)
                            time.sleep(0.2)
#                            q12=self.robot.GetConfigurationValues()
                            self.compareConfigs(q11,q12)
                            #                     
                        if(self.mode == "openrave_with_ros"):
                            pass
                        print"SendCommand "+fname+" answer: "+str(answer)
                    except openrave_exception, e:
                        print e
                except OSError, e:
                    print e
        except openrave_exception, e:
            print "Cannot send command"
            print e
        return 1

    def arm_planner(self):
        #print "Arm planner is called"
        self.notPlanning = False
        RaveSetDebugLevel(3)
        # Clean the trajectory files so that we don't execute an old trajectory
        for i in range(10):
            try:
                #print "Removing movetraj"+str(i)+".txt"
                os.remove("movetraj"+str(i)+".txt")
            except OSError, e:
                #print e
                pass   

        #######################################
        ## CREATE TRAJ 0:  INIT-->APPROACH 1 ##
        #######################################
        
        self.robot.SetActiveDOFValues(self.initconfig)

        
        #############################################
        ## CREATE TRAJ 2: APPROACH 2 --> STARTIK   ##
        #############################################

        goaljoints = str2num(self.startik)
        

        if(not self.plan_and_save_trajectory(goaljoints,'movetraj2.txt',False,False)):
            return 0

        #######################################
        ## CREATE TRAJ 3: STARTIK --> GOALIK ##
        #######################################

        goaljoints = str2num(self.goalik)
        goaljoints = concatenate((goaljoints,array([0])),axis=1)

        T0_w0L = MakeTransform(matrix(rodrigues([self.wristPitch,0,0])),transpose(matrix([0,0,0])))
        T0_w0R = MakeTransform(rodrigues([0,self.wristPitch,0]),matrix([[self.jointtm[9]],[self.jointtm[10]],[self.jointtm[11]]]))
        Tw0_eL = dot(linalg.inv(self.T0_crankcrank),self.T0_LH1_ENTRY)
        Tw0_eR = dot(linalg.inv(T0_w0R),self.T0_RH1_ENTRY)

        Bw0L = matrix([0,0,0,0,0,0,0,0,0,0,0,0])
        Bw0R = matrix([0,0,0,0,0,0,-pi,pi,0,0,0,0])   

        TSRString_G_R = SerializeTSR(5,'NULL',T0_w0R,Tw0_eR,Bw0R) # TSR for the right gripper
        TSRString_G_L = SerializeTSR(7,'crank crank',T0_w0L,Tw0_eL,Bw0L) # TSR for the left gripper

        TSRChainString_StoG = SerializeTSRChain(0,0,1,1,TSRString_G_R,'crank',matrix([self.crankjointind]))+' '+SerializeTSRChain(0,0,1,1,TSRString_G_L,'NULL',[])

        self.robot.SetActiveDOFValues(str2num(self.startik))


        if(not self.plan_and_save_trajectory(goaljoints,'movetraj3.txt',True,False,TSRChainString_StoG)):
            return 0

        ##########################################
        ## CREATE TRAJ 5: STARTIK --> APPROACH 2
        ##########################################

        #print "Press Enter to generate startik --> approachik_2 (trajectory5)"
        #sys.stdin.readline()
        goaljoints = self.initconfig

        self.robot.SetActiveDOFValues(str2num(self.startik))

        if(not self.plan_and_save_trajectory(goaljoints,'movetraj5.txt',False,False)):
            return 0

        return 1

if __name__ == "__main__":
    tp = TrajectoryPlanner()
