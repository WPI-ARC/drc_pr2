#!/usr/bin/env python
#
# Bener Suay, December 2012
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

## ROS ##
import roslib; 
roslib.load_manifest('std_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('std_srvs')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('trajectory_msgs')

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
import actionlib
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *


## Constraint Based Manipulation ##
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *

## RAVE2ROS ##
from pr2_rave2ros_traj_play_from_file import *

class TrajectoryPlanner:
    def __init__(self,args=None):
        RaveInitialize()
        #RaveSetDebugLevel(5)
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
            
        #self.env.SetViewer('qtcoin')

        self.notPlanning = True # This will be set to False when we're using the trajectory planner
        
        rospy.init_node('openrave', anonymous=False)

        # This subscriber gets the wheel pose in torso_lift_link frame
        rospy.Subscriber("wheel_pose", PoseStamped, self.wheel_pose_callback)

        # This subscriber gets the PR2's joint values 
        rospy.Subscriber("joint_states", JointState, self.robot_joints_callback)

        # This initializes a service to test the trajectory points before planning
        rospy.Service('openrave_pr2_test_points', Empty, self.handle_test_points)        

        # This initializes a service to trigger the trajectory planner
        rospy.Service('openrave_pr2_arm_planner', Empty, self.handle_arm_planner)

        # This initializes a service to play the trajectories generated with the arm planner
        rospy.Service('openrave_pr2_play_trajectories', Empty, self.handle_play_trajectories)
        
        rospy.on_shutdown(self.myhook)

        rospy.spin()
        
    def myhook(self):
        print "shutdown time!"

    def wheel_pose_callback(self,wheelPose):
        #if(self.notPlanning == True): # We don't want the wheel to move during trajectory planning
        T0_tll = self.robot.GetLinks()[16].GetTransform() # Transform from the origin to torso_lift_link frame
        Ttll_wheel = MakeTransform(rotationMatrixFromQuat([wheelPose.pose.orientation.w,wheelPose.pose.orientation.x,wheelPose.pose.orientation.y,wheelPose.pose.orientation.z]),matrix([wheelPose.pose.position.x,wheelPose.pose.position.y,wheelPose.pose.position.z])) # Transform from torso_lift_link to racing wheel's frame

        # Wheel's model is defined rotated, the following transform will make it look straight up
        wheelStraightUp = MakeTransform(matrix(dot(rodrigues([0,0,-pi/2]),rodrigues([pi/2,0,0]))),transpose(matrix([0,0,0])))

        wheelAtCorrectLocation = dot(T0_tll,Ttll_wheel) # Wheel's pose in world coordinates, adjusted for torso lift link transform
        wheelAtCorrectLocation = dot(wheelAtCorrectLocation,wheelStraightUp) # Rotate the model so that it looks straight up
        self.crankid.SetTransform(array(wheelAtCorrectLocation))
        
    def robot_joints_callback(self,robotJoints):
        #if(self.notPlanning == True): # We don't want robot to move during trajectory planning...
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
        self.robot.SetDOFValues(jointValues, jointIndices)

    def SetRobotDOFValuesInSyncWithROS(self,robot,vals,inds):
        print vals
        print inds
        
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

    def handle_test_points(self,req):
        print "Testing points"
        self.notPlanning = False # We are planning: this will stop robot model and wheel model being updated            

        manips = self.robot.GetManipulators()
        crankmanip = self.crankid.GetManipulators()

        print "Getting Loaded Problems"
        self.probs = self.env.GetLoadedProblems()
        print self.probs[0] # --> self.probs_cbirrt
        print self.probs[1] # --> self.probs_crankmover


        self.robot.SetActiveDOFs([15,16,17,18,19,20,21,27,28,29,30,31,32,33])

        # InSyncWithROS sets DOF values for both OpenRAVE and Gazebo
        self.SetRobotDOFValuesInSyncWithROS(self.robot,[-0.85,0.85,-1.00,-1.57,-1.00,-1.00,0.548],[27,28,29,30,32,33,34])

        self.SetRobotDOFValuesInSyncWithROS(self.robot,[0.85, 0.85,1.00,-1.57,-1.00,1.00,0.548],[15,16,17,18,20,21,22])

        self.initconfig = self.robot.GetActiveDOFValues()

        print "initconfig"
        print self.initconfig
        print type(self.initconfig)

        manip = self.robot.SetActiveManipulator('leftarm') # set the manipulator to leftarm
        self.T0_LH_INIT = manip.GetEndEffectorTransform() # end of manipulator 7

        manip = self.robot.SetActiveManipulator('rightarm') # set the manipulator to leftarm       
        self.T0_RH_INIT = manip.GetEndEffectorTransform() # end of manipulator 5

        self.robot.SetActiveDOFs([15,16,17,18,19,20,21,27,28,29,30,31,32,33])

        print "Press enter to continue 1..."
        sys.stdin.readline()

        links = self.robot.GetLinks()

        self.crankjointind = 0

        self.jointtm = self.probs[0].SendCommand('GetJointTransform name crank jointind '+str(self.crankjointind))

        maniptm = self.crankid.GetManipulators()[0].GetTransform()

        self.jointtm = self.jointtm.replace(" ",",")

        self.jointtm = eval('['+self.jointtm+']')

        rhanddofs = [34] # Right gripper links here
        rhandclosevals = [0.35] # What are the closed joint values for the right hand's links?--> Obtained experimentally
        rhandopenvals = [0.548] # What are the open joint values for the right hand's links? --> Obtained experimentally
        lhanddofs = [22] # Left gripper links here
        lhandclosevals = [0.35] # Same as rhandclosevals for the left gripper --> Obtained experimentally
        lhandopenvals = [0.548] # Same as rhandopenvals for the left gripper --> Obtained experimentally

        ##############################################################
        ## THIS IS WHERE THE FIRST BLOCK ENDS IN THE MATLAB VERSION ##
        ##############################################################

        self.robot.SetActiveDOFs([15,16,17,18,19,20,21,27,28,29,30,31,32,33])

        # Rotate the driving wheel's manipulator's coordinate frame 90 degrees around its Z-axis
        temp =  dot(maniptm,MakeTransform(matrix(rodrigues([0,0,pi/2])),transpose(matrix([0,0,0]))))

        # Rotate the new coordinate frame -90 degrees around its X-axis
        temp = dot(temp,MakeTransform(matrix(rodrigues([-pi/2,0,0])),transpose(matrix([0,0,0]))))

        # Rotate the new coordinate frame -30 degrees around its Y-axis
        temp = dot(temp,MakeTransform(matrix(rodrigues([0,pi/2.4,0])),transpose(matrix([0,0,0]))))

        # Translate the new coordinate frame -0.11 meters on its Z-axis and assign it to Left Hand
        self.T0_LH1_APPROACH_1 = dot(temp,MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0,0,-0.31]))))
        self.T0_LH1_APPROACH_2 = dot(temp,MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0,0,-0.21]))))
        self.T0_LH1_ENTRY = dot(temp,MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0,0,-0.11]))))
        

        # Rotate the driving wheel's manipulator's coordinate frame 90 degrees around its Z-axis
        temp =  dot(maniptm,MakeTransform(matrix(rodrigues([0,0,pi/2])),transpose(matrix([0,0,0]))))

        # Rotate the new coordinate frame +90 degrees around its X-axis
        temp = dot(temp,MakeTransform(matrix(rodrigues([pi/2,0,0])),transpose(matrix([0,0,0]))))

        # Rotate the new coordinate frame 30 degrees around its Y-axis
        temp = dot(temp,MakeTransform(matrix(rodrigues([0,pi/2.4,0])),transpose(matrix([0,0,0]))))

        # Translate the new coordinate frame -0.11 meters on its Z-axis and assign it to Left Hand
        self.T0_RH1_APPROACH_1 = dot(temp,MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0,0,-0.31]))))
        self.T0_RH1_APPROACH_2 = dot(temp,MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0,0,-0.21]))))
        self.T0_RH1_ENTRY = dot(temp,MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0,0,-0.11]))))
        

        # debug: at this point, it can be useful to visualize where we want our hand to move (use visualization_msgs / markers?)

        arg1 = str(GetRot(self.T0_LH1_APPROACH_1)).strip("[]")+str(GetTrans(self.T0_LH1_APPROACH_1)).strip("[]")
        arg1 = arg1.replace("\n"," ")

        arg2 = str(GetRot(self.T0_RH1_APPROACH_1)).strip("[]")+str(GetTrans(self.T0_RH1_APPROACH_1)).strip("[]")
        arg2 = arg2.replace("\n"," ")        

        self.approachik_1 = self.probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 6 '+arg1+' maniptm 4 '+arg2)
        print "Got self.approachik_1:"

        if self.approachik_1 == '':
            print "Hey: No IK Solution found! Move the robot!"
            self.notPlanning = True
            return []
        else:
            print str2num(self.approachik_1)
            print type(str2num(self.approachik_1))

        print "------"

        self.robot.SetActiveDOFValues(str2num(self.approachik_1)) 
        self.SetRobotDOFValuesInSyncWithROS(self.robot,str2num(self.approachik_1),self.robot.GetActiveDOFIndices())
        print "went to approach-1"
        sys.stdin.readline()

        ################################################################
        ## THIS IS WHERE THE SECOND BLOCK ENDS IN THE MATLAB VERSION  ##
        ## APPROACH-1 IK IS OVER - NOW GO TO APPROACH-2               ##
        ################################################################
        
        arg1 = str(GetRot(self.T0_LH1_APPROACH_2)).strip("[]")+str(GetTrans(self.T0_LH1_APPROACH_2)).strip("[]")
        arg1 = arg1.replace("\n"," ")

        arg2 = str(GetRot(self.T0_RH1_APPROACH_2)).strip("[]")+str(GetTrans(self.T0_RH1_APPROACH_2)).strip("[]")
        arg2 = arg2.replace("\n"," ")        

        self.approachik_2 = self.probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 6 '+arg1+' maniptm 4 '+arg2)
        print "Got approachik-2:"

        if self.approachik_2 == '':
            print "Hey: No IK Solution found! Move the robot!"
            self.notPlanning = True
            return []
        else:
            print self.approachik_2

        print "------"

        self.robot.SetActiveDOFValues(str2num(self.approachik_2)) 
        self.SetRobotDOFValuesInSyncWithROS(self.robot,str2num(self.approachik_2),self.robot.GetActiveDOFIndices())
        print "went to approach-2"
        sys.stdin.readline()

        ####################################################################
        ## APPROACH-2 IK IS OVER - NOW GO TO STARTIK WHERE WE WILL GRASP  ##
        ####################################################################

        
        arg1 = str(GetRot(self.T0_LH1_ENTRY)).strip("[]")+str(GetTrans(self.T0_LH1_ENTRY)).strip("[]")
        arg1 = arg1.replace("\n"," ")

        arg2 = str(GetRot(self.T0_RH1_ENTRY)).strip("[]")+str(GetTrans(self.T0_RH1_ENTRY)).strip("[]")
        arg2 = arg2.replace("\n"," ")        

        self.startik = self.probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 6 '+arg1+' maniptm 4 '+arg2)
        print "Got self.startik:"

        if self.startik == '':
            print "Hey: No IK Solution found! Move the robot!"
            self.notPlanning = True
            return []
        else:
            print self.startik

        print "------"

        self.robot.SetActiveDOFValues(str2num(self.startik))
        self.SetRobotDOFValuesInSyncWithROS(self.robot,str2num(self.startik),self.robot.GetActiveDOFIndices())
        print "went to startik"
        sys.stdin.readline()

        ###################################################
        ## COLLISION FREE REACHING TO GRASP WITH CBiRRT  ##
        ###################################################


        ## Now let's try a different version of the previous block.
        ## The previous block tries to go to initial configuration without any collision avoidance.
        ## IF we want to avoid colliding with the wheel, we need to define Task Space Regions:

        # TSRString1 = SerializeTSR(5,'NULL',T0_RH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0])) # RightHand_Torso manip: 4      
        # TSRString2 = SerializeTSR(7,'NULL',T0_LH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0])) # LeftHand_Torso manip: 6
        # #TSRChainStringGoToStartIK = SerializeTSRChain(0,1,0,1,TSRString1,'NULL',[])+' '+SerializeTSRChain(0,1,0,1,TSRString2,'NULL',[])
        
        # TSRChainStringGoToStartIK = SerializeTSRChain(0,1,0,1,TSRString2,'NULL',[])
        # #answer = probs[0].SendCommand('RunCBiRRT psample 0.2 smoothingitrs 150 %s'%(TSRChainStringGoToStartIK))
        # answer = probs[0].SendCommand('RunCBiRRT psample 0.25 %s'%(TSRChainStringGoToStartIK))
        # # Defined the TSRs as constraints during the trajectory execution. Now let's send them over to the planner.
        # print answer
        # if (answer == 1):
        #     print "runcbirrt successful for finding a path to start ik without collision with the wheel"
        #     startik = self.robot.GetDOFValues()
        #     print "Got startik:"
        #     print startik
        #     print "------"
        #     # Rename the file so that we can keep the data w/o overwriting it with a new trajectory
        #     try:
        #         os.rename("cmovetraj.txt","movetraj-goto-startik.txt")

        #         if os.path.isfile('movetraj-goto-startik.txt'):
        #             try:
        #                 answer = pr2_rave2ros_traj_play_from_file('movetraj-goto-startikt.txt')
        #             except openrave_exception, e:
        #                 print e

        #                 self.robot.WaitForController(0)
        #                 print "Press enter to restart"
        #                 sys.stdin.readline()
        #         else:
        #             print "Can't find movetraj-goto-startik.txt"            
        #     except OSError, e:
        #         print e

        #     print "went to startik"
        #     sys.stdin.readline()

        # else:
        #     print "Hey: No IK Solution found! Move the robot!"
        #     print T0_LH1
        #     print T0_RH1
        #     self.notPlanning = True
        #     return []            
        
        ##############################################################
        ## END OF STARTIK TSR DEFINITION AND TRAJECTORY EXEC. BLOCK ##
        ##############################################################

        # Left hand is evaluated first, so need to make right hand relative to crank
        cranklinks = self.crankid.GetLinks();

        self.T0_crankcrank = self.crankid.GetManipulators()[0].GetTransform()

        T0_w0L = eye(4)
        T0_w0R = MakeTransform(rodrigues([0,0.4,0]),matrix([[self.jointtm[9]],[self.jointtm[10]],[self.jointtm[11]]]))

        crank_rot = pi/12
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
        self.goalik = self.probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 6 '+arg1+' maniptm 4 '+arg2)

        print "Got self.goalik:"
        if self.startik == '':
            print "Hey: No IK Solution found! Change the target!"
            self.notPlanning = True
            return []
        else:
            print self.goalik

        print "------"

        #self.robot.SetActiveDOFValues(str2num(goalik)) # Note: self.goalik is T0_LH2 and T0_RH2
        self.SetRobotDOFValuesInSyncWithROS(self.robot,str2num(self.goalik),self.robot.GetActiveDOFIndices())
        print "went to goalik"
        sys.stdin.readline()
        # Let the TSR Magic Happen

        self.crankid.SetDOFValues([crank_rot],[self.crankjointind])

        #########################################################
        # This is the end of the second block in Matlab version #
        #########################################################
        self.crankid.SetDOFValues([self.crankjointind])

        #self.robot.SetActiveDOFValues(str2num(startik))
        self.SetRobotDOFValuesInSyncWithROS(self.robot,str2num(self.startik),self.robot.GetActiveDOFIndices())

        ################################################
        ## YOU'RE AT STARTIK - GO BACK TO APPROACH 2  ##
        ################################################

        goaljoints = str2num(self.approachik_2);
        print "Press enter to go to approach point 2"
        sys.stdin.readline()
        self.SetRobotDOFValuesInSyncWithROS(self.robot,goaljoints,self.robot.GetActiveDOFIndices())
                
        ##################################################
        ## YOU'RE AT APPROACH 2 - GO BACK TO APPROACH 1 ##
        ##################################################

        goaljoints = str2num(self.approachik_1);
        print "Press enter to go to approach point 1"
        sys.stdin.readline()
        self.SetRobotDOFValuesInSyncWithROS(self.robot,goaljoints,self.robot.GetActiveDOFIndices())

        ######################################################
        ## YOU'RE AT APPROACH 1 - GO BACK TO INITCONFIG ##
        ######################################################

        print "Press enter to go to initconfig"
        sys.stdin.readline()
        self.SetRobotDOFValuesInSyncWithROS(self.robot,self.initconfig,self.robot.GetActiveDOFIndices())

        self.notPlanning = True
        emptyResponse = []
        return emptyResponse


    
    def handle_play_trajectories(self,req):
        print "Playing trajectories"
        #######################################################
        ## WE'RE DONE GENERATING TRAJECTORIES, NOW PLAY THEM ##
        #######################################################

        pr2_rave2ros_traj_play_from_file('movetraj0.txt') # initconfig --> approach 1
        pr2_rave2ros_traj_play_from_file('movetraj1.txt') # approach 1 --> approach 2
        pr2_rave2ros_traj_play_from_file('movetraj2.txt') # approach 2 --> startik

        numreps = 4
        for r in range(numreps):
            pr2MoveGrippers('close') # Close grippers in ROS so that we can interact with the wheel in Gazebo
            pr2_rave2ros_traj_play_from_file('movetraj3.txt') # startik --> goalik
            pr2MoveGrippers('open') # Open grippers in ROS so that we can interact with the wheel in Gazebo
            pr2_rave2ros_traj_play_from_file('movetraj4.txt') # goalik --> startik

        pr2_rave2ros_traj_play_from_file('movetraj5.txt') # startik --> approach 2
        pr2_rave2ros_traj_play_from_file('movetraj6.txt') # approach 2 --> approach 1
        pr2_rave2ros_traj_play_from_file('movetraj7.txt') # approach 1 --> initconfig

        ##########################################
        ## DONE PLAYING THE TRAJECTORIES - EXIT ##
        ##########################################

        print "press enter to exit"
        sys.stdin.readline()

        emptyResponse = []
        return emptyResponse
        
    def handle_arm_planner(self,req):
        self.notPlanning = True
        print "Arm planner is called"

        # Clean the trajectory files so that we don't execute an old trajectory
        for i in range(10):
            try:
                print "Removing movetraj"+str(i)+".txt"
                os.remove("movetraj"+str(i)+".txt")
            except OSError, e:
                print e   

        #######################################
        ## CREATE TRAJ 0:  INIT-->APPROACH 1 ##
        #######################################

        print "Press enter to generate init --> approach-1 (trajectory0)"
        sys.stdin.readline()
        goaljoints = str2num(self.approachik_1);
        TSRString_A1_R = SerializeTSR(5,'NULL',self.T0_RH1_APPROACH_1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRString_A1_L = SerializeTSR(7,'NULL',self.T0_LH1_APPROACH_1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRChainString_A1 = SerializeTSRChain(0,0,1,1,TSRString_A1_R,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString_A1_L,'NULL',[])
        
        try:
            answer = self.probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_A1))
            print "runcbirrt successful"
            print answer
            if (answer == '0'):
                return []
        except openrave_exception, e:
            print "Cannot send command runcbirrt"
            print e

        # Rename the file so that we can keep the data w/o overwriting it with a new trajectory
        try:
            os.rename("cmovetraj.txt","movetraj0.txt")
            print "Executing trajectory 0"
            try:
                answer = pr2_rave2ros_traj_play_from_file('movetraj0.txt')
                print "traj call successful"
                print answer 
            except openrave_exception, e:
                print e
        except OSError, e:
            print e

        #############################################
        ## CREATE TRAJ 1: APPROACH 1 -->APPROACH 2 ##
        #############################################

        print "Press enter to generate approach-1 --> approach-2 (trajectory1)"
        sys.stdin.readline()
        goaljoints = str2num(self.approachik_2)
        TSRString_A2_R = SerializeTSR(5,'NULL',self.T0_RH1_APPROACH_2,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRString_A2_L = SerializeTSR(7,'NULL',self.T0_LH1_APPROACH_2,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRChainString_A2 = SerializeTSRChain(0,0,1,1,TSRString_A2_R,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString_A2_L,'NULL',[])

        try:
            answer = self.probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_A2))
            print "runcbirrrt successful"
            print answer
            if (answer == '0'):
                return []
        except openrave_exception, e:
            print "Cannot send command runcbirrt"
            print e

        try:
            os.rename("cmovetraj.txt","movetraj1.txt")
            print "Executing trajectory 1"
            try:
                answer = pr2_rave2ros_traj_play_from_file('movetraj1.txt')
                print "traj call successful"
                print answer
            except openrave_exception, e:
                print e
        except OSError, e:
            print e

        #############################################
        ## CREATE TRAJ 2: APPROACH 2 --> STARTIK   ##
        #############################################

        print "Press enter to generate approach-2 --> startik (trajectory2)"
        sys.stdin.readline()
        goaljoints = str2num(self.startik)
        TSRString_S_R = SerializeTSR(5,'NULL',self.T0_RH1_ENTRY,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRString_S_L = SerializeTSR(7,'NULL',self.T0_LH1_ENTRY,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRChainString_A2toS = SerializeTSRChain(0,0,1,1,TSRString_S_R,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString_S_L,'NULL',[])
        
        try:
            answer = self.probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_A2toS))
            print "runcbirrt successful"
            print answer
            if (answer == '0'):
                return []
        except openrave_exception, e:
            print "Cannot send command runcbirrt"
            print e
            
        try:
            os.rename("cmovetraj.txt","movetraj2.txt")
            print "Executing trajectory 2"
            try:
                answer = pr2_rave2ros_traj_play_from_file('movetraj2.txt')
            except openrave_exception, e:
                print e
        except OSError, e:
            print e

        #######################################
        ## CREATE TRAJ 3: STARTIK --> GOALIK ##
        #######################################

        print "Press enter to generate startik --> goalik (trajectory3)"
        sys.stdin.readline()
        goaljoints = str2num(self.goalik)
        goaljoints = concatenate((goaljoints,array([0])),axis=1)


        T0_w0L = MakeTransform(matrix(rodrigues([0.4,0,0])),transpose(matrix([0,0,0])))
        T0_w0R = MakeTransform(rodrigues([0,0.4,0]),matrix([[self.jointtm[9]],[self.jointtm[10]],[self.jointtm[11]]]))
        Tw0_eL = dot(linalg.inv(self.T0_crankcrank),self.T0_LH1_ENTRY)
        Tw0_eR = dot(linalg.inv(T0_w0R),self.T0_RH1_ENTRY)

        Bw0L = matrix([0,0,0,0,0,0,0,0,0,0,0,0])
        Bw0R = matrix([0,0,0,0,0,0,-pi,pi,0,0,0,0])    

        TSRString_G_R = SerializeTSR(5,'NULL',T0_w0R,Tw0_eR,Bw0R) # TSR for the right gripper
        TSRString_G_L = SerializeTSR(7,'crank crank',T0_w0L,Tw0_eL,Bw0L) # TSR for the left gripper

        TSRChainString_StoG = SerializeTSRChain(0,0,1,1,TSRString_G_R,'crank',matrix([self.crankjointind]))+' '+SerializeTSRChain(0,0,1,1,TSRString_G_L,'NULL',[])

        try:
            answer = self.probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_StoG))
            print "runcbirrt succesful"
            print answer
            if (answer == '0'):
                return []
        except openrave_exception, e:
            print e
        
        try:
            os.rename("cmovetraj.txt","movetraj3.txt")
            print "Executing trajectory 3"
            try:
                answer = pr2_rave2ros_traj_play_from_file('movetraj3.txt')
            except openrave_exception, e:
                print e
        except OSError, e:
            print e

        ########################################
        ## CREATE TRAJ 4: GOALIK --> STARTIK  ##
        ########################################

        print "Press enter to generate goalik --> startik (trajectory4)"
        sys.stdin.readline()
        goaljoints = str2num(self.startik)
        goaljoints = concatenate((goaljoints,array([0])),axis=1)
        
        # Task Space Region definition is the same as startik --> goalik
        TSRChainString_GtoS = TSRChainString_StoG
        try:
            answer = self.probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_GtoS))
            print "runcbirrt succesful"
            print answer
            if (answer == '0'):
                return []
        except openrave_exception, e:
            print e

        try:
            os.rename("cmovetraj.txt","movetraj4.txt")
            print "Exceuting trajectory 4"
            try:
                answer = pr2_rave2ros_traj_play_from_file('movetraj4.txt')
            except openrave_exception, e:
                print e
        except OSError, e:
            print e       
            
        ##########################################
        ## CREATE TRAJ 5: STARTIK --> APPROACH 2
        ##########################################

        print "Press Enter to generate startik --> approachik_2 (trajectory5)"
        sys.stdin.readline()
        goaljoints = str2num(self.approachik_2)
        
        # For trajectory1, we've already defined TSRChainString_A2, and we can use it again here:
        try:
            answer = self.probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_A2))
            print "runcbirrt successful"
            print answer
            if (answer == '0'):
                return []
        except openrave_exception, e:
            print "Cannot send command runcbirrt"
            print e

        try:
            os.rename("cmovetraj.txt","movetraj5.txt")
            print "Executing trajectory 5"
            try:
                answer = pr2_rave2ros_traj_play_from_file('movetraj5.txt')
            except openrave_exception, e:
                print e
        except OSError, e:
            print e

        ############################################
        ## CREATE TRAJ 6: APPROACH 2 --> APPROACH 1
        ############################################
        
        print "Press Enter to generate approachik_2 --> self.approachik_1 (trajectory6)"
        sys.stdin.readline()
        goaljoints = str2num(self.approachik_1)
        
        try:
            answer = self.probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_A1))
        except openrave_exception, e:
            print e
        
        try:
            os.rename("cmovetraj.txt","movetraj6.txt")
            print "Executing trajectory 6"
            try:
                answer = pr2_rave2ros_traj_play_from_file('movetraj6.txt')
                print answer
                if (answer == '0'):
                    return []
            except openrave_exception, e:
                print e
        except OSError, e:
            print e            

        ############################################
        ## CREATE TRAJ 7:_APPROACH 1 --> INITCONFIG
        ############################################
            
        print "Press Enter to generate approachik_1 --> initconfig (trajectory7)"
        sys.stdin.readline()
        goaljoints = self.initconfig
        TSRString_I_R = SerializeTSR(5,'NULL',self.T0_RH_INIT,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRString_I_L = SerializeTSR(7,'NULL',self.T0_LH_INIT,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRChainString_I = SerializeTSRChain(0,0,1,1,TSRString_I_R,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString_I_L,'NULL',[])

        try:
            answer = self.probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_I))
            print answer
            if (answer == '0'):
                return []
        except openrave_exception, e:
            print e

        try:
            os.rename("cmovetraj.txt","movetraj7.txt")
            try:
                answer = pr2_rave2ros_traj_play_from_file('movetraj7.txt')
            except openrave_exception, e:
                print e
        except OSError, e:
            print e     

        print "press enter to exit"
        sys.stdin.readline()

        self.notPlanning = True
        emptyResponse = []
        return emptyResponse        

if __name__ == "__main__":
    tp = TrajectoryPlanner()
