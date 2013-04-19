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
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

## Constraint Based Manipulation ##
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *

class TrajectoryPlanner:
    def __init__(self,args=None):
        self.env = Environment()
        self.env.Load('../../models/driving_wheel.robot2.xml')
        self.robot = self.env.ReadRobotURI('robots/pr2-beta-static.zae')
        self.jointDict = {'bl_caster_rotation_joint' : 0, 'bl_caster_l_wheel_joint' : 1, 'bl_caster_r_wheel_joint' : 2, 'br_caster_rotation_joint' : 3, 'br_caster_l_wheel_joint' : 4, 'br_caster_r_wheel_joint' : 5, 'fl_caster_rotation_joint' : 6, 'fl_caster_l_wheel_joint' : 7, 'fl_caster_r_wheel_joint' : 8, 'fr_caster_rotation_joint' : 9, 'fr_caster_l_wheel_joint' : 10, 'fr_caster_r_wheel_joint' : 11, 'torso_lift_joint' : 12, 'head_pan_joint' : 13, 'head_tilt_joint' : 14, 'l_shoulder_pan_joint' : 15, 'l_shoulder_lift_joint' : 16, 'l_upper_arm_roll_joint' : 17, 'l_elbow_flex_joint' : 18, 'l_forearm_roll_joint' : 19, 'l_wrist_flex_joint' : 20, 'l_wrist_roll_joint' : 21, 'l_gripper_l_finger_joint' : 22, 'l_gripper_motor_slider_joint' : 23, 'l_gripper_motor_screw_joint' : 24, 'l_gripper_joint' : 25, 'laser_tilt_mount_joint' : 26, 'r_shoulder_pan_joint' : 27, 'r_shoulder_lift_joint' : 28, 'r_upper_arm_roll_joint' : 29, 'r_elbow_flex_joint' : 30, 'r_forearm_roll_joint' : 31, 'r_wrist_flex_joint' : 32, 'r_wrist_roll_joint' : 33, 'r_gripper_l_finger_joint' : 34, 'r_gripper_motor_slider_joint' : 35, 'r_gripper_motor_screw_joint' : 36, 'r_gripper_joint' : 37, 'torso_lift_motor_screw_joint' : 38}

        self.crankid = self.env.GetRobots()[0]
        self.env.Add(self.robot)
        self.env.SetViewer('qtcoin')
        
        self.notPlanning = True # This will be set to False when we're using the trajectory planner
        
        rospy.init_node('openrave', anonymous=False)

        # This subscriber gets the wheel pose in torso_lift_link frame
        rospy.Subscriber("wheel_pose", PoseStamped, self.wheel_pose_callback)

        # This subscriber gets the PR2's joint values 
        rospy.Subscriber("joint_states", JointState, self.robot_joints_callback)

        # This initializes a service to trigger the trajectory planner
        rospy.Service('openrave_pr2_arm_planner', Empty, self.handle_arm_planner)        

        rospy.on_shutdown(self.myhook)

        rospy.spin()
        
    def myhook(self):
        print "shutdown time!"

    def wheel_pose_callback(self,wheelPose):
        if(self.notPlanning == True): # We don't want the wheel to move during trajectory planning
            T0_tll = self.robot.GetLinks()[16].GetTransform() # Transform from the origin to torso_lift_link frame
            Ttll_wheel = MakeTransform(rotationMatrixFromQuat([wheelPose.pose.orientation.w,wheelPose.pose.orientation.x,wheelPose.pose.orientation.y,wheelPose.pose.orientation.z]),matrix([wheelPose.pose.position.x,wheelPose.pose.position.y,wheelPose.pose.position.z])) # Transform from torso_lift_link to racing wheel's frame
            
            wheelStraightUp = MakeTransform(matrix(dot(rodrigues([0,0,-pi/2]),rodrigues([pi/2,0,0]))),transpose(matrix([0,0,0])))

            wheelAtCorrectLocation = dot(T0_tll,Ttll_wheel)
            wheelAtCorrectLocation = dot(wheelAtCorrectLocation,wheelStraightUp)

            self.crankid.SetTransform(array(wheelAtCorrectLocation))
        
    def robot_joints_callback(self,robotJoints):
        if(self.notPlanning == True): # We don't want robot to move during trajectory planning...
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

    def handle_arm_planner(self,req):
        print "Arm planner is called"
        self.notPlanning = False # We are planning: this will stop robot model and wheel model being updated

        probs_cbirrt = RaveCreateModule(self.env,'CBiRRT')
        probs_crankmover = RaveCreateModule(self.env,'CBiRRT')

        manips = self.robot.GetManipulators()
        crankmanip = self.crankid.GetManipulators()

        try:
            self.env.AddModule(probs_cbirrt,'pr2')
            self.env.AddModule(probs_crankmover,'crank')
        except openrave_exception, e:
            print e

        print "Getting Loaded Problems"
        probs = self.env.GetLoadedProblems()
        print probs[0] # --> probs_cbirrt
        print probs[1] # --> probs_crankmover

        self.robot.SetActiveDOFs([12,15,16,17,18,19,20,21,27,28,29,30,31,32,33])

        self.robot.SetDOFValues([-0.85,-1.00,-1.57,-1.00,-1.00,0.548],[27,29,30,32,33,34])

        self.robot.SetDOFValues([0.85,1.00,-1.57,-1.00,1.00,0.548],[15,17,18,20,21,22])

        self.robot.SetActiveDOFs([12,15,16,17,18,19,20,21,22,27,28,29,30,31,32,33,34])

        initconfig = self.robot.GetActiveDOFValues()

        print "Press enter to continue 1..."
        # sys.stdin.readline()

        links = self.robot.GetLinks()

        crankjointind = 0

        jointtm = probs[0].SendCommand('GetJointTransform name crank jointind '+str(crankjointind))

        maniptm = self.crankid.GetManipulators()[0].GetTransform()

        jointtm = jointtm.replace(" ",",")

        jointtm = eval('['+jointtm+']')

        rhanddofs = [34] # Right gripper links here
        rhandclosevals = [0.35] # What are the closed joint values for the right hand's links?--> Obtained experimentally
        rhandopenvals = [0.548] # What are the open joint values for the right hand's links? --> Obtained experimentally
        lhanddofs = [22] # Left gripper links here
        lhandclosevals = [0.35] # Same as rhandclosevals for the left gripper --> Obtained experimentally
        lhandopenvals = [0.548] # Same as rhandopenvals for the left gripper --> Obtained experimentally

        ##############################################################
        ## THIS IS WHERE THE FIRST BLOCK ENDS IN THE MATLAB VERSION ##
        ##############################################################

        self.robot.SetActiveDOFs([12,15,16,17,18,19,20,21,22,27,28,29,30,31,32,33,34])

        # Rotate the driving wheel's manipulator's coordinate frame 90 degrees around its Z-axis
        temp =  dot(maniptm,MakeTransform(matrix(rodrigues([0,0,pi/2])),transpose(matrix([0,0,0]))))

        # Rotate the new coordinate frame -90 degrees around its X-axis
        temp = dot(temp,MakeTransform(matrix(rodrigues([-pi/2,0,0])),transpose(matrix([0,0,0]))))

        # Rotate the new coordinate frame -30 degrees around its Y-axis
        temp = dot(temp,MakeTransform(matrix(rodrigues([0,-pi/6,0])),transpose(matrix([0,0,0]))))

        # Translate the new coordinate frame -0.11 meters on its Z-axis and assign it to Left Hand
        Left_Hand_Point_In_Wheel_Coordinate_Frame = dot(temp,MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0,0,-0.11]))))

        T0_LH1 = Left_Hand_Point_In_Wheel_Coordinate_Frame

        # Rotate the driving wheel's manipulator's coordinate frame 90 degrees around its Z-axis
        temp =  dot(maniptm,MakeTransform(matrix(rodrigues([0,0,pi/2])),transpose(matrix([0,0,0]))))

        # Rotate the new coordinate frame +90 degrees around its X-axis
        temp = dot(temp,MakeTransform(matrix(rodrigues([pi/2,0,0])),transpose(matrix([0,0,0]))))
        
        # Rotate the new coordinate frame 30 degrees around its Y-axis
        temp = dot(temp,MakeTransform(matrix(rodrigues([0,-pi/2,0])),transpose(matrix([0,0,0]))))

        # Translate the new coordinate frame -0.11 meters on its Z-axis and assign it to Left Hand
        Right_Hand_Point_In_Wheel_Coordinate_Frame = dot(temp,MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0,0,-0.11]))))

        T0_RH1 = Right_Hand_Point_In_Wheel_Coordinate_Frame

        arg1 = str(GetRot(T0_LH1)).strip("[]")+str(GetTrans(T0_LH1)).strip("[]")
        arg1 = arg1.replace("\n"," ")

        arg2 = str(GetRot(T0_RH1)).strip("[]")+str(GetTrans(T0_RH1)).strip("[]")
        arg2 = arg2.replace("\n"," ")        
 
        startik = probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 6 '+arg1+' maniptm 4 '+arg2)
        print "Got startik:"
        print startik
        print "------"

        self.robot.SetActiveDOFValues(str2num(startik)) # Note: startik: T0_LH1 and T0_RH1
        print "go to startik"
        # sys.stdin.readline()

        # THIS IS THE END OF THE SECOND BLOCK IN MATLAB VERSION

        # Left hand is evaluated first, so need to make right hand relative to crank
        cranklinks = self.crankid.GetLinks();

        T0_crankcrank = self.crankid.GetManipulators()[0].GetTransform()

        T0_w0L = eye(4)
        T0_w0R = MakeTransform(rodrigues([0,0.4,0]),matrix([[jointtm[9]],[jointtm[10]],[jointtm[11]]]))

        crank_rot = pi/4
        TSRChainMimicDOF = 1

        Tcrank_rot = MakeTransform(matrix(rodrigues([crank_rot,0,0])),transpose(matrix([0,0,0]))) # For the right gripper
        Tcrank_rot2 = MakeTransform(matrix(rodrigues([0,0,crank_rot])),transpose(matrix([0,0,0]))); # For the left gripper

        T0_cranknew = dot(T0_w0R,Tcrank_rot)

        temp = dot(linalg.inv(T0_w0R),T0_RH1)
        T0_RH2 = dot(T0_cranknew,temp);

        temp = dot(linalg.inv(T0_crankcrank),T0_LH1)
        temp = dot(Tcrank_rot2,temp)
        T0_LH2 = dot(T0_crankcrank,temp)

        arg1 = str(GetRot(T0_LH2)).strip("[]")+str(GetTrans(T0_LH2)).strip("[]")
        arg2 = str(GetRot(T0_RH2)).strip("[]")+str(GetTrans(T0_RH2)).strip("[]")

        self.crankid.SetDOFValues([crank_rot],[crankjointind])

        # Get the goal inverse kinematics
        goalik = probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 6 '+arg1+' maniptm 4 '+arg2)
        print "Got goalik:"
        print goalik
        print "------"
        self.robot.SetActiveDOFValues(str2num(goalik)) # Note: goalik is T0_LH2 and T0_RH2
        print "go to goalik"
        # sys.stdin.readline()
        # Let the TSR Magic Happen

        self.crankid.SetDOFValues([crank_rot],[crankjointind])

        #########################################################
        # This is the end of the second block in Matlab version #
        #########################################################
        self.crankid.SetDOFValues([crankjointind])

        self.robot.SetActiveDOFValues(str2num(startik))

        T0_w0L = MakeTransform(matrix(rodrigues([0.4,0,0])),transpose(matrix([0,0,0])))

        T0_w0R = MakeTransform(rodrigues([0,0.4,0]),matrix([[jointtm[9]],[jointtm[10]],[jointtm[11]]]))

        Tw0_eL = dot(linalg.inv(T0_crankcrank),T0_LH1)

        Tw0_eR = dot(linalg.inv(T0_w0R),T0_RH1)

        Bw0L = matrix([0,0,0,0,0,0,0,0,0,0,0,0])
        Bw0R = matrix([0,0,0,0,0,0,-pi,pi,0,0,0,0])    

        print "Look at the axes"
        # sys.stdin.readline()

        TSRstring1 = SerializeTSR(4,'NULL',T0_w0R,Tw0_eR,Bw0R) # TSR for the right gripper
        TSRstring2 = SerializeTSR(7,'crank crank',T0_w0L,Tw0_eL,Bw0L) # TSR for the left gripper

        TSRChainString = SerializeTSRChain(0,0,1,1,TSRstring1,'crank',matrix([crankjointind]))+' '+SerializeTSRChain(0,0,1,1,TSRstring2,'NULL',[])

        print TSRChainString

        # Clean the trajectory files so that we don't execute an old trajectory
        for i in range(2):
            try:
                print "Removing movetraj"+str(i)+".txt"
                os.remove("movetraj"+str(i)+".txt")
            except OSError, e:
                print e    


        # Set the current goal to RH1 and LH1        
        goaljoints = str2num(startik);
        #print goaljoints
        print "Press enter to continue 2.1.."
        # sys.stdin.readline()

        self.robot.SetActiveDOFValues(initconfig)
        self.crankid.SetDOFValues([0],[crankjointind])

        #print goaljoints
        print "Press enter to continue 2.2..."
        # sys.stdin.readline()

        TSRString_G1 = SerializeTSR(4,'NULL',T0_RH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRString_G2 = SerializeTSR(7,'NULL',T0_LH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
        TSRChainString_G = SerializeTSRChain(0,0,1,1,TSRString_G1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString_G2,'NULL',[])

        try:
            answer = probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_G))
            print "runcbirrt successful"
            print answer
        except openrave_exception, e:
            print "Cannot send command runcbirrt"
            print e

        # Rename the file so that we can keep the data w/o overwriting it with a new trajectory
        try:
            os.rename("cmovetraj.txt","movetraj0.txt")

            print "Executing trajectory 0"
            try:
                answer= probs[0].SendCommand('traj movetraj0.txt');
                # debug
                print "traj call successful"
                print answer 
            except openrave_exception, e:
                print e

            self.robot.WaitForController(0)
            # sys.stdin.readline()
        except OSError, e:
            print e

        goaljoints = str2num(goalik)
        print goaljoints
        print "Press enter to continue 3.1.."
        # sys.stdin.readline()

        goaljoints = append(goaljoints,0)
        print goaljoints
        print "Press enter to continue 3.2..."
        # sys.stdin.readline()

        # Crate a trajectory (from LH1 RH1 configuration --> to LH2 RH2 configuration)
        try:
            answer = probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString))
            print "runcbirrt successful"
            print answer
        except openrave_exception, e:
            print "Cannot send command runcbirrt"
            print e

        # sys.stdin.readline()
        # Rename the file so that we can keep the data w/o overwriting it with a new trajectory
        try:
            os.rename("cmovetraj.txt","movetraj1.txt")
        except OSError, e:
            print e

        self.robot.SetActiveDOFValues(initconfig)

        play_n_times = 1
        n = 0
        while (n < play_n_times):
            
            print "Press enter to execute trajectory 0"
            # sys.stdin.readline()        

            try:
                answer1= probs[0].SendCommand('traj movetraj0.txt')
            except openrave_exception, e:
                print e

            self.robot.WaitForController(0)
            # sys.stdin.readline()

            print "Press enter to execute trajectory 1"
            # sys.stdin.readline()
            try:
                answer1= probs[0].SendCommand('traj movetraj1.txt')
                answer2= probs[1].SendCommand('traj movetraj1.txt')
            except openrave_exception, e:
                print e

            self.robot.WaitForController(0)
            print "Press enter to restart"
            # sys.stdin.readline()

            n = n+1

        print "press enter to exit"
        # sys.stdin.readline()

        self.notPlanning = True

        emptyResponse = []
        return emptyResponse
        

if __name__ == "__main__":
    tp = TrajectoryPlanner()
