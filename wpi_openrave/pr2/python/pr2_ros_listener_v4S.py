#!/usr/bin/env python
#
# Bener Suay, December 2012
# benersuay@wpi.edu
#
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
import sys
import roslib; 
roslib.load_manifest('std_msgs')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('gui_user_pkg')
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *
from sensor_msgs.msg import JointState
from gui_user_pkg.srv import *

class TrajectoryPlanner:
    def __init__(self,args=None):
        self.env = Environment()
        self.env.Load('../../models/driving_wheel.robot2.xml')
        self.robot = self.env.ReadRobotURI('robots/pr2-beta-static.zae')
        self.jointDict = {'bl_caster_rotation_joint' : 0, 'bl_caster_l_wheel_joint' : 1, 'bl_caster_r_wheel_joint' : 2, 'br_caster_rotation_joint' : 3, 'br_caster_l_wheel_joint' : 4, 'br_caster_r_wheel_joint' : 5, 'fl_caster_rotation_joint' : 6, 'fl_caster_l_wheel_joint' : 7, 'fl_caster_r_wheel_joint' : 8, 'fr_caster_rotation_joint' : 9, 'fr_caster_l_wheel_joint' : 10, 'fr_caster_r_wheel_joint' : 11, 'torso_lift_joint' : 12, 'head_pan_joint' : 13, 'head_tilt_joint' : 14, 'l_shoulder_pan_joint' : 15, 'l_shoulder_lift_joint' : 16, 'l_upper_arm_roll_joint' : 17, 'l_elbow_flex_joint' : 18, 'l_forearm_roll_joint' : 19, 'l_wrist_flex_joint' : 20, 'l_wrist_roll_joint' : 21, 'l_gripper_l_finger_joint' : 22, 'l_gripper_motor_slider_joint' : 23, 'l_gripper_motor_screw_joint' : 24, 'l_gripper_joint' : 25, 'laser_tilt_mount_joint' : 26, 'r_shoulder_pan_joint' : 27, 'r_shoulder_lift_joint' : 28, 'r_upper_arm_roll_joint' : 29, 'r_elbow_flex_joint' : 30, 'r_forearm_roll_joint' : 31, 'r_wrist_flex_joint' : 32, 'r_wrist_roll_joint' : 33, 'r_gripper_l_finger_joint' : 34, 'r_gripper_motor_slider_joint' : 35, 'r_gripper_motor_screw_joint' : 36, 'r_gripper_joint' : 37, 'torso_lift_motor_screw_joint' : 38}

        self.crankid = self.env.GetRobots()[0]
        self.env.Add(self.robot)
        self.env.SetViewer('qtcoin')
        rospy.init_node('openrave_viewer', anonymous=False)
        # Start a service that will wait for the user to return the wheel pose
        rospy.Service("update_valve_pose", updateValvePos, self.handle_update_valve_pose)
        rospy.Subscriber("joint_states", JointState, self.robot_joints_callback)
        rospy.spin()

    def handle_update_valve_pose(self,req):        
        wheel = req.valve
        r = req.radius

        print "Valve radius is: "
        print r

        T0_tll = self.robot.GetLinks()[16].GetTransform() # Transform from the origin to torso_lift_link frame

        Ttll_wheel = MakeTransform(rotationMatrixFromQuat([wheel.pose.orientation.w,wheel.pose.orientation.x,wheel.pose.orientation.y,wheel.pose.orientation.z]),matrix([wheel.pose.position.x,wheel.pose.position.y,wheel.pose.position.z])) # Transform from torso_lift_link to racing wheel's frame

        # Wheel's model is defined rotated, the following transform will make it look straight up
        wheelStraightUp = MakeTransform(matrix(dot(rodrigues([0,0,-pi/2]),rodrigues([pi/2,0,0]))),transpose(matrix([0,0,0])))
                
        wheelAtCorrectLocation = dot(T0_tll,Ttll_wheel) # Wheel's pose in world coordinates, adjusted for torso lift link transform
        wheelAtCorrectLocation = dot(wheelAtCorrectLocation,wheelStraightUp) # Rotate the model so that it looks straight up

        self.crankid.SetTransform(array(wheelAtCorrectLocation))

        res = updateValvePosResponse()
        res.success_code = "yeeha"

        return res
        
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
        self.robot.SetDOFValues(jointValues, jointIndices)

if __name__ == "__main__":
    tp = TrajectoryPlanner()
