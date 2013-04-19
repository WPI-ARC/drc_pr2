#!/usr/bin/env python
#
# Bener Suay and Jim Mainprice, January 2013
#
# benersuay@wpi.edu
# jmainprice@wpi.edu
#

## OPENRAVE ##
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

from openravepy.misc import OpenRAVEGlobalArguments
from random import *

## SYSTEM - FILE OPS ##
import sys
import os
from datetime import datetime
import time

## Constraint Based Manipulation ##
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *

## DATA LOGGER ##
from bens_logger import *

class Sampler:

     #---------------------------------------------------------------
     # Loads the PR2 and wheel model
     #---------------------------------------------------------------
     def __init__(self,args=None):

        self.env = Environment()
        self.env.Load('../../models/driving_wheel.robot2.xml')
        self.robot = self.env.ReadRobotURI('robots/pr2-beta-static.zae')
        self.env.Add(self.robot)
        self.wheel = self.env.GetRobots()[0]
        self.wheelOrientation = [ 0.0, 0.0, 0.0]
        self.h = [0,0,0,0,0,0,0,0] # This will make it possible for us to display N transformation matrix drawings in qt viewer.

        self.robot.SetActiveManipulator('rightarm')
        self.ikmodel_right = databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterizationType.Transform6D) 

        if not self.ikmodel_right.load():
            self.ikmodel_right.autogenerate()
            print "auto generate ikmodel right"      

        self.robot.SetActiveManipulator('leftarm')
        self.ikmodel_left = databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterizationType.Transform6D) 
        self.robotPosition = [0,0,0]

        if not self.ikmodel_left.load():
            self.ikmode_left.autogenerate()
            print "auto generate ikmodel left"

        #T_wheel =  MakeTransform(matrix(xyz_rotation([pi/2,0,pi/2])),transpose(matrix([1,0,1])))
        self.T_wheel =  MakeTransform(matrix(xyz_rotation([pi/2,0,pi/2])),transpose(matrix([0,0,0])))
        #self.T_wheel = eye(4)
        self.wheel.SetTransform(array(self.T_wheel))
        
        
        # Open Grippers
        self.robot.SetDOFValues([0.5,0.5],[22,34])

        #self.env.SetViewer('qtcoin')
        #sys.stdin.readline()

     #---------------------------------------------------------------
     # Sample robot configurations around the wheel
     #---------------------------------------------------------------
     def SampleConfig(self):

         # ------------------------------------------
         # WHEEL

         #print "sampler config"
         #u1 = random.random()
         #u2 = random.random()
         #u3 = random.random()
         #quat = [sqrt(1-u1)*sin(2*pi*u2),sqrt(1-u2)*cos(2*pi*u2),sqrt(u1)*sin(2*pi*u3),sqrt(u1)*cos(2*pi*u3)]
         #return quat
         ra = 0
         # ra = random.uniform(-pi,pi) # Wheel axis (no need for sampling)
         rb = 0
         rb = random.uniform(-pi,pi)
         rc = 0
         rc = random.uniform(-pi,pi) 

         T_rot = MakeTransform(matrix(yaw_pitch_roll_rotation([ra,rb,rc])),transpose(matrix([0,0,0])))
         self.T_wheel =  dot(MakeTransform(matrix(xyz_rotation([pi/2,0,pi/2])),transpose(matrix([0,0,0]))),T_rot) 
         #self.T_wheel = eye(4)
         #self.T_wheel = MakeTransform(matrix(xyz_rotation([pi/2,0,pi/2])),transpose(matrix([0,0,0])))
         #self.T_wheel = MakeTransform(matrix(xyz_rotation([rx,ry,rz])),transpose(matrix([0,0,0])))

         self.wheelOrientation[0] = ra
         self.wheelOrientation[1] = rb
         self.wheelOrientation[2] = rc
         
         # ------------------------------------------
         # ROBOT
         x = -0.6
         y = 0
         z = 0

         # Sample Robot location
         # while True:
         #     x = 2*random.random()
         #     y = 4*random.random()-2
         #     z=0
         #     dist = linalg.norm([x,y,z])
         #     if( dist <= 1 and dist >=0.3):
         #         break

         x = random.uniform(-1,0) 
         y = random.uniform(-0.7,0.7)

         self.robotPosition = [x,y,z]

         # Sample the orientation
         self.robotOrientation = theta = 0

         M = eye(3)

         # Towards the wheel
         #X =  [x,y,z] / (-1*linalg.norm([x,y,z]))
         #M[0][0] = X[0]
         #M[1][0] = X[1]
         #M[2][0] = 0

         #M[0][1] = math.cos(math.atan2(X[1],X[0])+pi/2)
         #M[1][1] = math.sin(math.atan2(X[1],X[0])+pi/2)
         #M[2][1] = 0

         # Set last axis
         #M[0][2] = 0
         #M[1][2] = 0
         #M[2][2] = 1
         
         #print M

         return M

     #---------------------------------------------------------------
     # Check both self collision and collision with the environment
     # Returns True if there is a collision, False if all is clear
     #---------------------------------------------------------------
     def CheckCollision(self):
         return (self.env.CheckCollision(self.robot,self.wheel) or self.robot.CheckSelfCollision())
         
     #---------------------------------------------------------------
     # Test if the IK is feasible on the wheel
     #---------------------------------------------------------------
     def TestIK(self,q):

         self.wheel.SetTransform(array(self.T_wheel))
         #euler = axisAngleFromQuat(q)
         #T = MakeTransform(matrix(xyz_rotation(euler)),transpose(matrix([1,0,0])))
         #print self.robotPosition
         T_orientaiton = MakeTransform(q,transpose(matrix(self.robotPosition)))
         T_height = MakeTransform(eye(3),transpose(matrix([0,0,-0.8])))
         T = dot(T_orientaiton,T_height) 
         
         self.robot.SetTransform(array(T))
         self.h[0] = misc.DrawAxes(self.env,matrix(T),1)
         #self.h[1] = misc.DrawAxes(self.env,matrix(eye(4)),0.3)
         self.h[1] = misc.DrawAxes(self.env,self.T_wheel,0.3)

         manips = self.robot.GetManipulators()
         crankmanip = self.wheel.GetManipulators()      
          
         self.robot.SetActiveDOFs([15,16,17,18,19,20,21,27,28,29,30,31,32,33])
    
         initconfig = self.robot.GetActiveDOFValues()

         manip = self.robot.SetActiveManipulator('leftarm') # set the manipulator to leftarm
         T0_LH_INIT = manip.GetEndEffectorTransform() # end of manipulator 7

         manip = self.robot.SetActiveManipulator('rightarm') # set the manipulator to leftarm       
         T0_RH_INIT = manip.GetEndEffectorTransform() # end of manipulator 5
        
         self.robot.SetActiveDOFs([15,16,17,18,19,20,21,27,28,29,30,31,32,33])

         links = self.robot.GetLinks()

         crankjointind = 0

         maniptm = self.wheel.GetManipulators()[0].GetTransform()

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
         T0_LH1_ENTRY = dot(temp,MakeTransform(matrix(rodrigues([pi/4,0,0])),transpose(matrix([-0.01,0.01,-0.14]))))
         
         # Rotate the driving wheel's manipulator's coordinate frame 90 degrees around its Z-axis
         temp =  dot(maniptm,MakeTransform(matrix(rodrigues([0,0,pi/2])),transpose(matrix([0,0,0]))))
         
         # Rotate the new coordinate frame +90 degrees around its X-axis
         temp = dot(temp,MakeTransform(matrix(rodrigues([pi/2,0,0])),transpose(matrix([0,0,0]))))

         # Rotate the new coordinate frame 30 degrees around its Y-axis
         temp = dot(temp,MakeTransform(matrix(rodrigues([0,-pi/8,0])),transpose(matrix([0,0,0]))))

         # Translate the new coordinate frame -0.11 meters on its Z-axis and assign it to Left Hand
         T0_RH1_ENTRY = dot(temp,MakeTransform(matrix(rodrigues([-pi/4,0,0])),transpose(matrix([-0.01,-0.01,-0.12]))))
         
         T0_crankcrank = self.wheel.GetManipulators()[0].GetTransform()  

         T0_w0L = eye(4)
         T0_w0R = eye(4)
         
         crank_rot = pi/4
         TSRChainMimicDOF = 1

         Tcrank_rot = MakeTransform(matrix(rodrigues([crank_rot,0,0])),transpose(matrix([0,0,0]))) # For the right gripper
         Tcrank_rot2 = MakeTransform(matrix(rodrigues([0,0,crank_rot])),transpose(matrix([0,0,0]))); # For the left gripper

         # We will need the following lines for CBiRRT
         # Since we don't use CBiRRT, we rotate the right hand just like we rotated the left hand
         # T0_cranknew = dot(T0_w0R,Tcrank_rot)
         # temp = dot(linalg.inv(T0_w0R),T0_RH1_ENTRY)
         # T0_RH2 = dot(T0_cranknew,temp);

         temp = dot(linalg.inv(T0_crankcrank),T0_LH1_ENTRY)
         temp = dot(Tcrank_rot2,temp)
         T0_LH2 = dot(T0_crankcrank,temp) 

         temp = dot(linalg.inv(T0_crankcrank),T0_RH1_ENTRY)
         temp = dot(Tcrank_rot2,temp)
         T0_RH2 = dot(T0_crankcrank,temp) 

         ##############################################################
         ## TODO WHAT IS BEFORE ##
         ##############################################################

         self.robot.SetDOFValues( [0,0,0,0,0,0,0], [15,16,17,18,19,20,21] )
         self.robot.SetDOFValues( [0,0,0,0,0,0,0], [27,28,29,30,31,32,33] )

         #self.h[2] = misc.DrawAxes(self.env,T0_LH1_ENTRY,1)
         #self.h[3] = misc.DrawAxes(self.env,T0_RH1_ENTRY,1)

         #self.h[4] = misc.DrawAxes(self.env,T0_LH2,1)
         #self.h[5] = misc.DrawAxes(self.env,T0_RH2,1)
         
         sol0 = self.ikmodel_right.manip.FindIKSolution(array(T0_RH1_ENTRY),False) # StartIK right gripper
         sol1 = self.ikmodel_left.manip.FindIKSolution(array(T0_LH1_ENTRY),False) # StartIK left gripper
         sol2 = self.ikmodel_right.manip.FindIKSolution(array(T0_RH2),False) # GoalIK right gripper
         sol3 = self.ikmodel_left.manip.FindIKSolution(array(T0_LH2),False) # GoalIK left gripper

         if( (sol0 is not None) and (sol1 is not None ) and (sol2 is not None) and (sol3 is not None) ):
             self.robot.SetDOFValues( array(sol0), [27,28,29,30,31,32,33] )
             self.robot.SetDOFValues( array(sol1), [15,16,17,18,19,20,21] )
             #sys.stdin.readline()
             # Do collision check for StartIK
             if(self.CheckCollision()):
                 self.cause_of_failure = "collision at start ik"
                 return False # We failed.
             else:                 
                 # If no collision then try GoalIK
                 self.cause_of_failure = "start ik ok"
                 self.robot.SetDOFValues( array(sol2), [27,28,29,30,31,32,33] )
                 self.robot.SetDOFValues( array(sol3), [15,16,17,18,19,20,21] )
                 # sys.stdin.readline()
                 # Return True if there is no collision
                 return not self.CheckCollision()

         self.cause_of_failure = "no solution"
         return False

 #---------------------------------------------------------------
 # This class should handle the statistics
 #---------------------------------------------------------------  
class LearnIKSolution:
    
    def __init__(self,S=None,sample_number=0):
        self.sampler=S
        self.iterations=sample_number

        timestamp = str(datetime.now())
        filename = "reachability_data/" + timestamp[0:10] + '_' + timestamp[11:19]
        self.my_logger = BensLogger( "config_finder", filename )
        #self.my_logger.header(["x","y","z","a","result"])
        self.my_logger.header(["x","y","rb","rc","result"])

    def run(self):
       
        st = datetime.now()
        prev_percent = 0
        update = True
        print "0.0 percent done."
        for i in range(self.iterations):

            percent = int(100.0*(float(i)/self.iterations))

            if( percent>0 and percent!=prev_percent ):
                timestamp = str(datetime.now())
                print (str(percent) + " percent done.   " + timestamp[11:19] + " iteration is " + str(i))

            q = self.sampler.SampleConfig()
            r = self.sampler.TestIK(q)
            #data = [self.sampler.robotPosition[0], self.sampler.robotPosition[1], self.sampler.robotPosition[2], self.sampler.robotOrientation, int(r)]
            #data = [self.sampler.robotPosition[0], self.sampler.robotPosition[1], self.sampler.robotPosition[2], self.sampler.robotOrientation, int(r)]
            data = [self.sampler.robotPosition[0], self.sampler.robotPosition[1],
                    self.sampler.wheelOrientation[1], self.sampler.wheelOrientation[2],
                    int(r)]
            self.my_logger.save(data)
            prev_percent = percent
            #print str(i)+": "+str(r)+" "+self.sampler.cause_of_failure
            
        print "Iteration started at:"
        print st    
        print "Iteration ended at:"
        print datetime.now()
        print "Ran "+str(self.iterations)+" iterations."
   
 #---------------------------------------------------------------
 # Main test
 #---------------------------------------------------------------        
if __name__ == "__main__":
    smp = Sampler()
    lrn = LearnIKSolution(smp,10000)
    lrn.run()
    
