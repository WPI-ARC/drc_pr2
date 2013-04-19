#!/usr/bin/env python
#
# Bener Suay and Jim Mainprice, February 2013
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
from xyz_rotation import *
from TransformMatrix import *
from str2num import *
from TSR import *

## DATA LOGGER ##
from bens_logger import *

class Sampler:

     #---------------------------------------------------------------
     # Loads the HUBO and wheel model
     #---------------------------------------------------------------
     def __init__(self,args=None):
        self.env = Environment()
        self.env.Load('../../models/driving_wheel.robot2.xml')
        RaveSetDebugLevel(1)
        self.robot = self.env.ReadRobotURI('../../../openHubo/huboplus/rlhuboplus.robot.xml')
        self.env.Add(self.robot)
        self.wheel = self.env.GetRobots()[0]
        self.wheelOrientation = [ 0.0, 0.0, 0.0]
        self.h = [0,0,0,0,0,0,0,0] # This will make it possible for us to display N transformation matrix drawings in qt viewer.

        # Note: RaveCreateProblem will be deprecated and it uses RaveCrateModule in it anyways so we just use RaveCreateModule instead
        self.probs_cbirrt = RaveCreateModule(self.env,'CBiRRT')
        self.probs_crankmover = RaveCreateModule(self.env,'CBiRRT')

        #self.footlinknames = ' Body_RAR Body_LAR polyscale 0.7 0.5 0 polytrans -0.015 0 0 '
        self.footlinknames = ' Body_RAR Body_LAR '
        #self.cogtarg_offsets = [-0.05, 0.085, 0]
        self.cogtarg_offsets = [0, 0, 0]
        self.manips = self.robot.GetManipulators()
        self.links = self.robot.GetLinks()

        # Note: The argument you pass in must be the same as (the robot name) defined in the .zae or .xml file
        # Try - Catch is very useful to figure out what the problem is (if any) with cbirrt function calls
        try:
            self.env.AddModule(self.probs_cbirrt,'rlhuboplus') # this name that we pass in, written in quotes, has to match the name field in the xml.
            self.env.AddModule(self.probs_crankmover,'crank')
        except openrave_exception, e:
            print e

        print "Getting Loaded Problems"
        self.probs = self.env.GetLoadedProblems()

        # probs[0] is now probs_cbirrt
        # probs[1] is now probs_crankmover

        self.robotPosition = [0,0,0]

        self.T_wheel =  MakeTransform(matrix(xyz_rotation([pi/2,0,pi/2])),transpose(matrix([0,0,0])))

        self.wheel.SetTransform(array(self.T_wheel))
                
        # Open Grippers
        # [27:41] right hand
        # [42:56] left hand
        self.robot.SetDOFValues([0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08,0.08],[27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56])

        self.env.SetViewer('qtcoin')
        sys.stdin.readline()

     #---------------------------------------------------------------
     # Sample robot configurations around the wheel
     #---------------------------------------------------------------
     def SampleConfig(self):

         # WHEEL
         ra = 0
         rb = 0 # random.uniform(-pi,pi)
         rc = 0 # random.uniform(-pi,pi)

         T_rot = MakeTransform(matrix(yaw_pitch_roll_rotation([ra,rb,rc])),transpose(matrix([0,0,0])))
         self.T_wheel =  dot(MakeTransform(matrix(xyz_rotation([pi/2,0,pi/2])),transpose(matrix([0,0,0]))),T_rot) 

         self.wheelOrientation[0] = ra
         self.wheelOrientation[1] = rb
         self.wheelOrientation[2] = rc
         
         # ROBOT
         z = random.uniform(-1.1,-0.7)
         x = -0.3 #random.uniform(-0.7,0) 
         y = -0.1 #random.uniform(0,0)

         self.robotPosition = [x,y,z]

         # Sample the orientation
         self.robotOrientation = theta = 0

         M = eye(3)

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
         T_orientation = MakeTransform(q,transpose(matrix([0,0,0])))
         T_height = MakeTransform(eye(3),transpose(matrix(self.robotPosition)))
         T = dot(T_orientation,T_height) 
         
         self.robot.SetTransform(array(T))
         #self.h[0] = misc.DrawAxes(self.env,matrix(T),1)
         #self.h[1] = misc.DrawAxes(self.env,self.T_wheel,0.3)

         crankmanip = self.wheel.GetManipulators()      
         cranklink = self.wheel.GetLinks()
         CTlink = cranklink[0].GetTransform()
         CTgrasp = crankmanip[0].GetLocalToolTransform()
         CTee = dot(CTlink,CTgrasp)

         self.robot.SetActiveDOFs([14,16,18,20,22,24,13,15,17,19,21,23,2,4,6,8,10,12,1,3,5,7,9,11])
         self.robot.SetDOFValues([-0.95,-0.95],[19,20]) # elbows
         self.robot.SetDOFValues([1, 1],[41,56]) #thumbs
    
         initconfig = self.robot.GetDOFValues()

         T0_LH_INIT = self.manips[0].GetEndEffectorTransform() # end of manipulator 1, left arm
         T0_RH_INIT = self.manips[1].GetEndEffectorTransform() # end of manipulator 2, right arm

         links = self.robot.GetLinks()

         crankjointind = 0

         maniptm = self.wheel.GetManipulators()[0].GetTransform()

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

         #########################################################
         # Rotate the driving wheel's manipulator's coordinate frame 90 degrees around its Z-axis
         temp = CTee*MakeTransform(matrix(rodrigues([-pi/2,0,0])),transpose(matrix([0,0,0])))

         # Rotate the new coordinate frame +90 degrees around its X-axis
         temp = temp*MakeTransform(matrix(rodrigues([0,0,-pi/2])),transpose(matrix([0,0,0])))
    
         # Translate the new coordinate frame -0.11 meters on its Z-axis and assign it to Left Hand
         Left_Hand_Point_In_Wheel_Coordinate_Frame = temp*MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-0.075,0.15,0])))
             
         T0_LH1 = Left_Hand_Point_In_Wheel_Coordinate_Frame;
    
         # Rotate the driving wheel's manipulator's coordinate frame 90 degrees around its Z-axis
         temp = CTee*MakeTransform(matrix(rodrigues([-pi/2,0,0])),transpose(matrix([0,0,0])))

         # Rotate the new coordinate frame +90 degrees around its X-axis
         temp = temp*MakeTransform(matrix(rodrigues([0,0,-pi/2])),transpose(matrix([0,0,0])))
    
         # Translate the new coordinate frame -0.11 meters on its Z-axis and assign it to Left Hand
         Right_Hand_Point_In_Wheel_Coordinate_Frame = temp*MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-0.075,-0.15,0])))
    
         T0_RH1 = Right_Hand_Point_In_Wheel_Coordinate_Frame;
         #########################################################

         
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

         # Get left foot link's transform
         for l in range(len(self.links)):
             if(self.links[l].GetName() == 'leftFoot'):
                 self.Tlink = self.links[l].GetTransform()
                 
         self.Tgrasp =  self.manips[2].GetLocalToolTransform() 
         self.Tee_LF = dot(self.Tlink,self.Tgrasp)

         #self.robot.SetDOFValues( [0,0,0,0,0,0,0], [15,16,17,18,19,20,21] )
         #self.robot.SetDOFValues( [0,0,0,0,0,0,0], [27,28,29,30,31,32,33] )

         self.h[2] = misc.DrawAxes(self.env,T0_LH1,1)
         self.h[3] = misc.DrawAxes(self.env,T0_RH1,1)

         #self.h[4] = misc.DrawAxes(self.env,T0_LH2,1)
         #self.h[5] = misc.DrawAxes(self.env,T0_RH2,1)

         # probs[0] is equivalent to "probs.cbirrt" in the matlab version
         # Stringify and combine the rotation and translation matrix to obtain a transformation matrix for the left hand
         arg1 = str(GetRot(T0_LH1)).strip("[]")+str(GetTrans(T0_LH1)).strip("[]")
         arg1 = arg1.replace("\n"," ")
         # Stringify and combine the rotation and translation matrix to obtain a transformation matrix for the right hand
         arg2 = str(GetRot(T0_RH1)).strip("[]")+str(GetTrans(T0_RH1)).strip("[]")
         arg2 = arg2.replace("\n"," ")

         arg3 = str(GetRot(self.Tee_LF)).strip("[]")+str(GetTrans(self.Tee_LF)).strip("[]")
         arg3 = arg3.replace("\n"," ")

         arg4 = str(GetRot(T0_LH2)).strip("[]")+str(GetTrans(T0_LH2)).strip("[]")
         arg4 = arg1.replace("\n"," ")

         arg5 = str(GetRot(T0_RH2)).strip("[]")+str(GetTrans(T0_RH2)).strip("[]")
         arg5 = arg1.replace("\n"," ")   

         self.cogtarg = [self.cogtarg_offsets[0]+self.robotPosition[0], self.cogtarg_offsets[1]+self.robotPosition[1], self.cogtarg_offsets[2]+self.robotPosition[2]]

         self.footlinknames = ' Body_RAR Body_LAR polyscale 0.4 0.8 0 polytrans 0 -0.015'

         self.h[4] = self.env.plot3(points=array((self.cogtarg[0],self.cogtarg[1],self.cogtarg[2])),pointsize=25.0,colors=array(((0,0,1,0.2))))

         self.h[5] = self.env.plot3(points=array((self.robotPosition[0]-0.015,self.robotPosition[1],self.robotPosition[2])),pointsize=25.0,colors=array(((1,0,0,0.6))))

         startik = self.probs[0].SendCommand('DoGeneralIK exec supportlinks 2 '+self.footlinknames+' movecog '+str(self.cogtarg[0])+' '+str(self.cogtarg[1])+' '+str(self.cogtarg[2])+' nummanips 3 maniptm 0 '+arg1+' maniptm 1 '+arg2+' maniptm 2 '+arg3)
         
         if(startik != ""):
             print "startik is not an empty string"
             print startik
             sys.stdin.readline()
         
         #goalik = self.probs[0].SendCommand('DoGeneralIK exec supportlinks 2 '+self.footlinknames+' movecog '+str(self.cogtarg)+' nummanips 3 maniptm 0 '+arg4+' maniptm 1 '+arg5+' maniptm 2 '+arg3)

         
         # if (startik != "" and goalik != ""):
         #     print "solution found!"
         #sys.stdin.readline()


         # sol0 = self.ikmodel_right.manip.FindIKSolution(array(T0_RH1_ENTRY),False) # StartIK right gripper
         # sol1 = self.ikmodel_left.manip.FindIKSolution(array(T0_LH1_ENTRY),False) # StartIK left gripper
         # sol2 = self.ikmodel_right.manip.FindIKSolution(array(T0_RH2),False) # GoalIK right gripper
         # sol3 = self.ikmodel_left.manip.FindIKSolution(array(T0_LH2),False) # GoalIK left gripper

         # if( (sol0 is not None) and (sol1 is not None ) and (sol2 is not None) and (sol3 is not None) ):
         #     self.robot.SetDOFValues( array(sol0), [27,28,29,30,31,32,33] )
         #     self.robot.SetDOFValues( array(sol1), [15,16,17,18,19,20,21] )
         #     sys.stdin.readline()
         #     # Do collision check for StartIK
         #     if(self.CheckCollision()):
         #         self.cause_of_failure = "collision at start ik"
         #         return False # We failed.
         #     else:                 
         #         # If no collision then try GoalIK
         #         self.cause_of_failure = "start ik ok"
         #         self.robot.SetDOFValues( array(sol2), [27,28,29,30,31,32,33] )
         #         self.robot.SetDOFValues( array(sol3), [15,16,17,18,19,20,21] )
         #         sys.stdin.readline()
         #         # Return True if there is no collision
         #         return not self.CheckCollision()

         # self.cause_of_failure = "no solution"
         return False

 #---------------------------------------------------------------
 # This class should handle the statistics
 #---------------------------------------------------------------  
class LearnIKSolution:
    
    def __init__(self,S=None,sample_number=0):
        self.sampler=S
        self.iterations=sample_number

        timestamp = str(datetime.now())
        filename = "../reachability_data/" + timestamp[0:10] + '_' + timestamp[11:19]
        self.my_logger = BensLogger( "config_finder", filename )
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
            data = [self.sampler.robotPosition[0], self.sampler.robotPosition[1],
                    self.sampler.wheelOrientation[1], self.sampler.wheelOrientation[2],
                    int(r)]
            self.my_logger.save(data)
            prev_percent = percent
            
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
