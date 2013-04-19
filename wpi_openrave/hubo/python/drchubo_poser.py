#!/usr/bin/env python
#
# Bener Suay, March 2013
#
# benersuay@wpi.edu
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
import commands

env = Environment()
RaveSetDebugLevel(1)
robot = env.ReadRobotURI('../../../openHubo/drchubo/drchubo-urdf/robots/drchubo.robot.xml')
env.Add(robot)
manips  = robot.GetManipulators()

env.SetViewer('qtcoin')

r00 = 1.0
r01 = 0.0
r02 = 0.0

r10 = 0.0
r11 = 1.0
r12 = 0.0

r20 = 0.0
r21 = 0.0
r22 = 1.0

tx = 0.3
ty = 0.0
tz = 0.0

debug=True # if true, will print things out

# value of the joint that we set free for the ik solver
# the last number of the ik solver executable name indicates the
# joint index that we set free. For example if the executable name
# ends with _f5 it means the joint that has 5 as its index is set free.
# Then we manually define the joint value here with the following 
# variable for the ik solver.
free_joint_val = 0.0 
free_joint_index = 5

executable_name = "./drchubo_leftArm_ik_solver_f5"

cmd = executable_name + ' ' + str(r00) + ' ' + str(r01) + ' ' + str(r02) + ' ' + str(tx) + ' ' + str(r10) + ' ' + str(r11) + ' ' + str(r12) + ' ' + str(ty) + ' ' + str(r20) + ' ' + str(r21) + ' ' + str(r22) + ' ' + str(tz)

if(free_joint_index != None):
    cmd = cmd + ' ' + str(free_joint_val)

solutions_str = commands.getoutput(cmd)

solutions_float = []

if(solutions_str.find("Failed") != 0):
    words = solutions_str.split()
    #print words
    for w in range(len(words)): 
        if(words[w] == 'Found'):
            num_solutions = int(words[w+1])
            print "Found "+str(num_solutions)+" solutions:"
        elif(words[w] == '(free=0):'): # configuration comes after this word
            q = []
            for j in range(7):
                # the following will strip the comma in the end of the joint value and convert it into a float
                if(j == 4): # do a smarter comparison here. Get the joint name, and index, and see if it matches to the one that we actually set free in the ik solver.
                    q.append(0.0) # Set the free joint to zero
                else:
                    q.append(float(words[w+1+j][0:len(words[w+1+j])-1]))
            # now we have a configuration for 7 joints for this solution
            # print q
            solutions_float.append(q)
    print solutions_float
else:
    print solutions_str
    #print 0

if(solutions_float != []):
    for s in range(len(solutions_float)):
        q = solutions_float[s]
        robot.SetDOFValues(q,[1,2,3,4,5,6,7])
        
        leftPalm_x =  manips[0].GetEndEffectorTransform()[0][3]
        leftPalm_y =  manips[0].GetEndEffectorTransform()[1][3]
        leftPalm_z =  manips[0].GetEndEffectorTransform()[2][3]

        Body_Torso_x = manips[0].GetBase().GetTransform()[0][3]
        Body_Torso_y = manips[0].GetBase().GetTransform()[1][3]
        Body_Torso_z = manips[0].GetBase().GetTransform()[2][3]

        x_close_enough = allclose((leftPalm_x - Body_Torso_x),tx)
        y_close_enough = allclose((leftPalm_y - Body_Torso_y),ty)
        z_close_enough = allclose((leftPalm_z - Body_Torso_z),tz)

        
        if(debug):
            print "robot collision with other objects?"
            print env.CheckCollision(robot)
            print "robot self collision?"
            print robot.CheckSelfCollision()
            print "is the hand at the requested location?"
            print "left palm is at ",str(leftPalm_x - Body_Torso_x),', ',str(leftPalm_y - Body_Torso_y),', ',str(leftPalm_z - Body_Torso_z)
            print "requested position is ",str(tx),', ',str(ty),', ',str(tz)
            print "is x close enough? ",str(x_close_enough)
            print "is y close enough? ",str(y_close_enough)
            print "is z close enough? ",str(z_close_enough)

        if(x_close_enough and y_close_enough and z_close_enough):
            print "IK solution good enough"
        else:
            print "Bad solution"


        sys.stdin.readline()


print "Done!"
sys.stdin.readline()
