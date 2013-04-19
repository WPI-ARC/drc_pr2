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
robot = env.ReadRobotURI('../../../openHubo/huboplus/rlhuboplus.robot.xml')
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
ty = 0.1
tz = 0.0

executable_name = "./compute"

cmd = executable_name + ' ' + str(r00) + ' ' + str(r01) + ' ' + str(r02) + ' ' + str(tx) + ' ' + str(r10) + ' ' + str(r11) + ' ' + str(r12) + ' ' + str(ty) + ' ' + str(r20) + ' ' + str(r21) + ' ' + str(r22) + ' ' + str(tz)

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
            for j in range(6):
                # the following will strip the comma in the end of the joint value and convert it into a float
                q.append(float(words[w+1+j][0:len(words[w+1+j])-1]))
            # now we have a configuration for 6 joints for this solution
            # print q
            solutions_float.append(q)
    print solutions_float
else:
    print solutions_str
    #print 0

if(solutions_float != []):
    for s in range(len(solutions_float)):
        q = solutions_float[s]
        robot.SetDOFValues(q,[14,16,18,20,22,24])
        print "robot collision with other objects?"
        print env.CheckCollision(robot)
        print "robot self collision?"
        print robot.CheckSelfCollision()
        print "is the hand at the requested location?"
        leftPalm_x =  manips[0].GetEndEffectorTransform()[0][3]
        leftPalm_y =  manips[0].GetEndEffectorTransform()[1][3]
        leftPalm_z =  manips[0].GetEndEffectorTransform()[2][3]

        Body_Torso_x = manips[0].GetBase().GetTransform()[0][3]
        Body_Torso_y = manips[0].GetBase().GetTransform()[1][3]
        Body_Torso_z = manips[0].GetBase().GetTransform()[2][3]

        x_close_enough = allclose((leftPalm_x - Body_Torso_x),tx)
        y_close_enough = allclose((leftPalm_y - Body_Torso_y),ty)
        z_close_enough = allclose((leftPalm_z - Body_Torso_z),tz)

        if(x_close_enough and y_close_enough and z_close_enough):
            print "IK solution good enough"
        else:
            print "Bad solution"

        sys.stdin.readline()


print "Done!"
sys.stdin.readline()
