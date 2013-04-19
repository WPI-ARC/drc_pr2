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

import inspect


env = Environment()
RaveSetDebugLevel(5)
robot = env.ReadRobotURI('../../../openHubo/huboplus/rlhuboplus.robot.xml')
env.Add(robot)
manips  = robot.GetManipulators()

#env.SetViewer('qtcoin')
sys.stdin.readline()



ksh = []
iksolvernames = []
print "Kinematics Structure Hash for rlhubo's manipulators:"
for m in range(len(manips)):
    # the following method returns the long unique number that belongs
    # to a manipulator of a robot. The number is used when generating 
    # an IK solver or an inverse/reachability database.
    # If you ever generated an IK Solver or an inverse/reachability
    # database, if you look at your /home/your_user_name/.openrave
    # directory, you will see a kinematics.<structure_hash_number>
    # folder. That <stucture_hash_number> is the same number that this
    # method returns.
    ksh.append(manips[m].GetKinematicsStructureHash())
    iksolvernames.append('ikfast.%s.%s.%s'%(manips[m].GetKinematicsStructureHash(),str(IkParameterizationType.Transform6D),manips[m].GetName()))
    print iksolvernames[m]

ikfastproblem = RaveCreateModule(env,'ikfast')

print "here"
print ikfastproblem

try:
    env.Add(ikfastproblem) # this name that we pass in, written in quotes, has to match the name field in the xml.    
except openrave_exception, e:
    print e

print "Getting Loaded Problems"
probs = env.GetLoadedProblems()



#argument = 'AddIkLibrary %s %s'%(iksolvernames[0],"/home/bener/.openrave/kinematics.b3836b1eef7ad787b4063da651535802/ikfast61.Transform6D.14_16_18_20_22_24.so")
cmd = 'AddIkLibrary %s %s'%(iksolvernames[0],"/home/bener/.openrave/kinematics.b3836b1eef7ad787b4063da651535802/ikfast61.Transform6D.i686.14_16_18_20_22_24.so")
print cmd
iktype_id = ikfastproblem.SendCommand(cmd)

# print "iktype_id"
# print iktype_id

# print "Transform6D type"
# print str(IkParameterizationType.Transform6D)

#cmd = 'LoadIKFastSolver rlhuboplus Transform6D'

#ikfastproblem.SendCommand(cmd) # Returns nothing but does call the SetIKSolver for the robot

iksolver = RaveCreateIkSolver(env,"ikfast "+iksolvernames[0])
print "setting the iksolver"
print manips[0].SetIKSolver(iksolver)

print manips[0].GetTransform()

matrix=str(float64(manips[0].GetTransform()[0][0]))+" "+str(float64(manips[0].GetTransform()[0][1]))+" "+str(float64(manips[0].GetTransform()[0][2]))+" "+str(float64(manips[0].GetTransform()[1][0]))+" "+str(float64(manips[0].GetTransform()[1][1]))+" "+str(float64(manips[0].GetTransform()[1][2]))+" "+str(float64(manips[0].GetTransform()[2][0]))+" "+str(float64(manips[0].GetTransform()[2][1]))+" "+str(float64(manips[0].GetTransform()[2][2]))+" "+str(float64(manips[0].GetTransform()[0][3]))+" "+str(float64(manips[0].GetTransform()[1][3]))+" "+str(float64(manips[0].GetTransform()[2][3]))


#manips[0].SetTransform([[0.01006044022136415,-0.5673744299208516,-0.8233984720744512,0.09529901163540351],[-0.9232426339900602,-0.3215580283701716,0.2102937782574255,0.3210613213834305],[-0.3840857017979812,0.7580809261963237,-0.5270592784608537,-0.1292768460329552],[0,0,0,1]])

#matrix="0.01006044022136415 -0.5673744299208516 -0.8233984720744512 0.09529901163540351 -0.9232426339900602 -0.3215580283701716 0.2102937782574255 0.3210613213834305 -0.3840857017979812 0.7580809261963237 -0.5270592784608537 -0.1292768460329552"

print matrix

print "active manipulator:"
print robot.GetActiveManipulator()

cmd = "IKTest matrix "+matrix+" robot rlhuboplus nocol armjoints 14 16 18 20 22 24"
ikfastproblem.SendCommand(cmd)

#ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterizationType.Transform6D)

#print "ikmodel"
#print ikmodel


#print "iksolver"
#print iksolver



# This prints out the list of methods of the iksolver object
#for m in inspect.getmembers(iksolver, predicate=inspect.ismethod):
#    print m

#q0 = manips[0].GetEndEffectorTransform()
#print q0

#print "type of ikparameterization"
#print type(manips[0].GetIkParameterization(IkParameterizationType.Transform6D).GetValues())
#print manips[0].GetIkParameterization(IkParameterizationType.Transform6D)

#numpyvals = manips[0].GetIkParameterization(IkParameterizationType.Transform6D).GetValues()
#floatvals = []
#for v in range(len(numpyvals)):
#    floatvals.append(float64(numpyvals[v]))

#iksolver.Solve(manips[0].GetBase(),floatvals,q0,0)

#iksolver.solveFullIK_6D()
#manips[0].FindIKSolution()

#print manips[0].GetIkSolver()

cmd = "DebugIK robot rlhuboplus numtests 1"
ikfastproblem.SendCommand(cmd)

sys.stdin.readline()
