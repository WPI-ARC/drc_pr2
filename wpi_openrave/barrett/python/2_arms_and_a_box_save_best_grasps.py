#!/usr/bin/env python
#
# Bener Suay, April 2013
#
# benersuay@wpi.edu
#

# Uses object's grasp model
# Checks for IK Solutions


## OPENRAVE ##
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

from openravepy.misc import OpenRAVEGlobalArguments
from openravepy.misc import InitOpenRAVELogging
from openravepy.databases import inversereachability,grasping

## SYSTEM - MATH - FILE OPS ##
import sys
import os
from datetime import datetime
import time
import commands
from random import *
from itertools import izip

## Constraint Based Manipulation ##
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *

InitOpenRAVELogging() # This removes the error "No handlers cold be found for logger openravepy"
env = Environment()
env.SetViewer('qtcoin')

RaveSetDebugLevel(1)

robots = []

# the following for loop will add N robots from the same xml file
# and rename them to prevent failure
for i in range(2):
    robots.append(env.ReadRobotURI('robots/barrettwam.robot.xml'))
    env.Add(robots[len(robots)-1]) # add the last robot in the environment so that we can rename it.

    for body in env.GetBodies():
        rname = body.GetName()
        if(rname == 'BarrettWAM'):
            newname = 'BarrettWAM_'+str(i)
            body.SetName(newname)
        print body

shift_robot0 = MakeTransform(matrix(rodrigues([0,0,pi])),transpose(matrix([0.5,0.0,0.0])))
shift_robot1 = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-1.5,0.0,0.0])))

robots[0].SetTransform(array(shift_robot0))
robots[1].SetTransform(array(shift_robot1))

mybox = RaveCreateKinBody(env,'')
mybox.SetName('box')
mybox.InitFromBoxes(numpy.array([[-0.5,0,0.3,0.025,0.025,0.3]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
env.Add(mybox,True)

handles = []

handles.append(misc.DrawAxes(env,mybox.GetTransform(),1.0))

gmodels = []

gmodels.append(openravepy.databases.grasping.GraspingModel(robots[0],mybox))
if not gmodels[0].load():
    print "generating grasping set for robot_0 and the target"
    gmodels[0].autogenerate()
else:
    print "grasping set already exists for robot_0"


gmodels.append(openravepy.databases.grasping.GraspingModel(robots[1],mybox))
if not gmodels[1].load():
    print "generating grasping set for robot_1 and the target"
    gmodels[1].autogenerate()
else:
    print "grasping set already exists for robot_1"

def graspiter(index):
    for grasp,graspindex in izip(validgrasps,validindices):
        gmodel.showgrasp(grasp)
        yield gmodels[index].getGlobalGraspTransform(grasp,collisionfree=True),(gmodels[index],graspindex)

for i in range(2):
    validgrasps,validindices = gmodels[i].computeValidGrasps(checkik=True,backupdist=0.01)

    print 'validgrasps',len(validgrasps)
    print 'validindices',validindices
    
    for grasp in validgrasps:
        if(i == 0): # robot 0
            approach_dir = gmodels[i].getGlobalApproachDir(grasp).tolist() # returns a numpy ndarray of form [0. 1. 0.]
            print approach_dir
            if(approach_dir[0]==0 and approach_dir[0]==1 and approach_dir[0]==0):
                Tgrasp = gmodels[i].getGlobalGraspTransform(grasp,collisionfree=True).tolist()
                Grasp_x
                Grasp_z
                if(Tgrasp[2][3]<
        

        
        print "global approach direction for this grasp is:"
        print type(gmodels[i].getGlobalApproachDir(grasp).tolist())
        h = misc.DrawAxes(env,Tgrasp,0.3)
        print "press enter to see grasp"
        l = sys.stdin.readline()
        print l
        
        gmodels[i].showgrasp(grasp)

        print "press enter to run grasp"
        l = sys.stdin.readline()
        print l
        result = gmodels[i].runGrasp(grasp)
        print result

        print "press enter to run grasp from trans"
        l = sys.stdin.readline()
        print l
        result = gmodels[i].runGraspFromTrans(grasp)
        print result

        print "press enter to move to preshape"
        l = sys.stdin.readline()
        print l
        traj = gmodels[i].moveToPreshape(grasp)
        print traj
        
        print "press enter to get preshape joint values"
        l = sys.stdin.readline()
        print l
        psjv = gmodels[i].getPreshape(grasp)
        print psjv
        
        print "press enter to see the next grasp"
        l = sys.stdin.readline()
        print l
        

print "Done! Press Enter to exit..."
sys.stdin.readline()

env.Destroy()
RaveDestroy()
