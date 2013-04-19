#!/usr/bin/env python
#
# Bener Suay, April 2013
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

## Constraint Based Manipulation ##
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *

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
            newname = 'robot'+str(i)
            body.SetName(newname)
        print body


# drchubo_manips  = drchubo.GetManipulators()
# drchubo_rightFoot = drchubo_manips[3]

# eet1 = robot1.GetEndEffectorTransform() # right foot end effector transform

shift_robot0 = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))
shift_robot1 = MakeTransform(matrix(rodrigues([0,0,pi])),transpose(matrix([-1.0,0.0,0.0])))

robots[0].SetTransform(array(shift_robot0))
robots[1].SetTransform(array(shift_robot1))

mybox = RaveCreateKinBody(env,'')
mybox.SetName('box')
mybox.InitFromBoxes(numpy.array([[-0.5,0,0.3,0.025,0.025,0.3]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
env.Add(mybox,True)

h=misc.DrawAxes(env,mybox.GetTransform(),1.0)


print "Done! Press Enter to exit..."
sys.stdin.readline()

env.Destroy()
RaveDestroy()
