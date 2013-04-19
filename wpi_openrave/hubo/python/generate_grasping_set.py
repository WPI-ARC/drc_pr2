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

from openravepy.databases import inversereachability,grasping



env = Environment()
env.SetViewer('qtcoin')
env.Reset()
robot = env.ReadRobotXMLFile('../../../openHubo/huboplus/rlhuboplus.robot.xml')
env.AddRobot(robot)
robot.SetActiveManipulator('leftArm')
target = env.ReadKinBodyXMLFile('data/mug1.kinbody.xml')
env.AddKinBody(target)
target.SetTransform(array(mat([[1,0,0,.7],[0,1,0,0],[0,0,1,.673],[0,0,0,1]])))
gmodel = openravepy.databases.grasping.GraspingModel(robot,target)
if not gmodel.load():
    print "generating grasping set for target"
    gmodel.autogenerate()
