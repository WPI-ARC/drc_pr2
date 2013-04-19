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
RaveSetDebugLevel(1)
drchubo = env.ReadRobotURI('../../../openHubo/drchubo/drchubo-urdf/robots/drchubo.robot.xml')
env.Add(drchubo)
drchubo_manips  = drchubo.GetManipulators()
drchubo_rightFoot = drchubo_manips[3]

rf_eet = drchubo_rightFoot.GetEndEffectorTransform() # right foot end effector transform
shift_drchubo_to_its_right_foot = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.5,-1*rf_eet[1][3],-1*rf_eet[2][3]])))
drchubo.SetTransform(array(shift_drchubo_to_its_right_foot))

rlhuboplus = env.ReadRobotURI('../../../openHubo/huboplus/rlhuboplus.robot.xml')
shift_rlhuboplus_back = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-0.5,0.0,0.0])))
rlhuboplus.SetTransform(array(shift_rlhuboplus_back))
env.Add(rlhuboplus)

env.SetViewer('qtcoin')

print "Done! Press Enter to exit..."
sys.stdin.readline()
