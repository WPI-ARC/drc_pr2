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

## ROBOT PLACEMENET ##
from Reachability import *

## MATH ##
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

offset = [0.35,0.1,0.0]


# 1. Create a trajectory for the tool center point to follow
Tstart = array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0+offset[0],0.0+offset[1],0.0]))))
Tgoal = array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.20+offset[0],0.0+offset[1],0.0]))))

traj = []
traj.append(Tstart)
traj.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.02+offset[0],0.0+offset[1],0.0])))))
# Move, then rotate around the axis with that shift being the radius
#traj.append(dot(array(MakeTransform(matrix(rodrigues([0,0,pi/2])),transpose(matrix([0.0,0.0,0.0])))),array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.07+offset[0],0.0+offset[1],0.0]))))))
traj.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.12+offset[0],0.0+offset[1],0.0])))))
traj.append(array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.16+offset[0],0.0+offset[1],0.0])))))
traj.append(Tgoal)

h = misc.DrawAxes(env,array(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.0,0.0,0.0])))),1.0)

myPatterns = []
# 2. Create a search pattern from the trajectory for the first manipulator
myPattern = SearchPattern(traj)
myPattern.setColor(array((0,1,1,0.5))) 
myPattern.show(env)
sys.stdin.readline()

myPatterns.append(myPattern)

# 3. Create a search pattern from the trajectory for the second manipulator
myPattern = SearchPattern(traj)
myPattern.setColor(array((1,1,0,0.5))) 
myPattern.show(env)
sys.stdin.readline()

myPatterns.append(myPattern)

for p in myPatterns:
    p.hide()
    sys.stdin.readline()

# 3. Add both robots
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

rotz=[]
rotz.append(pi);
rotz.append(0.0);

shift_robot0 = MakeTransform(matrix(rodrigues([0,0,rotz[0]])),transpose(matrix([0.5,0.0,0.0])))
shift_robot1 = MakeTransform(matrix(rodrigues([0,0,rotz[1]])),transpose(matrix([-1.5,0.0,0.0])))

robots[0].SetTransform(array(shift_robot0))
robots[1].SetTransform(array(shift_robot1))

# 4. Generate reachability models for robots
rmaps = []
rm = ReachabilityMap("./barrettwam_ik_solver",robots[0],robots[0].GetManipulators()[0])
rm.xmax = 0.3
rm.xmin = -0.3
rm.ymax = 0.3
rm.ymin = -0.3
rm.zmax = 0.3
rm.zmin = 0.0

rm.r = 0
rm.g = 0
rm.b = 1

rm.free_joint_val = 0.0
rm.free_joint_index = 2

rm.generate(env)
sys.stdin.readline()

rm.hide()
sys.stdin.readline()

# 4. Search for the pattern in the reachability model and get the results
# candidates = Reachability.solve(patterns, rmaps, constraints)

# 5. Iterate through the results (move the tool-center-point to Tstart of the pattern)

# 6. Add an object in the environment to a random location

# 7. Use the object location and define where Tstart_left and Tstart_right is on the object

# 8. Find the relative transform between Tstart_left and Tstart_right.
#    We assume that this relative transform will remain constant throughout the path.

# 9. 

env.Destroy()
RaveDestroy()

