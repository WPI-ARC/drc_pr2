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
from itertools import izip

## SYSTEM - FILE OPS ##
import sys
import os
from datetime import datetime
import time
import commands

from openravepy.databases import inversereachability,grasping
from openravepy.misc import InitOpenRAVELogging

## OTHER ##
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *

if __name__=='__main__':
    t="wheel"
    InitOpenRAVELogging()
    RaveSetDebugLevel(DebugLevel.Verbose)
    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    robot = env.ReadRobotXMLFile('../../../openHubo/huboplus/rlhuboplus.robot.xml')
    env.AddRobot(robot)
    if(t=="wheel"):
        target = env.ReadKinBodyXMLFile('../../models/driving_wheel.kinbody.xml')
        T1 = MakeTransform(rodrigues([0,0,pi/2]),transpose(matrix([0,0,0])))
        T2 = MakeTransform(rodrigues([pi/2,0,0]),transpose(matrix([0,0,0])))
        T3 = dot(T1,T2)
        T4 = MakeTransform(eye(3),transpose(matrix([0, 0.8, 1.0])))
        T_wheel = dot(T3,T4)
        target.SetTransform(array(T_wheel))        
    elif(t=="mug"):
        target = env.ReadKinBodyXMLFile('../../models/mug1.kinbody.xml')
    robot.SetDOFValues([-0.7,-0.7],[19,20])
    manip = robot.SetActiveManipulator('leftArm')

    env.AddKinBody(target)
    sys.stdin.readline()

    gmodel = grasping.GraspingModel(robot=robot,target=target)

    print "grasping model filename:"
    print gmodel.getfilename()  
    
    if not gmodel.load():
        print 'go to http://people.csail.mit.edu/liuhuan/pr2/openrave/.openrave/ to get the database for openrave r1741!'

    print "getting inverse reachability model"
    irmodel = inversereachability.InverseReachabilityModel(robot=robot)
    
    print "inverse reachability model filename:"
    print irmodel.getfilename()

    if not irmodel.load():
        print 'go to http://people.csail.mit.edu/liuhuan/pr2/openrave/.openrave/ to get the database for openrave r1741!'
    
    validgrasps,validindices = gmodel.computeValidGrasps(checkik=False,backupdist=0.01)

    print 'validgrasps',len(validgrasps)
    print 'validindices',validindices

    def graspiter():
        for grasp,graspindex in izip(validgrasps,validindices):
            #gmodel.showgrasp(grasp)
            yield gmodel.getGlobalGraspTransform(grasp,collisionfree=True),(gmodel,graspindex)

    densityfn,samplerfn,bounds = irmodel.computeAggregateBaseDistribution(graspiter(),logllthresh=-100)

    print 'densityfn',densityfn
    print 'bounds',bounds
    print 'samplerfn',samplerfn

    thresh=1.0
    zoffset=0.0

    result = inversereachability.InverseReachabilityModel.showBaseDistribution(env,densityfn,bounds,zoffset=zoffset,thresh=thresh)

    print "inverse reachability result"
    print result

    sys.stdin.readline()

    try:
        poses, gmo, jointvalues = samplerfn(5,1.0)
        print poses
        print gmo
        print jointvalues
    except openrave_exception, e:
        print e

    print "here"
    for pose in poses:
        robot.SetTransform(matrixFromPose(pose)) 
        sys.stdin.readline()

    sys.stdin.readline()
