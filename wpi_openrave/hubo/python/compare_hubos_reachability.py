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

def frange(start,stop,inc):
    i=start
    a=[]
    a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
    while i < stop:
        i += inc
        a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
    return a

occupancy = {}
reachability_dict = {}

env = Environment()
RaveSetDebugLevel(1)
robot = env.ReadRobotURI('../../../openHubo/drchubo/drchubo-urdf/robots/drchubo.robot.xml')
env.Add(robot)

manips  = robot.GetManipulators()
drchubo_rightFoot = manips[3]
rf_eet = drchubo_rightFoot.GetEndEffectorTransform() # right foot end effector transform
shift_drchubo_to_its_right_foot = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.5,-1*rf_eet[1][3],-1*rf_eet[2][3]])))
robot.SetTransform(array(shift_drchubo_to_its_right_foot))

debug=False # if true, will print things out

env.SetViewer('qtcoin')

# list of rotation matrices that we will evaluate around a point in space
rm3D=[]
rm3D.append(rodrigues([pi/2,0,0]))
rm3D.append(rodrigues([-pi/2,0,0]))
rm3D.append(rodrigues([0,pi/2,0]))
rm3D.append(rodrigues([0,-pi/2,0]))
rm3D.append(rodrigues([0,0,pi/2]))
rm3D.append(rodrigues([0,0,-pi/2]))

drchubo_handles = [] # These are for the points we will draw

# limits for left arm
xmax=1.0
xmin=0.0
ymax=1.0
ymin=-1.0
zmax=1.0
zmin=-1.0

# increments for left arm
dx=0.05
dy=0.05
dz=0.05

###### DRCHubo Left Arm Reachability ##########################

# value of the joint that we set free for the ik solver
# the last number of the ik solver executable name indicates the
# joint index that we set free. For example if the executable name
# ends with _f5 it means the joint that has 5 as its index is set free.
# Then we manually define the joint value here with the following 
# variable for the ik solver.
free_joint_val = 0.0 
free_joint_index = 5

executable_name = "./drchubo_leftArm_ik_solver_f5"

total_num_points = len(frange(xmin,xmax,dx))*len(frange(ymin,ymax,dy))*len(frange(zmin,zmax,dz))
current_point_ind = 0

bt = manips[0].GetBase().GetTransform() # manipulator's base transform in world coord. frame

for x in frange(xmin,xmax,dx):
    for y in frange(ymin,ymax,dy):
        for z in frange(zmin,zmax,dz):
            tx = x
            ty = y
            tz = z
            reachability = 0
            for rm in rm3D:
                there_exists_at_least_one_good_solution = False # for this rotation
                # rodrigues returns a numpy ndarray, we should convert it to a list before extracting data.
                r00 = rm.tolist()[0][0] 
                r01 = rm.tolist()[0][1]
                r02 = rm.tolist()[0][2]
                
                r10 = rm.tolist()[1][0]
                r11 = rm.tolist()[1][1]
                r12 = rm.tolist()[1][2]
                
                r20 = rm.tolist()[2][0]
                r21 = rm.tolist()[2][1]
                r22 = rm.tolist()[2][2]
                
                cmd = executable_name + ' ' + str(r00) + ' ' + str(r01) + ' ' + str(r02) + ' ' + str(tx) + ' ' + str(r10) + ' ' + str(r11) + ' ' + str(r12) + ' ' + str(ty) + ' ' + str(r20) + ' ' + str(r21) + ' ' + str(r22) + ' ' + str(tz)

                if(free_joint_index != None):
                    cmd = cmd + ' ' + str(free_joint_val)

                solutions_str = commands.getoutput(cmd)
                solutions_float = []

                if(solutions_str.find("Failed") != 0):
                    words = solutions_str.split()
    
                    for w in range(len(words)): 
                        if(words[w] == 'Found'):
                            num_solutions = int(words[w+1])
                            if(debug): 
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
                                    if(debug):
                                        print solutions_float
                else:
                    if(debug):
                        print solutions_str

                if(solutions_float != []):
                    for s in range(len(solutions_float)):
                        q = solutions_float[s]
                        robot.SetDOFValues(q,[1,2,3,4,5,6,7])

                        eet = manips[0].GetEndEffectorTransform() # manipulator's end effector transform in world coord. frame
                        

                        relt = []
                        for r in range(3): # rows
                            relt.append([eet[r][0],eet[r][1],eet[r][2],eet[r][3]-bt[r][3]])

                        reqt = [[r00,r01,r02,tx],[r10,r11,r12,ty],[r20,r21,r22,tz]]# requested transform

                        #print "requested transform"
                        #print reqt
                        
                        #print "leftPalm is at:"
                        #print relt

                        #sys.stdin.readline()
                        
                        not_close_enough = False
                        for r in range(3): # rows
                            for c in range(4): # columns
                                if(not allclose(reqt[r][c],relt[r][c])):
                                    not_close_enough = True
                                    break

                        if(not_close_enough):
                            #print "Bad solution"
                            pass
                        else:
                            #print "Good solution"
                            pass
                            if((not env.CheckCollision(robot)) and (not robot.CheckSelfCollision())):
                                #print "IK solution good enough, no collision with the environment or self"
                                there_exists_at_least_one_good_solution = True
                                break
                            else:
                                #print "robot in collision."
                                pass
                        #sys.stdin.readline()
                            
                if(there_exists_at_least_one_good_solution):
                    # Increment this point's reachability by one
                    reachability += 1

            #print "Point: ",str(x),", ",str(y),", ",str(z)," is reachable from ",str(reachability)," different directions without collision."
            #print there_exists_at_least_one_good_solution
            #print reachability
            #print z
            if(reachability>0):
                mykey = str(x),str(y),str(z)
                reachability_dict[mykey]=reachability                
                occupancy[str(x),str(y),str(z)]='L'
                drchubo_handles.append(env.plot3(points=array((bt[0][3]+x,bt[1][3]+y,bt[2][3]+z)),
                                         pointsize=0.025, # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                         #colors=array((0,0,0.15*reachability,0.5)), # This changes the color intensity
                                         colors=array((0,0,1,0.166*reachability)), # This changes the transparency
                                         drawstyle=1
                                         ))
            current_point_ind += 1
            if((current_point_ind%100)==0):
                print str(current_point_ind),"/",str(total_num_points)

robot.SetDOFValues([0.0,0.0,0.0,0.0,0.0,0.0,0.0],[1,2,3,4,5,6,7])

####### DRC Hubo Right Arm Reachability ###########################################

# value of the joint that we set free for the ik solver
# the last number of the ik solver executable name indicates the
# joint index that we set free. For example if the executable name
# ends with _f5 it means the joint that has 5 as its index is set free.
# Then we manually define the joint value here with the following 
# variable for the ik solver.
free_joint_val = 0.0 
free_joint_index = 23

executable_name = "./drchubo_rightArm_ik_solver_f23"

total_num_points = len(frange(xmin,xmax,dx))*len(frange(ymin,ymax,dy))*len(frange(zmin,zmax,dz))
current_point_ind = 0

bt = manips[1].GetBase().GetTransform() # manipulator's base transform in world coord. frame

for x in frange(xmin,xmax,dx):
    for y in frange(ymin,ymax,dy):
        for z in frange(zmin,zmax,dz):
            tx = x
            ty = y
            tz = z
            reachability = 0
            for rm in rm3D:
                there_exists_at_least_one_good_solution = False # for this rotation
                # rodrigues returns a numpy ndarray, we should convert it to a list before extracting data.
                r00 = rm.tolist()[0][0] 
                r01 = rm.tolist()[0][1]
                r02 = rm.tolist()[0][2]
                
                r10 = rm.tolist()[1][0]
                r11 = rm.tolist()[1][1]
                r12 = rm.tolist()[1][2]
                
                r20 = rm.tolist()[2][0]
                r21 = rm.tolist()[2][1]
                r22 = rm.tolist()[2][2]
                
                cmd = executable_name + ' ' + str(r00) + ' ' + str(r01) + ' ' + str(r02) + ' ' + str(tx) + ' ' + str(r10) + ' ' + str(r11) + ' ' + str(r12) + ' ' + str(ty) + ' ' + str(r20) + ' ' + str(r21) + ' ' + str(r22) + ' ' + str(tz)

                if(free_joint_index != None):
                    cmd = cmd + ' ' + str(free_joint_val)

                solutions_str = commands.getoutput(cmd)
                solutions_float = []

                if(solutions_str.find("Failed") != 0):
                    words = solutions_str.split()
    
                    for w in range(len(words)): 
                        if(words[w] == 'Found'):
                            num_solutions = int(words[w+1])
                            if(debug): 
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
                                    if(debug):
                                        print solutions_float
                else:
                    if(debug):
                        print solutions_str

                if(solutions_float != []):
                    for s in range(len(solutions_float)):
                        q = solutions_float[s]
                        robot.SetDOFValues(q,[19,20,21,22,23,24,25])

                        eet = manips[1].GetEndEffectorTransform() # manipulator's end effector transform in world coord. frame
                        

                        relt = []
                        for r in range(3): # rows
                            relt.append([eet[r][0],eet[r][1],eet[r][2],eet[r][3]-bt[r][3]])

                        reqt = [[r00,r01,r02,tx],[r10,r11,r12,ty],[r20,r21,r22,tz]]# requested transform

                        #print "requested transform"
                        #print reqt
                        
                        #print "leftPalm is at:"
                        #print relt

                        #sys.stdin.readline()
                        
                        not_close_enough = False
                        for r in range(3): # rows
                            for c in range(4): # columns
                                if(not allclose(reqt[r][c],relt[r][c])):
                                    not_close_enough = True
                                    break

                        if(not_close_enough):
                            #print "Bad solution"
                            pass
                        else:
                            #print "Good solution"
                            pass
                            if((not env.CheckCollision(robot)) and (not robot.CheckSelfCollision())):
                                #print "IK solution good enough, no collision with the environment or self"
                                there_exists_at_least_one_good_solution = True
                                break
                            else:
                                #print "robot in collision."
                                pass
                        #sys.stdin.readline()
                            
                if(there_exists_at_least_one_good_solution):
                    # Increment this point's reachability by one
                    reachability += 1

            #print "Point: ",str(x),", ",str(y),", ",str(z)," is reachable from ",str(reachability)," different directions without collision."
            #print there_exists_at_least_one_good_solution
            #print reachability
            #print z
            if(reachability>0):
                mykey = str(x),str(y),str(z)
                if(mykey in occupancy):
                    occupancy[str(x),str(y),str(z)]='RL'
                else:
                    occupancy[str(x),str(y),str(z)]='R'
                
                if(mykey in reachability_dict):
                    reachability_dict[mykey]=(reachability_dict[mykey]+reachability)/2
                else:
                    reachability_dict[mykey]=reachability
                
                drchubo_handles.append(env.plot3(points=array((bt[0][3]+x,bt[1][3]+y,bt[2][3]+z)),
                                                 pointsize=0.025, # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                                 colors=array((1,0,0,0.166*reachability)), # This changes the transparency
                                                 drawstyle=1
                                                 ))

            current_point_ind += 1
            if((current_point_ind%100)==0):
                print str(current_point_ind),"/",str(total_num_points)

robot.SetDOFValues([0.0,0.0,0.0,0.0,0.0,0.0,0.0],[19,20,21,22,23,24,25])

# Both Arms Done, now update colors
drchubo_handles = []
print "updating colors for drchubo"
for x in frange(xmin,xmax,dx):
    for y in frange(ymin,ymax,dy):
        for z in frange(zmin,zmax,dz):
            mykey = str(x),str(y),str(z)
            if( mykey in occupancy):
                if(occupancy[mykey]=='R'):
                    mycolor = array((1,0,0,0.166*reachability_dict[mykey]))
                elif(occupancy[mykey]=='L'):
                    mycolor = array((0,0,1,0.166*reachability_dict[mykey]))
                elif(occupancy[mykey]=='RL'):
                    mycolor = array((1,0,1,0.166*reachability_dict[mykey]))
                
                drchubo_handles.append(env.plot3(points=array((bt[0][3]+x,bt[1][3]+y,bt[2][3]+z)),
                                                 pointsize=0.025, # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                                 colors=mycolor, # This changes the transparency
                                                 drawstyle=1
                                                 ))

print "DRCHubo Done! Press Enter to run Hubo+..."
sys.stdin.readline()
###################################################################################
occupancy = {}
dict_reachability = {}
rlhuboplus_handles = []

robot = env.ReadRobotURI('../../../openHubo/huboplus/rlhuboplus.robot.xml')
env.Add(robot)
manips  = robot.GetManipulators()
shift_rlhuboplus_back = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([-0.5,0.0,0.0])))
robot.SetTransform(array(shift_rlhuboplus_back))

executable_name = "./rlhuboplus_leftArm_ik_solver"
current_point_ind = 0
total_num_points = len(frange(xmin,xmax,dx))*len(frange(ymin,ymax,dy))*len(frange(zmin,zmax,dz))
free_joint_index = None
bt = manips[0].GetBase().GetTransform() # manipulator's base transform in world coord. frame
for x in frange(xmin,xmax,dx):
    for y in frange(ymin,ymax,dy):
        for z in frange(zmin,zmax,dz):
            tx = x
            ty = y
            tz = z
            reachability = 0
            for rm in rm3D:
                there_exists_at_least_one_good_solution = False # for this rotation
                # rodrigues returns a numpy ndarray, we should convert it to a list before extracting data.
                r00 = rm.tolist()[0][0] 
                r01 = rm.tolist()[0][1]
                r02 = rm.tolist()[0][2]
                
                r10 = rm.tolist()[1][0]
                r11 = rm.tolist()[1][1]
                r12 = rm.tolist()[1][2]
                
                r20 = rm.tolist()[2][0]
                r21 = rm.tolist()[2][1]
                r22 = rm.tolist()[2][2]
                
                cmd = executable_name + ' ' + str(r00) + ' ' + str(r01) + ' ' + str(r02) + ' ' + str(tx) + ' ' + str(r10) + ' ' + str(r11) + ' ' + str(r12) + ' ' + str(ty) + ' ' + str(r20) + ' ' + str(r21) + ' ' + str(r22) + ' ' + str(tz)

                if(free_joint_index != None):
                    cmd = cmd + ' ' + str(free_joint_val)

                solutions_str = commands.getoutput(cmd)
                solutions_float = []

                if(solutions_str.find("Failed") != 0):
                    words = solutions_str.split()
    
                    for w in range(len(words)): 
                        if(words[w] == 'Found'):
                            num_solutions = int(words[w+1])
                            if(debug): 
                                print "Found "+str(num_solutions)+" solutions:"
                        elif(words[w] == '(free=0):'): # configuration comes after this word
                            q = []
                            for j in range(6):
                                # the following will strip the comma in the end of the joint value and convert it into a float
                                q.append(float(words[w+1+j][0:len(words[w+1+j])-1]))
                                # now we have a configuration for 7 joints for this solution
                                # print q
                                solutions_float.append(q)
                                if(debug):
                                    print solutions_float
                else:
                    if(debug):
                        print solutions_str

                if(solutions_float != []):
                    
                    for s in range(len(solutions_float)):
                        q = solutions_float[s]
                        robot.SetDOFValues(q,[14,16,18,20,22,24])

                        eet = manips[0].GetEndEffectorTransform() # manipulator's end effector transform in world coord. frame
                        

                        relt = []
                        for r in range(3): # rows
                            relt.append([eet[r][0],eet[r][1],eet[r][2],eet[r][3]-bt[r][3]])

                        reqt = [[r00,r01,r02,tx],[r10,r11,r12,ty],[r20,r21,r22,tz]]# requested transform

                        #print "requested transform"
                        #print reqt
                        
                        #print "leftPalm is at:"
                        #print relt

                        #sys.stdin.readline()
                        
                        not_close_enough = False
                        for r in range(3): # rows
                            for c in range(4): # columns
                                if(not allclose(reqt[r][c],relt[r][c])):
                                    not_close_enough = True
                                    break

                        if(not_close_enough):
                            #print "Bad solution"
                            pass
                        else:
                            #print "Good solution"
                            pass
                            if((not env.CheckCollision(robot)) and (not robot.CheckSelfCollision())):
                                #print "IK solution good enough, no collision with the environment or self"
                                there_exists_at_least_one_good_solution = True
                                break
                            else:
                                #print "robot in collision."
                                pass

                            
                if(there_exists_at_least_one_good_solution):
                    # Increment this point's reachability by one
                    reachability += 1

            if(reachability>0):
                mykey = str(x),str(y),str(z)
                reachability_dict[mykey]=reachability                
                occupancy[str(x),str(y),str(z)]='L'
                drchubo_handles.append(env.plot3(points=array((bt[0][3]+x,bt[1][3]+y,bt[2][3]+z)),
                                         pointsize=0.025, # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                         #colors=array((0,0,0.15*reachability,0.5)), # This changes the color intensity
                                         colors=array((0,0,1,0.166*reachability)), # This changes the transparency
                                         drawstyle=1
                                         ))
                
            current_point_ind += 1
            if((current_point_ind%100)==0):
                print str(current_point_ind),"/",str(total_num_points)

robot.SetDOFValues([0.0,0.0,0.0,0.0,0.0,0.0],[14,16,18,20,22,24])

####### RLHUBO+ RIGHT ARM ######################

executable_name = "./rlhuboplus_rightArm_ik_solver"
current_point_ind = 0
total_num_points = len(frange(xmin,xmax,dx))*len(frange(ymin,ymax,dy))*len(frange(zmin,zmax,dz))
free_joint_index = None
bt = manips[1].GetBase().GetTransform() # manipulator's base transform in world coord. frame
for x in frange(xmin,xmax,dx):
    for y in frange(ymin,ymax,dy):
        for z in frange(zmin,zmax,dz):
            tx = x
            ty = y
            tz = z
            reachability = 0
            for rm in rm3D:
                there_exists_at_least_one_good_solution = False # for this rotation
                # rodrigues returns a numpy ndarray, we should convert it to a list before extracting data.
                r00 = rm.tolist()[0][0] 
                r01 = rm.tolist()[0][1]
                r02 = rm.tolist()[0][2]
                
                r10 = rm.tolist()[1][0]
                r11 = rm.tolist()[1][1]
                r12 = rm.tolist()[1][2]
                
                r20 = rm.tolist()[2][0]
                r21 = rm.tolist()[2][1]
                r22 = rm.tolist()[2][2]
                
                cmd = executable_name + ' ' + str(r00) + ' ' + str(r01) + ' ' + str(r02) + ' ' + str(tx) + ' ' + str(r10) + ' ' + str(r11) + ' ' + str(r12) + ' ' + str(ty) + ' ' + str(r20) + ' ' + str(r21) + ' ' + str(r22) + ' ' + str(tz)

                if(free_joint_index != None):
                    cmd = cmd + ' ' + str(free_joint_val)

                solutions_str = commands.getoutput(cmd)
                solutions_float = []

                if(solutions_str.find("Failed") != 0):
                    words = solutions_str.split()
    
                    for w in range(len(words)): 
                        if(words[w] == 'Found'):
                            num_solutions = int(words[w+1])
                            if(debug): 
                                print "Found "+str(num_solutions)+" solutions:"
                        elif(words[w] == '(free=0):'): # configuration comes after this word
                            q = []
                            for j in range(6):
                                # the following will strip the comma in the end of the joint value and convert it into a float
                                q.append(float(words[w+1+j][0:len(words[w+1+j])-1]))
                                # now we have a configuration for 7 joints for this solution
                                # print q
                                solutions_float.append(q)
                                if(debug):
                                    print solutions_float
                else:
                    if(debug):
                        print solutions_str

                if(solutions_float != []):
                    
                    for s in range(len(solutions_float)):
                        q = solutions_float[s]
                        robot.SetDOFValues(q,[13,15,17,19,21,23])

                        eet = manips[1].GetEndEffectorTransform() # manipulator's end effector transform in world coord. frame
                        

                        relt = []
                        for r in range(3): # rows
                            relt.append([eet[r][0],eet[r][1],eet[r][2],eet[r][3]-bt[r][3]])

                        reqt = [[r00,r01,r02,tx],[r10,r11,r12,ty],[r20,r21,r22,tz]]# requested transform

                        #print "requested transform"
                        #print reqt
                        
                        #print "leftPalm is at:"
                        #print relt

                        #sys.stdin.readline()
                        
                        not_close_enough = False
                        for r in range(3): # rows
                            for c in range(4): # columns
                                if(not allclose(reqt[r][c],relt[r][c])):
                                    not_close_enough = True
                                    break

                        if(not_close_enough):
                            #print "Bad solution"
                            pass
                        else:
                            #print "Good solution"
                            pass
                            if((not env.CheckCollision(robot)) and (not robot.CheckSelfCollision())):
                                #print "IK solution good enough, no collision with the environment or self"
                                there_exists_at_least_one_good_solution = True
                                break
                            else:
                                #print "robot in collision."
                                pass

                            
                if(there_exists_at_least_one_good_solution):
                    # Increment this point's reachability by one
                    reachability += 1

            if(reachability>0):
                mykey = str(x),str(y),str(z)
                if(mykey in occupancy):
                    occupancy[str(x),str(y),str(z)]='RL'
                else:
                    occupancy[str(x),str(y),str(z)]='R'
                
                if(mykey in reachability_dict):
                    reachability_dict[mykey]=(reachability_dict[mykey]+reachability)/2
                else:
                    reachability_dict[mykey]=reachability
                
                rlhuboplus_handles.append(env.plot3(points=array((bt[0][3]+x,bt[1][3]+y,bt[2][3]+z)),
                                                    pointsize=0.025, # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                                    colors=array((1,0,0,0.166*reachability)), # This changes the transparency
                                                    drawstyle=1
                                                    ))
            current_point_ind += 1
            if((current_point_ind%100)==0):
                print str(current_point_ind),"/",str(total_num_points)

robot.SetDOFValues([0.0,0.0,0.0,0.0,0.0,0.0],[13,15,17,19,21,23])
################################################

# Both Arms Done, now update colors
print "updating colors for rlhuboplus"
rlhuboplus_handles = []

for x in frange(xmin,xmax,dx):
    for y in frange(ymin,ymax,dy):
        for z in frange(zmin,zmax,dz):
            mykey = str(x),str(y),str(z)
            if( mykey in occupancy):
                if(occupancy[mykey]=='R'):
                    mycolor = array((1,0,0,0.166*reachability_dict[mykey]))
                elif(occupancy[mykey]=='L'):
                    mycolor = array((0,0,1,0.166*reachability_dict[mykey]))
                elif(occupancy[mykey]=='RL'):
                    mycolor = array((1,0,1,0.166*reachability_dict[mykey]))
                
                rlhuboplus_handles.append(env.plot3(points=array((bt[0][3]+x,bt[1][3]+y,bt[2][3]+z)),
                                                 pointsize=0.025, # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                                 colors=mycolor, # This changes the transparency
                                                 drawstyle=1
                                                 ))

###################################################################################
print "Hubo+ Done! Press Enter to exit..."
sys.stdin.readline()

env.Destroy()
RaveDestroy()
