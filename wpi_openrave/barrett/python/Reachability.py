from openravepy import *
from numpy import *
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *
import commands

class ReachabilitySphere:
    def __init__(self):
        self.radius = 0.025
        self.diameter = self.radius*2
        self.reachability = 0 # int: +1 for each direction the manipulator can approach
        self.directions = [] # string: +/-x, +/-y, +/-z
        self.comOffset = [] # How much does reaching to this point add to the center of mass of the robot?
        self.T = [] # transform of the sphere wrt the base of the manipulator. Azimuth is always +z in world coordinates, so rotation would be calculated in world coordinate frame.
        self.alpha = 0.5
        self.color = array((0,1,1,self.alpha))
        self.visible = True
        self.axisLength = 0.1


class ReachabilityMap:
    def __init__(self, iksolver, robot, manip):
        self.handles = []
        self.map = []
        self.Tbase = manip.GetBase().GetTransform() # This is the coordinate system of the map, usually attached to the base of the manipulator.
        
        # list of rotation matrices that we will evaluate around a point in space
        self.rm3D=[rodrigues([pi/2,0,0]),rodrigues([-pi/2,0,0]),rodrigues([0,pi/2,0]),rodrigues([0,-pi/2,0]),rodrigues([0,0,pi/2]),rodrigues([0,0,-pi/2])]
        # limits
        self.xmax=1.0
        self.xmin=-1.0
        self.ymax=1.0
        self.ymin=-1.0
        self.zmax=1.0
        self.zmin=-1.0

        # increments for left arm
        self.dx=ReachabilitySphere().diameter
        self.dy=ReachabilitySphere().diameter
        self.dz=ReachabilitySphere().diameter

        # If this manipulator has a free joint for its Fast IK Solver
        self.free_joint_val = 0.0 
        self.free_joint_index = None

        self.solver = iksolver
        self.robot = robot
        self.manip = manip
        self.armJoints = manip.GetArmJoints()

        self.r = 0
        self.g = 0
        self.b = 0
        
    def frange(self,start,stop,inc):
        i=start
        a=[]
        a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
        while i < stop:
            i += inc
            a.append(round(i,2)) # if we don't round the floatint-point number to 2 decimal places we get the exact value
        return a

    def show(self,myEnv):
        # draw all, append to handles
        self.handles=[]
        for s in self.map:
            self.handles.append(myEnv.plot3(points=dot(s.T,self.Tbase),
                                            pointsize=s.radius, # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                             #colors=array((0,0,0.15*reachability,0.5)), # This changes the color intensity
                                             colors=array(self.r,self.g,self.b,0.166*(s.reachability)), # This changes the transparency
                                             drawstyle=1
                                             ))

            # self.handles.append(myEnv.plot3(points=array((bt[0][3]+x,bt[1][3]+y,bt[2][3]+z)),
            #                                  pointsize=0.025, # In case dx, dy and dz are all equal, this should be half of that increment constant.
            #                                  #colors=array((0,0,0.15*reachability,0.5)), # This changes the color intensity
            #                                  colors=array((0,0,1,0.166*reachability)), # This changes the transparency
            #                                  drawstyle=1
            #                                  ))

    def hide(self):
        # destroy handles
        pass

    def update(self):
        pass

    def manip_reset(self):
        q=[]
        for j in range(len(self.armJoints)):
            q.append(0.0)
        self.robot.SetDOFValues(q,self.armJoints)

    def generate(self,env):
        # This is the main loop
        self.xarray = self.frange(self.xmin,self.xmax,self.dx)
        self.yarray = self.frange(self.ymin,self.ymax,self.dy)
        self.zarray = self.frange(self.zmin,self.zmax,self.dz)

        self.totalNumPoints = len(self.xarray)*len(self.yarray)*len(self.zarray)

        current_point_ind = 0
        for x in self.xarray:
            for y in self.yarray:
                for z in self.zarray:
                    #t =  dot(MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([x,y,z]))),self.Tbase)
                    tx = x # t[0,3]
                    ty = y # t[1,3] 
                    tz = z # t[2,3]

                    s = ReachabilitySphere()
                    s.reachability = 0
                    for rm in self.rm3D:
                        there_exists_at_least_one_good_solution = False # for this rotation
                        # rodrigues returns a numpy ndarray, we should convert it to a list before extracting data.
                        r00 = rm[0,0] 
                        r01 = rm[0,1]
                        r02 = rm[0,2]
                        
                        r10 = rm[1,0]
                        r11 = rm[1,1]
                        r12 = rm[1,2]
                         
                        r20 = rm[2,0]
                        r21 = rm[2,1]
                        r22 = rm[2,2]

                        Tbase_req = MakeTransform(rm, transpose(matrix([tx,ty,tz]))) # requested transform

                        cmd = self.solver + ' ' + str(r00) + ' ' + str(r01) + ' ' + str(r02) + ' ' + str(tx) + ' ' + str(r10) + ' ' + str(r11) + ' ' + str(r12) + ' ' + str(ty) + ' ' + str(r20) + ' ' + str(r21) + ' ' + str(r22) + ' ' + str(tz)
                        if(self.free_joint_index != None):
                            cmd = cmd + ' ' + str(self.free_joint_val)
                             
                        solutions_str = commands.getoutput(cmd)
                        solutions_float = []

                        if(solutions_str.find("Failed") != 0):
                            words = solutions_str.split()

                            for w in range(len(words)): 
                                if(words[w] == 'Found'):
                                    num_solutions = int(words[w+1])
                                elif(words[w] == '(free=0):'): # configuration comes after this word
                                    q = []
                                    for j in range(len(self.armJoints)):
                                        # the following will strip the comma in the end of the joint value and convert it into a float
                                        if(j == self.free_joint_index): # do a smarter comparison here. Get the joint name, and index, and see if it matches to the one that we actually set free in the ik solver.
                                            q.append(self.free_joint_val) # Set the free joint to zero
                                        else:
                                            q.append(float(words[w+1+j][0:len(words[w+1+j])-1]))
                                            # now we have a configuration for 7 joints for this solution
                                            # print q
                                            solutions_float.append(q)
                                             
                        if(solutions_float != []):
                            for sol in range(len(solutions_float)):
                                q = solutions_float[sol]
                                self.robot.SetDOFValues(q,self.armJoints)
                                # manipulator's end effector transform in world coord. frame
                                eet = self.manip.GetEndEffectorTransform()
                                Tbase_eet = dot(linalg.inv(self.Tbase),eet)
                                 
                                close_enough = True
                                for r in range(3): # rows
                                    for c in range(4): # columns
                                        if(not allclose(Tbase_req[r,c],Tbase_eet[r,c])):
                                            close_enough = False
                                            break

                                if(close_enough):
                                    if((not env.CheckCollision(self.robot)) and (not self.robot.CheckSelfCollision())):
                                        there_exists_at_least_one_good_solution = True
                                        break

                        if(there_exists_at_least_one_good_solution):
                            s.reachability += 1

                    if(s.reachability>0):
                        s.T = Tbase_eet
                        self.map.append(s)
                        self.handles.append(env.plot3(points=dot(s.T,self.Tbase)[0:3,3],
                                                      pointsize=s.radius, # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                                      colors=array((self.r,self.g,self.b,0.166*s.reachability)), # This changes the transparency
                                                      drawstyle=1
                                                      ))

                    current_point_ind += 1
                    if((current_point_ind%100)==0):
                        print str(current_point_ind),"/",str(self.totalNumPoints)
        self.manip_reset()

class SearchPattern:
    # transforms: a list of 
    def __init__(self,transforms):
        self.pattern = [] # a list of reachability spheres
        self.delta = 0.05 # discretization value in milimeters
        self.handles = []
        self.prePatternTransforms = transforms
        self.delta = 2*(ReachabilitySphere().radius)
        self.generate(transforms) # will discretize and generate a search pattern from the list of transforms

    def generate(self,transforms):
        # Put the first sphere where the first transform is
        for t in range(len(transforms)):
            s = ReachabilitySphere()
            if(t==0): # Put a reachability sphere on the first transform
                s.T = transforms[t]
                self.pattern.append(s)
            else: # Otherwise do mapping (discretization)
                r = self.map([transforms[0],transforms[t]])
                if(r != None):
                    s.T = r
                    self.pattern.append(s)
    
    def map(self,tCouple):
        debug = False
        # find the relative transform between the couple
        Trel = dot(linalg.inv(tCouple[0]),tCouple[1])

        # First, discretize the distance
        relx = Trel.tolist()[0][3]
        rely = Trel.tolist()[1][3]
        relz = Trel.tolist()[2][3]

        mx = relx%ReachabilitySphere().diameter
        dx = round(relx/ReachabilitySphere().diameter)
        if(mx>ReachabilitySphere().radius):
            dx+=1            
            
        my = rely%ReachabilitySphere().diameter
        dy = round(rely/ReachabilitySphere().diameter)
        if(my>ReachabilitySphere().radius):
            dy+=1

        mz = relz%ReachabilitySphere().diameter
        dz = round(relz/ReachabilitySphere().diameter)
        if(mz>ReachabilitySphere().radius):
            dz+=1

        if(dx == 0 and dy == 0 and dz == 0):
            Tnew = None
        else:
            Tnew = array(MakeTransform(Trel[0:3,0:3],transpose(matrix([tCouple[0][0,3]+dx*ReachabilitySphere().diameter, tCouple[0][1,3]+dy*ReachabilitySphere().diameter, tCouple[0][2,3]+dz*ReachabilitySphere().diameter]))))

        if(debug):
            print "Trel"
            print Trel
            print "mx: ",str(mx)
            print "dx: ",str(dx)
            print "my: ",str(my)
            print "dy: ",str(dy)
            print "mz: ",str(mz)
            print "dz: ",str(dz)
            print "Tnew: "
            print Tnew
            

        return Tnew
        
    
    def show(self,myEnv):
        # draw all reachability spheres and keep their handles
        for s in self.pattern:
            self.handles.append(myEnv.plot3(points=array((s.T.tolist()[0][3],s.T.tolist()[1][3],s.T.tolist()[2][3])),
                                            pointsize=s.radius, # In case dx, dy and dz are all equal, this should be half of that increment constant.
                                            colors=s.color, # This changes the transparency
                                            drawstyle=1)
                                )
            #self.handles.append(misc.DrawAxes(myEnv,s.T,s.axisLength))

    
        for p in self.prePatternTransforms:
            self.handles.append(misc.DrawAxes(myEnv,p,ReachabilitySphere().axisLength))
            
    def hide(self):
        # undraw all reachability spheres
        self.handles = []
    
    def setColor(self,color):
        # change the color of all reachability spheres
        for s in self.pattern:
            s.color = color
        pass
 
def satisfy():
    # candidates = []
    # for m in range(len(paths)):
    #     for p in paths[m]:
    #         for s in p:
    #             for n in range(m+1,len(paths)):
    #                 for o in paths[n]:
    #                     for t in o:
    #                         for c in constraints:
    #                             if(dot(s.T,t.T)==c):
    pass

def solve():
    # if(len(patterns) != len(rmaps)):
    # numManips = len(rmaps)
    # return error
    #
    # paths = []
    #
    # for m in range(numManips):
    # paths.append(rmaps[m].find(patterns[m]))
    #
    # candidates = satisfy(constraints, paths)
    # 
    # return candidates
    pass
