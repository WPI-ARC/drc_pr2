#!/usr/bin/env python
from datetime import * # to use now()
from roslib import * # to use package.get_pkg_dir
from random import * # to use randint()
from numpy import * # to use pi
from math import * # to use ceil() and floor()
from copy import deepcopy

class Occupant:
    def __init__(self,name,pos):
        self.name = name
        self.pos = pos

class OccupancyGrid:
    def __init__(self):
        self.minBound = -10 # Minimum grid boundary in X and Y directions
        self.maxBound = 10 # Maximum grid boundary in X and Y directions
        self.resolution = 0.5 # In meters
        self.xdim = self.maxBound - self.minBound
        self.ydim = self.maxBound - self.minBound
        # zeros( (rows,columns) )
        self.map = zeros( (self.xdim/self.resolution,self.ydim/self.resolution) ); # 0 if the cell is empty, 1 if the cell is full
        self.iterationLimit = 1000
        self.listOfOccupants = []

    def randGen(self):
        # Generate random values based on environment boundaries
        yaw = random.random()*pi
        x1 = random.randint(self.minBound,self.maxBound)
        y1 = random.randint(self.minBound,self.maxBound)
        z = 1.0 # Note that this is only OK for the valve structure we're using. For autogenerating the robot we don't pay attention to this value.   
        return [x1,y1,z,yaw]
    
    def meters2indices(self,xm,ym):
        print "m2i"
        # xm: x in meters
        # ym: y in meters
        # xind: x index
        # yind: y index
        print 'xm: '+str(xm)
        print 'ym: '+str(ym)
        if(xm >= self.maxBound):
            print "xm is passed max bound. pulling back to limit."
            xm = self.maxBound-self.resolution
        if(xm <= self.minBound):
            print "xm is passed min bound. pulling back to limit."
            xm = self.minBound+self.resolution
        if(ym >= self.maxBound):
            print "ym is passed max bound. pulling back to limit."
            ym = self.maxBound-self.resolution
        if(ym <= self.minBound):
            print "ym is passed min bound. pulling back to limit."
            ym = self.minBound+self.resolution
            
        xind = int(floor((xm - self.minBound)/self.resolution))
        yind = int(floor((ym - self.minBound)/self.resolution))
        return [xind,yind]

    def isFree(self,pos,objRadius,clearance):
        print "isFree"
        allClear = True
        x2 = pos[0] - (objRadius*sin(pos[2]))
        y2 = pos[1] + (objRadius*cos(pos[2]))
        startX = min(pos[0],x2)-clearance
        stopX = max(pos[0],x2)+clearance+self.resolution # Make sure you include the last cell
        startY = min(pos[1],y2)-clearance
        stopY = max(pos[1],y2)+clearance+self.resolution # Make sure you include the last cell
        
        print "search lims[x]: "+str(startX)+" "+str(stopX)
        print "search lims[y]: "+str(startY)+" "+str(stopY)

        x = startX
        while(x < stopX):
            y = startY
            while(y < stopY):
                # For every single fracking cell, make sure that the neighbors are also empty
                startXCL = min(x-self.resolution,x+self.resolution)
                stopXCL = max(x-self.resolution,x+self.resolution)
                startYCL = min(y-self.resolution,y+self.resolution)
                stopYCL = max(y+self.resolution,y+self.resolution)
                
                print "search lims[x] - along the way: "+str(startXCL)+" "+str(stopXCL)
                print "search lims[y] - along the way: "+str(startYCL)+" "+str(stopYCL)
                xcl = startXCL
                while(xcl < stopXCL):                    
                    ycl = startYCL
                    print 'xi: '+str(self.meters2indices(xcl,ycl)[0])
                    while(ycl < stopYCL):
                        print 'yi: '+str(self.meters2indices(xcl,ycl)[1])
                        if(self.map[self.meters2indices(xcl,ycl)[0],self.meters2indices(xcl,ycl)[1]]!=0):
                            allClear = False
                        ycl = ycl + self.resolution
                    xcl = xcl + self.resolution

                y = y + self.resolution
            x = x + self.resolution

        if(allClear):
            self.fillIn([startX,stopX,startY,stopY])
            return True
        else:
            return False
    

    def searchEmptyZone(self,name,objRadius,clearance):
        found = False
        i = 0
        while(not(found)):
            pos=self.randGen()
            if(self.isFree(pos,objRadius,clearance)):
                found = True
            i = i +1
            if(i == self.iterationLimit):
                break
        if(found):
            o = Occupant(name,pos)
            self.listOfOccupants.append(o)
            return pos
        else:
            return None

    def fillIn(self,zone):
        print "Filling in"
        startX = zone[0]
        stopX = zone[1]
        startY = zone[2]
        stopY = zone[3]

        print "from - to [x]: "+str(startX)+' '+str(stopX)
        print "from - to [y]: "+str(startY)+' '+str(stopY)

        x = startX
        while(x < stopX):
            y = startY
            print 'xf: '+str(self.meters2indices(x,y)[0])
            while(y < stopY):
                print 'yf: '+str(self.meters2indices(x,y)[1])
                self.map[self.meters2indices(x,y)[0],self.meters2indices(x,y)[1]] = 1
                y = y + self.resolution
            x = x + self.resolution
                    

def generateValveLoc(name,my_og):
    # Safety radius in meter (let's not spawn objects / robots too close to each other)
    clearance = 0.5 # in meters
    valveBridgeLength = 2 # This comes from the model we're using for the valve. This is the length of the middle bridge that carries the valve handle. We have to know this to calculate which cells are occupied.

    # This function may return a position vector or None. It should be checked at the higher level.
    return my_og.searchEmptyZone(name,valveBridgeLength,clearance)
        

def generateRobotLoc(name,my_og):
    # Safety radius in meter (let's not spawn objects / robots too close to each other)
    clearance = 0.5 # in meters
    robotBaseRadius = 0.5
    return my_og.searchEmptyZone(name,robotBaseRadius,clearance)

if __name__ == '__main__':
    
    og = OccupancyGrid()

    # Get the current time
    t = datetime.now()

    # Where do you want the urdf to be stored?
    # Note that you should run this script in the script or nodes directory of wpi_drc_sim

    pkgDir = packages.get_pkg_dir('wpi_drc_sim')
    launchFileLocation = pkgDir + '/launch/'

    # Strip the year and hour and make a filename
    stamp = str(t)[0:10]+'_'+str(t)[11:19].replace(':','_')
    launchFileName = stamp + '.launch'

    # Append the filename and type to file location
    launchPath = launchFileLocation + launchFileName

    # Open the file
    launch = open(launchPath,'w')

    # Dump the launch file with the appropriate urdf name in it
    launch.write('<launch><param name="/use_sim_time" value="true" /><node name="gazebo" pkg="gazebo" type="gazebo" args="-r $(find gazebo_worlds)/worlds/empty.world" respawn="false" output="screen"/><!--send valve urdf to param server-->')


    # How many instances of valve do we want?
    numModels = 4

    randomHeight = True # Generate height of the valve randomly?
    randomPitch = True # Generate inclination of the valve randomly?
    randomValve = True # Generate the valve handle type randomly?

    # Where to save this autogenerated urdf?
    urdfLocation = pkgDir + '/descriptions/'

    # What are the valve handle options we have?
    # valveNames = ['logitech_driving_force_pro_wheel','red-valve-3-spokes-handle','red-valve-6-star','red-valve-shallow','silver-valve-8-star']
    #  Note: 6-star seems to be too heavy. In Gazebo it pulls the bridge down :/
    valveNames = ['logitech_driving_force_pro_wheel','red-valve-3-spokes-handle','red-valve-shallow','silver-valve-8-star']
    
    for m in range(0,numModels):


        urdfName = stamp + '-' + str(m) + '.urdf'
        urdfPath = urdfLocation + urdfName
        urdf = open(urdfPath,'w')

        if(randomHeight):
            # Generate the valve (height and angle will be random)
            minHeight = 0.5 # [In meters]. This will set how low the valve can be. Max possible height will be 1 meter above this value.
            
            #heightOffset = minHeight - 1.0 # We need this offset because Z=0 is hardcoded at 1.0m (the height of the support legs are 2.0m and the middle beam is generated in the middle)
            
            h = random.random() # This will generate a random number between 0 and 1.
            
            h = h - minHeight # This is the height value we will write in the urdf parameter
            
        else:
            h = 0 # Default height

            
        if(randomPitch):
            p = random.random()*2*pi # This 
        else:
            p = 0 # Default pitch

        if(randomValve):
            v = random.randint(0,len(valveNames)-1)
        else:
            v = 0 # Default valve handle
            
        # Get the valve name
        vName = valveNames[v]

        # Now dump all the variables and the text in the urdf            
        urdf.write('<?xml version="1.0"?><robot name="valve"><link name="valve_base_left_support"><visual><geometry><box size="0.1 0.1 2.0"/></geometry><origin rpy="0 0 0" xyz="0 0 0"/><material name="blueish"><color rgba="0.1 0.1 0.1 0.5"/></material></visual><collision><geometry><box size="0.1 0.1 2.0"/></geometry></collision><inertial><origin xyz="0 0 0"/><mass value="10"/><inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/></inertial></link><link name="valve_base_right_support"><visual><geometry><box size="0.1 0.1 2.0"/></geometry><origin rpy="0 0 0" xyz="0 0 0"/><material name="blueish"><color rgba="0.1 0.1 0.1 0.5"/></material></visual><collision><geometry><box size="0.1 0.1 2.0"/></geometry></collision><inertial><origin xyz="0 0 0"/><mass value="10"/><inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/></inertial></link><link name="valve_base_middle_bridge"><visual><geometry><cylinder length="2.0" radius="0.05"/></geometry><origin rpy="1.57075 0 0" xyz="0 0 0"/><material name="whiteish"><color rgba="1.0 1.0 1.0 1.0"/></material></visual><collision><geometry><cylinder length="2.0" radius="0.05"/></geometry><origin rpy="1.57075 0 0" xyz="0 0 0"/></collision><inertial><origin xyz="0 0 0"/><mass value="0.1"/><inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/></inertial></link><link name="handle"><visual><geometry><mesh filename="package://wpi_drc_sim/meshes/valve_models/' + vName  + '.dae" scale="0.05 0.05 0.05"/></geometry></visual><collision><geometry><mesh filename="package://wpi_drc_sim/meshes/valve_models/' + vName + '.stl" scale="0.05 0.05 0.05"/></geometry></collision><inertial><origin xyz="0 0 0"/><mass value="0.1"/><inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/></inertial></link><link name="rod"><visual><geometry><cylinder length="0.2" radius="0.005"/></geometry><origin rpy="0 1.57075 0" xyz="0.1 0 0"/><material name="whiteish"><color rgba="1.0 1.0 1.0 1.0"/></material></visual><collision><geometry><cylinder length="0.1" radius="0.005"/></geometry><origin rpy="0 1.57075 0" xyz="0.1 0 0"/></collision><inertial><origin xyz="0 0 0"/><mass value="0.1"/><inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/></inertial></link><joint name="left_support" type="fixed"><parent link="valve_base_left_support"/><child link="valve_base_middle_bridge"/><origin xyz="0 1 ' + str(-h)  + '"/></joint><joint name="right_support" type="fixed"><parent link="valve_base_middle_bridge"/><child link="valve_base_right_support"/><origin xyz="0 1 ' + str(h)  + '"/></joint><joint name="body_to_rod" type="fixed"><parent link="valve_base_middle_bridge"/><child link="rod"/><origin xyz="0 0 0" rpy="0 ' + str(p)  + ' 0"/></joint><joint name="rod_to_handle" type="continuous"><parent link="rod"/><child link="handle"/><axis xyz="1 0 0"/><origin xyz="0.2 0 0"/></joint><gazebo reference="valve_base_middle_bridge"><material>Gazebo/White</material></gazebo></robot>')
        urdf.close()
        
        # Launch the valve somewhere empty
        vPos = generateValveLoc(vName+'_'+str(m),og)
        if(vPos != None):
            [vX,vY,vZ,vYaw] = deepcopy(vPos)
            

            # Write it in the launch file
            launch.write('<param name="valve_urdf_' + str(m)  + '" textfile="$(find wpi_drc_sim)/descriptions/' + urdfName + '" /><node name="spawn_valve_'+str(m)+'" pkg="gazebo" type="spawn_model" args="-urdf -param valve_urdf_'+ str(m)  +' -x '+ str(vX) +' -y ' + str(vY) + ' -z '+ str(vZ) +' -Y '+ str(vYaw) +' -model valve_urdf_' + str(m)  + '" respawn="false" output="screen" />')

    # Finally launch the robot somewhere
    rPos = generateRobotLoc('pr2',og)
    if(rPos != None):
        [robotX,robotY,robotZ,robotYaw] = rPos
         
        launch.write('<!--start pr2 robot--> <!-- send pr2 urdf to param server --><include file="$(find pr2_description)/robots/upload_pr2.launch" /><!-- push robot_description to factory and spawn robot in gazebo --><node name="spawn_pr2_model" pkg="gazebo" type="spawn_model" args="-x ' +str(robotX) +' -y '+ str(robotY) +' -Y ' + str(robotYaw) + ' -unpause -urdf -param robot_description -model pr2 -ros_namespace /gazebo" respawn="false" output="screen" /><!-- default bringup script --><include file="$(find pr2_gazebo)/launch/pr2_bringup.launch" /> <!-- Load and Start Default Controllers --><include file="$(find pr2_controller_configuration_gazebo)/pr2_default_controllers.launch" /></launch>')

    # Close the file
    launch.close()
    
    print "List of occupants:"
    for o in og.listOfOccupants:
        print o.name
        print o.pos
        
    print "Occupancy Map"

    line = '|__|'
    for c in range(len(og.map[0])):
        if( c < 10):
            line = line + '|0'+str(c)+'|'
        else:
            line = line + '|'+str(c)+'|'

    print line

    for r in range(len(og.map)):
        if(r < 10):
            line = '|0'+str(r)+'|'
        else:
            line = '|'+str(r)+'|'

        for c in range(len(og.map[r])):
            if(og.map[r][c] == 1.0):
                icon = '|xx|'
            elif(c==floor(len(og.map[r])/2) and r==floor(len(og.map)/2)):
                icon = '|++|'
            else:
                icon = '|__|'
            line = line +icon
        print line
