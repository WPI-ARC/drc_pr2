#!/usr/bin/env python
from datetime import * # to use now()
from roslib import * # to use package.get_pkg_dir
from random import * # to use randint()
from numpy import * # to use pi

def randGen(limMin,limMax):
    # Generate random values based on environment boundaries
    x = random.randint(limMin,limMax)
    y = random.randint(limMin,limMax)
    z = 1.0
    yaw = random.random()*pi

    indX = x+limMax-1
    indY = y+limMax-1
    
    return [x,y,z,yaw,indX,indY]


if __name__ == '__main__':

    # Get the current time
    t = datetime.now()

    # Where do you want the urdf to be stored?
    # Note that you should run this script in the script or nodes directory of wpi_drc_sim

    pkgDir = packages.get_pkg_dir('wpi_drc_sim')
    urdfLocation = pkgDir + '/descriptions/'
    launchFileLocation = pkgDir + '/launch/'

    # Strip the year and hour and make a filename
    stamp = str(t)[0:10]+'_'+str(t)[11:19].replace(':','_')

    urdfName = 'pr2valve.env.urdf'

    launchFileName = stamp + '.launch'

    # Append the filename and type to file location
    urdfPath = urdfLocation + urdfName 
    launchPath = launchFileLocation + launchFileName

    # Open the file
    launch = open(launchPath,'w')

    # Dump the launch file with the appropriate urdf name in it
    launch.write('<launch><param name="/use_sim_time" value="true" /><arg name="gui" default="true"/><node name="gazebo" pkg="gazebo" type="gazebo" args="$(find gazebo_worlds)/worlds/empty.world" respawn="false" output="screen"/><!-- start gui --><group if="$(arg gui)"><node name="gazebo_gui" pkg="gazebo" type="gui" respawn="false" output="screen"/></group>')

    # Define boundaries for X and Y values (where the objects will be spawned)
    limMin = -5
    limMax = 5

    # Safety radius in meter (let's not spawn objects / robots too close to each other)
    sR = 3

    # How many instances of valve do we want?
    numModels = 7

    gridSize = limMax-limMin

    # Keep the occupancy grid
    occupancy = zeros((gridSize,gridSize))

    [x,y,z,yaw,indX,indY] = randGen(limMin,limMax)

    for m in range(0,numModels):

        [x,y,z,yaw,indX,indY] = randGen(limMin,limMax)
        
        while occupancy[indX,indY] != 0:
            [x,y,z,yaw,indX,indY] = randGen(limMin,limMax)
            
        launch.write('<node name="spawn_valve_'+str(m)+'" pkg="gazebo" type="spawn_model" args="-urdf -file '+ urdfPath  +' -x '+ str(x) +' -y ' + str(y) + ' -z '+ str(z) +' -Y '+ str(yaw) +' -model valve_urdf_' + str(m)  + '" respawn="false" output="screen" />')

        # Fill in the occupancy grid, consider the safety radius
        for ox in range(indX-sR,indX+sR):
            for oy in range(indY-sR,indY+sR):
                if(ox < 0):
                    ox = 0
                elif(ox > gridSize-1):
                    ox = gridSize-1
                    
                if(oy < 0):
                    oy = 0
                elif(oy > gridSize-1):
                    oy = gridSize-1
                        
                occupancy[ox,oy] = 1

    # Finally launch the robot somewhere
    [robotX,robotY,robotZ,robotYaw,gridIndX,gridIndY] = randGen(limMin,limMax)
    while occupancy[gridIndX,gridIndY] != 0:
        [robotX,robotY,robotZ,robotYaw,gridIndX,gridIndY] = randGen(limMin,limMax)
    
 
    launch.write('<!--start pr2 robot--> <!-- send pr2 urdf to param server --><include file="$(find pr2_description)/robots/upload_pr2.launch" /><!-- push robot_description to factory and spawn robot in gazebo --><node name="spawn_pr2_model" pkg="gazebo" type="spawn_model" args="-x ' +str(robotX) +' -y '+ str(robotY) +' -Y ' + str(robotYaw) + ' -unpause -urdf -param robot_description -model pr2 -ros_namespace /gazebo" respawn="false" output="screen" /><!-- default bringup script --><include file="$(find pr2_gazebo)/launch/pr2_bringup.launch" /> <!-- Load and Start Default Controllers --><include file="$(find pr2_controller_configuration_gazebo)/pr2_default_controllers.launch" /></launch>')

    

    # Close the file
    launch.close()
