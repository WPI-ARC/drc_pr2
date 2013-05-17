-huboplus.xacro
 This file is in URDF Xacro format and several macro parameters
 especially path must be reset:

 --effort: many joint in openHubo do not have torque limits, so set
   it to some default value, you many change it

 --lower: similar to the above but for joint lower bound

 --upper: similar to the above but for joint upper bound

 --to convert it to URDF format, use command:
 
   rosrun xacro xacro.py huboplus.xacro > huboplus.xml
 
 --to visualize and move it in RVIZ, refer to the 2.2 section
   of tutorial:

   http://www.ros.org/wiki/urdf/Tutorials 

-dae folder
 Contains all the geometry meshes

 

 
