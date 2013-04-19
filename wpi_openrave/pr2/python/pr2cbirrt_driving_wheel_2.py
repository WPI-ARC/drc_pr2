# Ben Suay, RAIL
# November 2012
# Worcester Polytechnic Institute
#
# This code is the translation of hubo_turn_valve.m (written by Dmitry Berenson) into Python for the PR2
# It follows the same flow. The environment file is slightly different

# The following is the list of links for rlhuboplus.robot.xml (comes with OpenHUBO)
#
# name                index parents            
# ---------------------------------------------
# rightFoot           0                        
# Body_RAR            1     rightFoot          
# Body_RAP            2     Body_RAR           
# Body_RKP            3     Body_RAP           
# Body_RHP            4     Body_RKP           
# Body_RHR            5     Body_RHP           
# Body_RHY            6     Body_RHR           
# Body_Hip            7     Body_RHY           
# Body_Torso          8     Body_Hip           
# Body_LHY            9     Body_Hip           
# Body_LHR            10    Body_LHY           
# Body_LHP            11    Body_LHR           
# Body_LKP            12    Body_LHP           
# Body_LAP            13    Body_LKP           
# Body_LAR            14    Body_LAP           
# leftFoot            15    Body_LAR           
# Body_RSP            16    Body_Torso         
# Body_LSP            17    Body_Torso         
# Body_RSR            18    Body_RSP           
# Body_LSR            19    Body_LSP           
# Body_RSY            20    Body_RSR           
# Body_LSY            21    Body_LSR           
# Body_REP            22    Body_RSY           
# Body_LEP            23    Body_LSY           
# Body_RWY            24    Body_REP           
# Body_LWY            25    Body_LEP           
# Body_RWP            26    Body_RWY           
# rightPalm           27    Body_RWP           
# Body_LWP            28    Body_LWY           
# leftPalm            29    Body_LWP           
# Body_HNR            30    Body_Torso         
# Body_HNP            31    Body_HNR           
# rightIndexProximal  32    Body_RWP           
# rightIndexMedial    33    rightIndexProximal 
# rightIndexDistal    34    rightIndexMedial   
# rightMiddleProximal 35    Body_RWP           
# rightMiddleMedial   36    rightMiddleProximal
# rightMiddleDistal   37    rightMiddleMedial  
# rightRingProximal   38    Body_RWP           
# rightRingMedial     39    rightRingProximal  
# rightRingDistal     40    rightRingMedial    
# rightPinkyProximal  41    Body_RWP           
# rightPinkyMedial    42    rightPinkyProximal 
# rightPinkyDistal    43    rightPinkyMedial   
# rightThumbProximal  44    Body_RWP           
# rightThumbMedial    45    rightThumbProximal 
# rightThumbDistal    46    rightThumbMedial   
# leftIndexProximal   47    Body_LWP           
# leftIndexMedial     48    leftIndexProximal  
# leftIndexDistal     49    leftIndexMedial    
# leftMiddleProximal  50    Body_LWP           
# leftMiddleMedial    51    leftMiddleProximal 
# leftMiddleDistal    52    leftMiddleMedial   
# leftRingProximal    53    Body_LWP           
# leftRingMedial      54    leftRingProximal   
# leftRingDistal      55    leftRingMedial     
# leftPinkyProximal   56    Body_LWP           
# leftPinkyMedial     57    leftPinkyProximal  
# leftPinkyDistal     58    leftPinkyMedial    
# leftThumbProximal   59    Body_LWP           
# leftThumbMedial     60    leftThumbProximal  
# leftThumbDistal     61    leftThumbMedial    
# ---------------------------------------------
# name                index parents            
#
# ##############################################
# 
# The following is the manipulator information for rlhuboplus.robot.xml 
# 
# name      base       end       arm-dof gripper-dof arm               gripper
# ----------------------------------------------------------------------------
# leftArm   Body_Torso leftPalm  6       0           14,16,18,20,22,24        
# rightArm  Body_Torso rightPalm 6       0           13,15,17,19,21,23        
# leftFoot  Body_Hip   leftFoot  6       0           2,4,6,8,10,12            
# rightFoot Body_Hip   rightFoot 6       0           1,3,5,7,9,11             
# ----------------------------------------------------------------------------
# name      base       end       arm-dof gripper-dof arm               gripper
# 
# ##############################################
#
# The following is the list of links for the PR2 (robots/pr2-beta-static.zae)
#
# name                                               index parents                                   
# ---------------------------------------------------------------------------------------------------
# base_footprint                                     0                                               
# base_link                                          1     base_footprint                            
# base_bellow_link                                   2     base_link                                 
# base_laser_link                                    3     base_link                                 
# bl_caster_rotation_link                            4     base_link                                 
# bl_caster_l_wheel_link                             5     bl_caster_rotation_link                   
# bl_caster_r_wheel_link                             6     bl_caster_rotation_link                   
# br_caster_rotation_link                            7     base_link                                 
# br_caster_l_wheel_link                             8     br_caster_rotation_link                   
# br_caster_r_wheel_link                             9     br_caster_rotation_link                   
# fl_caster_rotation_link                            10    base_link                                 
# fl_caster_l_wheel_link                             11    fl_caster_rotation_link                   
# fl_caster_r_wheel_link                             12    fl_caster_rotation_link                   
# fr_caster_rotation_link                            13    base_link                                 
# fr_caster_l_wheel_link                             14    fr_caster_rotation_link                   
# fr_caster_r_wheel_link                             15    fr_caster_rotation_link                   
# torso_lift_link                                    16    base_link                                 
# head_pan_link                                      17    torso_lift_link                           
# head_tilt_link                                     18    head_pan_link                             
# head_plate_frame                                   19    head_tilt_link                            
# projector_wg6802418_frame                          20    head_plate_frame                          
# projector_wg6802418_child_frame                    21    projector_wg6802418_frame                 
# sensor_mount_link                                  22    head_plate_frame                          
# double_stereo_link                                 23    sensor_mount_link                         
# narrow_stereo_link                                 24    double_stereo_link                        
# narrow_stereo_gazebo_l_stereo_camera_frame         25    narrow_stereo_link                        
# narrow_stereo_gazebo_l_stereo_camera_optical_frame 26    narrow_stereo_gazebo_l_stereo_camera_frame
# narrow_stereo_gazebo_r_stereo_camera_frame         27    narrow_stereo_gazebo_l_stereo_camera_frame
# narrow_stereo_gazebo_r_stereo_camera_optical_frame 28    narrow_stereo_gazebo_r_stereo_camera_frame
# narrow_stereo_optical_frame                        29    narrow_stereo_link                        
# wide_stereo_link                                   30    double_stereo_link                        
# wide_stereo_gazebo_l_stereo_camera_frame           31    wide_stereo_link                          
# wide_stereo_gazebo_l_stereo_camera_optical_frame   32    wide_stereo_gazebo_l_stereo_camera_frame  
# wide_stereo_gazebo_r_stereo_camera_frame           33    wide_stereo_gazebo_l_stereo_camera_frame  
# wide_stereo_gazebo_r_stereo_camera_optical_frame   34    wide_stereo_gazebo_r_stereo_camera_frame  
# wide_stereo_optical_frame                          35    wide_stereo_link                          
# high_def_frame                                     36    sensor_mount_link                         
# high_def_optical_frame                             37    high_def_frame                            
# imu_link                                           38    torso_lift_link                           
# l_shoulder_pan_link                                39    torso_lift_link                           
# l_shoulder_lift_link                               40    l_shoulder_pan_link                       
# l_upper_arm_roll_link                              41    l_shoulder_lift_link                      
# l_upper_arm_link                                   42    l_upper_arm_roll_link                     
# l_elbow_flex_link                                  43    l_upper_arm_link                          
# l_forearm_roll_link                                44    l_elbow_flex_link                         
# l_forearm_cam_frame                                45    l_forearm_roll_link                       
# l_forearm_cam_optical_frame                        46    l_forearm_cam_frame                       
# l_forearm_link                                     47    l_forearm_roll_link                       
# l_wrist_flex_link                                  48    l_forearm_link                            
# l_wrist_roll_link                                  49    l_wrist_flex_link                         
# l_gripper_palm_link                                50    l_wrist_roll_link                         
# l_gripper_l_finger_link                            51    l_gripper_palm_link                       
# l_gripper_l_finger_tip_link                        52    l_gripper_l_finger_link                   
# l_gripper_led_frame                                53    l_gripper_palm_link                       
# l_gripper_motor_accelerometer_link                 54    l_gripper_palm_link                       
# l_gripper_motor_slider_link                        55    l_gripper_palm_link                       
# l_gripper_motor_screw_link                         56    l_gripper_motor_slider_link               
# l_gripper_r_finger_link                            57    l_gripper_palm_link                       
# l_gripper_r_finger_tip_link                        58    l_gripper_r_finger_link                   
# l_gripper_l_finger_tip_frame                       59    l_gripper_r_finger_tip_link               
# l_gripper_tool_frame                               60    l_gripper_palm_link                       
# l_torso_lift_side_plate_link                       61    torso_lift_link                           
# laser_tilt_mount_link                              62    torso_lift_link                           
# laser_tilt_link                                    63    laser_tilt_mount_link                     
# r_shoulder_pan_link                                64    torso_lift_link                           
# r_shoulder_lift_link                               65    r_shoulder_pan_link                       
# r_upper_arm_roll_link                              66    r_shoulder_lift_link                      
# r_upper_arm_link                                   67    r_upper_arm_roll_link                     
# r_elbow_flex_link                                  68    r_upper_arm_link                          
# r_forearm_roll_link                                69    r_elbow_flex_link                         
# r_forearm_cam_frame                                70    r_forearm_roll_link                       
# r_forearm_cam_optical_frame                        71    r_forearm_cam_frame                       
# r_forearm_link                                     72    r_forearm_roll_link                       
# r_wrist_flex_link                                  73    r_forearm_link                            
# r_wrist_roll_link                                  74    r_wrist_flex_link                         
# r_gripper_palm_link                                75    r_wrist_roll_link                         
# r_gripper_l_finger_link                            76    r_gripper_palm_link                       
# r_gripper_l_finger_tip_link                        77    r_gripper_l_finger_link                   
# r_gripper_led_frame                                78    r_gripper_palm_link                       
# r_gripper_motor_accelerometer_link                 79    r_gripper_palm_link                       
# r_gripper_motor_slider_link                        80    r_gripper_palm_link                       
# r_gripper_motor_screw_link                         81    r_gripper_motor_slider_link               
# r_gripper_r_finger_link                            82    r_gripper_palm_link                       
# r_gripper_r_finger_tip_link                        83    r_gripper_r_finger_link                   
# r_gripper_l_finger_tip_frame                       84    r_gripper_r_finger_tip_link               
# r_gripper_tool_frame                               85    r_gripper_palm_link                       
# r_torso_lift_side_plate_link                       86    torso_lift_link                           
# torso_lift_motor_screw_link                        87    base_link                                 
# ---------------------------------------------------------------------------------------------------
# name                                               index parents 
# 
#
# ########################################
# 
# The following is manipulator information for the PR2 (robots/pr2-beta-static.zae)
#
#
# name            base            end                         arm-dof gripper-dof arm                     gripper
# ---------------------------------------------------------------------------------------------------------------
# leftarm_camera  torso_lift_link l_forearm_cam_optical_frame 5       0           15,16,17,18,19                 
# rightarm_camera torso_lift_link r_forearm_cam_optical_frame 5       0           27,28,29,30,31                 
# head_torso      base_link       wide_stereo_optical_frame   3       0           12,13,14                       
# head            torso_lift_link wide_stereo_optical_frame   2       0           13,14                          
# rightarm_torso  base_link       r_gripper_palm_link         8       1           12,27,28,29,30,31,32,33 34     
# rightarm        torso_lift_link r_gripper_palm_link         7       1           27,28,29,30,31,32,33    34     
# leftarm_torso   base_link       l_gripper_palm_link         8       1           12,15,16,17,18,19,20,21 22     
# leftarm         torso_lift_link l_gripper_palm_link         7       1           15,16,17,18,19,20,21    22     
# ---------------------------------------------------------------------------------------------------------------
# name            base            end                         arm-dof gripper-dof arm                     gripper
#
#
# ########################################
#
#

from openravepy import *
import sys
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
    import numpy
import time
from rodrigues import *
from TransformMatrix import *
from str2num import *
from TSR import *
import os # for file operations

def run():
    normalsmoothingitrs = 150;
    fastsmoothingitrs = 20;

    env = Environment()
    RaveSetDebugLevel(DebugLevel.Info) # set output level to debug
    env.Load('../../models/driving_wheel.robot2.xml')
    robotid = env.ReadRobotURI('robots/pr2-beta-static.zae')
    crankid = env.GetRobots()[0]

    # Move the wheel to the front of the robot, somewhere it can grasp and rotate
    # Note that when mounted on our foldable table the height of the wheel is 0.8128
    crankid.SetTransform(array(MakeTransform(matrix(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2,0,0]))), transpose(matrix([0.5,0,0.8128])))))

    env.Add(robotid)
    env.SetViewer('qtcoin')
    # Note: RaveCreateProblem will be deprecated and it uses RaveCrateModule in it anyways so we just use RaveCreateModule instead
    probs_cbirrt = RaveCreateModule(env,'CBiRRT')
    probs_crankmover = RaveCreateModule(env,'CBiRRT')

    manips = robotid.GetManipulators()
    crankmanip = crankid.GetManipulators()

    # for m in range(len(manips)):
    #     print manips[m].armjoints
    
    # Note: The argument you pass in must be the same as (the robot name) defined in the .zae or .xml file
    # Try - Catch is very useful to figure out what the problem is (if any) with cbirrt function calls
    try:
        env.AddModule(probs_cbirrt,'pr2')
        env.AddModule(probs_crankmover,'crank')
    except openrave_exception, e:
        print e

    print "Getting Loaded Problems"
    probs = env.GetLoadedProblems()
    print probs[0] # --> probs_cbirrt
    print probs[1] # --> probs_crankmover

    # 'leftarm' joint indices for pr2: [15 - 21] 
    # 'rightarm' joint indices for pr2: [27 - 33]
    robotid.SetActiveDOFs([12,15,16,17,18,19,20,21,27,28,29,30,31,32,33])

    # Our valve has only 1 joint anyway
    # crankid.SetActiveDOFs([0])

    # debug: Following block tests if we can rotate the valve with the SetDOFValues function
    # delta = pi/10
    # rotangle = 0
    # for i in range(10):
    #     crankid.SetDOFValues([rotangle],[0])
    #     rotangle=rotangle+delta
    #     print "Press enter to continue..."
    #     sys.stdin.readline()

    # Tweak the pose of the PR2
    # SetDOFValues(values, indices)
    #
    #
    # Set Right Arm Pose
    # 27: shoulder pan
    # 29: shoulder lift
    # 30: elbow smt
    # 32: wrist smt
    # 33: wrist smt
    # 34: gripper
    robotid.SetDOFValues([-0.85,-1.00,-1.57,-1.00,-1.00,0.548],[27,29,30,32,33,34])
    # Set Left Arm Pose
    robotid.SetDOFValues([0.85,1.00,-1.57,-1.00,1.00,0.548],[15,17,18,20,21,22])
    # Torso Lift Link Pose (note: max limit is 0.31)
    # robotid.SetDOFValues([0.2],[12])
    
    
    robotid.SetActiveDOFs([12,15,16,17,18,19,20,21,22,27,28,29,30,31,32,33,34])
    # Note: GetDOFValues() returns all 39 joints' values, whereas GetActiveDOFValues returns only the active joints' values
    initconfig = robotid.GetActiveDOFValues()
    
    print "Press enter to continue 1..."
    sys.stdin.readline()
    
    links = robotid.GetLinks()

    # Note: in matlab version the following block does something related to legs
    # Since the PR2 doesn't have any, I'm skipping this part.
    #
    # #################
    # Tee = 
    # for i in range(4):
    #     Tlink = MakeTransform(links(1:9,manips{i}.eelink+1),links(10:12,manips{i}.eelink+1));
    #     Tgrasp = [manips{i}.Tgrasp;[0 0 0 1]];    
    #     Tee{i} = Tlink*Tgrasp;
    # #################
        
    # in matlab version this is cell(1,4), because there are 4 manipulators. 
    # 
    # Tlink is a 4x4 matrix
    # 
    # Note: I checked the documentation for manips{}.Tgrasp and it says it's a 4x4 matrix but somehow
    #       in matlab it's 3x4 so Dmitry inserts an extra row in the end with
    #
    #       Tgrasp = [manips{i}.Tgrasp;[0 0 0 1]]; 
    #Tee = numpy.zeros(2,4,4) # Create a 3D array of depth 2 and size 4x4 
    
    #Tlink = MakeTransform()
    #Tgrasp = []
    
    #for i in range(4):
    #    Tee[i]=something
        
    crankjointind = 0

    # probs[0] is equivalent to "probs.cbirrt" in the matlab version
    # Note: jointtm is a string. --> joint transformation matrix for the valve
    # Also Note: jointtm has no rotation wrt the world coordinate frame
    jointtm = probs[0].SendCommand('GetJointTransform name crank jointind '+str(crankjointind))
    
    # Driving wheel has only one manipulator
    # Note: This transform actually is 23 degrees slant around X because of the physical structure of the racing wheel
    maniptm = crankid.GetManipulators()[0].GetTransform()

    # print "manipulator's transformation matrix"
    # print maniptm
    # print maniptm[2][0:3]
    
    
    # Now convert jointtm from string to array
    # Replace spaces with commas
    jointtm = jointtm.replace(" ",",")
    
    # and then evaluate the text to get an array
    jointtm = eval('['+jointtm+']')

    # debug
    #h_joint=misc.DrawAxes(env,MakeTransform(matrix([[jointtm[0],jointtm[3],jointtm[6]],[jointtm[1],jointtm[4],jointtm[7]],[jointtm[2],jointtm[5],jointtm[8]]]),matrix([[jointtm[9]],[jointtm[10]],[jointtm[11]]])))
    #h_manip=misc.DrawAxes(env,maniptm)

    
    #print "Joint Transformation Matrix"
    #print jointtm
    
    # Note:  For the PR2
    #
    # l_gripper_l_finger_joint --> Joint Index Number: 22
    # r_gripper_l_finger_joint --> Joint Index Number: 34
    # 
    # #################################################### 
    #
    rhanddofs = [34] # Right gripper links here
    rhandclosevals = [0.35] # What are the closed joint values for the right hand's links?--> Obtained experimentally
    rhandopenvals = [0.548] # What are the open joint values for the right hand's links? --> Obtained experimentally
    lhanddofs = [22] # Left gripper links here
    lhandclosevals = [0.35] # Same as rhandclosevals for the left gripper --> Obtained experimentally
    lhandopenvals = [0.548] # Same as rhandopenvals for the left gripper --> Obtained experimentally

    ##############################################################
    ## THIS IS WHERE THE FIRST BLOCK ENDS IN THE MATLAB VERSION ##
    ##############################################################
    
    # The following function call give you the flexibility of letting some joints be fixed. The joints you remove from the list won't be moving.
    #
    #robotid.SetActiveDOFs([0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38])
    
    # Joint numbers of the manipulators:
    #
    # The following two manipulators have their coordinate frames in the middle of their grippers
    #
    # rightarm_torso: 12,27,28,29,30,31,32,33,34
    # leftarm_torso: 12,15,16,17,18,19,20,21,22 
    # Combination of the two sets:
    #
    robotid.SetActiveDOFs([12,15,16,17,18,19,20,21,22,27,28,29,30,31,32,33,34])

    # Note: GetDOFValues() returns all 39 joints' values, whereas GetActiveDOFValues returns only the active joints' values
    #initconfig_candidate_1 = robotid.GetActiveDOFValues()
    
    # You can also use the joints' names to get their indices as in the following example
    #
    # jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
    # robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])

    # Or you can get a links transformation matrix and convert it to our format
    #
    # print "l_gripper_palm_link transform"
    # arg1=str(robotid.GetLinks()[50].GetTransform()).strip("[]").replace("\n"," ").replace("]  ["," ")
    # print arg1
    
    # The following two matrices define no rotation. Only translation
    # T0_LH1=matrix([[1,0,0,0.5],[0,1,0,0.3],[0,0,1,1.2],[0,0,0,1]])
    # T0_RH1=matrix([[1,0,0,0.5],[0,1,0,-0.3],[0,0,1,1.1],[0,0,0,1]])

    

    # The following two matrices define translation and rotation of 90 degrees around WORLD's x-axis and the new coordinate frame's Z-axis
    #
    # For rotation around X, left hand should be rotated in the positive direction, whereas right hand should be rotated in the negative
    # For rotation around Y both hands should be rotated in the negative direction
    # KEEP THESE JUST IN CASE UNTIL YOU DEBUG
    #T0_LH1 = MakeTransform(array(dot(rodrigues([pi/2,0,0]),rodrigues([0,0,pi/2]))),array([[0.51],[0.11],[0.8]]))
    #T0_RH1 = MakeTransform(array(dot(rodrigues([-pi/2,0,0]),rodrigues([0,0,pi/2]))),array([[0.51],[-0.11],[0.8]]))
    
    # Rotate the driving wheel's manipulator's coordinate frame 90 degrees around its Z-axis
    temp =  dot(maniptm,MakeTransform(matrix(rodrigues([0,0,pi/2])),transpose(matrix([0,0,0]))))

    # Rotate the new coordinate frame -90 degrees around its X-axis
    temp = dot(temp,MakeTransform(matrix(rodrigues([-pi/2,0,0])),transpose(matrix([0,0,0]))))
    
    # Translate the new coordinate frame -0.11 meters on its Z-axis and assign it to Left Hand
    Left_Hand_Point_In_Wheel_Coordinate_Frame = dot(temp,MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0,0,-0.11]))))
    
    T0_LH1 = Left_Hand_Point_In_Wheel_Coordinate_Frame
    

    # Rotate the driving wheel's manipulator's coordinate frame 90 degrees around its Z-axis
    temp =  dot(maniptm,MakeTransform(matrix(rodrigues([0,0,pi/2])),transpose(matrix([0,0,0]))))

    # Rotate the new coordinate frame +90 degrees around its X-axis
    temp = dot(temp,MakeTransform(matrix(rodrigues([pi/2,0,0])),transpose(matrix([0,0,0]))))
    
    # Translate the new coordinate frame -0.11 meters on its Z-axis and assign it to Left Hand
    Right_Hand_Point_In_Wheel_Coordinate_Frame = dot(temp,MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0,0,-0.11]))))
    
    T0_RH1 = Right_Hand_Point_In_Wheel_Coordinate_Frame
    

    # Note: if you want to show the coordinate frames, uncomment following lines
    #
    # h1=misc.DrawAxes(env,T0_LH1)
    # h2=misc.DrawAxes(env,T0_RH1)

    # probs[0] is equivalent to "probs.cbirrt" in the matlab version
    # Stringify and combine the rotation and translation matrix to obtain a transformation matrix for the left hand
    arg1 = str(GetRot(T0_LH1)).strip("[]")+str(GetTrans(T0_LH1)).strip("[]")
    arg1 = arg1.replace("\n"," ")
    # Stringify and combine the rotation and translation matrix to obtain a transformation matrix for the right hand
    arg2 = str(GetRot(T0_RH1)).strip("[]")+str(GetTrans(T0_RH1)).strip("[]")
    arg2 = arg2.replace("\n"," ")        

    # Note: It turns out the PR2 has 8 manipulators in OpenRAVE
    #
    # maniptm 6: leftarm_torso (this coordinate frame is in the middle of the gripper)
    # maniptm 4: rightarm_torso (this coordinate frame is in the middle of the gripper)
    #
    # 
    startik = probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 6 '+arg1+' maniptm 4 '+arg2)
    print "Got startik:"
    print startik
    print "------"
    
    # Matlab version has the following line but in Python it doesn't seem to do anything,
    # Mostly beause probs[0].SendCommand makes the manipulators move already.
    
    robotid.SetActiveDOFValues(str2num(startik)) # Note: startik: T0_LH1 and T0_RH1
    print "go to startik"
    sys.stdin.readline()

    # Note: GetDOFValues() returns all 39 joints' values, whereas GetActiveDOFValues returns only the active joints' values

    # THIS IS THE END OF THE SECOND BLOCK IN MATLAB VERSION

    # Left hand is evaluated first, so need to make right hand relative to crank
    
    cranklinks = crankid.GetLinks();
    
    # debug
    # print cranklinks
    # for cl in range(len(cranklinks)):
    #     print cranklinks[cl].GetName()
    #     print cranklinks[cl].GetTransform()
    #     print type(cranklinks[cl].GetTransform())

    # cranklinks[0]: pole
    # cranklinks[1]: crank
    #
    # cranklinks[1].GetTransform()[0:3,0:3] --> This strips the rotation matrix from crank's transform
    # cranklinks[1].GetTransform()[0:3,3] --> This strips the translation vector from crank's transform
        
    # T0_crankcrank = MakeTransform(matrix(cranklinks[1].GetTransform()[0:3,0:3]),matrix(cranklinks[1].GetTransform()[0:3,3]))    
    
    #T0_crankcrank = crankid.GetLinks()[1].GetTransform()
    T0_crankcrank = crankid.GetManipulators()[0].GetTransform()

    #T0_w0L = MakeTransform(matrix([[1],[0],[0],[0],[1],[0],[0],[0],[1]]),matrix([[jointtm[9]],[jointtm[10]],[jointtm[11]]]))
    #T0_w0L = matrix([[1,0,0,0.5],[0,1,0,0.1],[0,0,1,0.8],[0,0,0,1]])

    T0_w0L = eye(4)
    T0_w0R = MakeTransform(rodrigues([0,0.4,0]),matrix([[jointtm[9]],[jointtm[10]],[jointtm[11]]]))
    #T0_w0R = MakeTransform(matrix([[jointtm[0],jointtm[3],jointtm[6]],[jointtm[1],jointtm[4],jointtm[7]],[jointtm[2],jointtm[5],jointtm[8]]]),matrix([[jointtm[9]],[jointtm[10]],[jointtm[11]]]))

    crank_rot = pi/3
    TSRChainMimicDOF = 1

    Tcrank_rot = MakeTransform(matrix(rodrigues([crank_rot,0,0])),transpose(matrix([0,0,0]))) # For the right gripper
    Tcrank_rot2 = MakeTransform(matrix(rodrigues([0,0,crank_rot])),transpose(matrix([0,0,0]))); # For the left gripper
    
    T0_cranknew = dot(T0_w0R,Tcrank_rot)
    
    temp = dot(linalg.inv(T0_w0R),T0_RH1)
    T0_RH2 = dot(T0_cranknew,temp);

    temp = dot(linalg.inv(T0_crankcrank),T0_LH1)
    temp = dot(Tcrank_rot2,temp)
    T0_LH2 = dot(T0_crankcrank,temp);

    # Note: Following two lines 
    #h3=misc.DrawAxes(env,T0_LH2)
    #h4=misc.DrawAxes(env,T0_RH2)

    arg1 = str(GetRot(T0_LH2)).strip("[]")+str(GetTrans(T0_LH2)).strip("[]")
    arg2 = str(GetRot(T0_RH2)).strip("[]")+str(GetTrans(T0_RH2)).strip("[]")

    #robotid.SetDOFValues(rhandclosevals,rhanddofs)
    #robotid.SetDOFValues(lhandclosevals,lhanddofs)
    crankid.SetDOFValues([crank_rot],[crankjointind])

    # Get the goal inverse kinematics
    goalik = probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 6 '+arg1+' maniptm 4 '+arg2)
    print "Got goalik:"
    print goalik
    print "------"
    robotid.SetActiveDOFValues(str2num(goalik)) # Note: goalik is T0_LH2 and T0_RH2
    print "go to goalik"
    sys.stdin.readline()
    # Let the TSR Magic Happen
    
    crankid.SetDOFValues([crank_rot],[crankjointind])

    #########################################################
    # This is the end of the second block in Matlab version #
    #########################################################
    crankid.SetDOFValues([crankjointind])
    
    # Now lets put the PR2 back in its initial configuration
    # initconfig is T0_LH1 and T0_RH1
    robotid.SetActiveDOFValues(str2num(startik))
    
    # SerializeTSR(manipindex,bodyandlink,T0_w,Tw_e,Bw)
    # Input:
    # manipindex (int): the 0-indexed index of the robot's manipulator
    # bodyandlink (str): body and link which is used as the 0 frame. Format 'body_name link_name'. To use world frame, specify 'NULL'
    # T0_w (double 4x4): transform matrix of the TSR's reference frame relative to the 0 frame
    # Tw_e (double 4x4): transform matrix of the TSR's offset frame relative the w frame
    # Bw (double 1x12): bounds in x y z roll pitch yaw. Format: [x_min x_max y_min y_max...]
    
    
    # The list of what the heck is going on here:
    #
    # T0_w0L defines where the crank's center is in the world coord frame (should this be the grasping point of crank with the left gripper?)
    #
    # T0_LH1 defines where the left gripper is in the world coord frame
    #
    # Tw0_eL defines where the left gripper is wrt to crank
    # 
    # Bw0L defines the boundaries of allowable translation and rotation: as can be seen, we don't allow any translation, but rotation around the x-axis (min 0, max pi).
    
    #Tw0_eL = dot(linalg.inv(T0_w0L),T0_LH1)
    # print "Tw0_eL"
    # print Tw0_eL
    #Bw0L = matrix([0,0,0,0,0,0,0,pi,0,0,0,0])

    # debug --> The following Ts define the initial grasp where the robots hands are at 9:15 o'clock position around the valve
    T0_w0L = MakeTransform(matrix(rodrigues([0.4,0,0])),transpose(matrix([0,0,0])))
    
    #T0_w0R = MakeTransform(matrix(rodrigues([0,0,0])),transpose(matrix([0.5,0,0.8])))
    T0_w0R = MakeTransform(rodrigues([0,0.4,0]),matrix([[jointtm[9]],[jointtm[10]],[jointtm[11]]]))
   

    #Tw0_eL = MakeTransform(matrix(dot(rodrigues([0,pi,0]),rodrigues([pi/2,0,0]))),transpose(matrix([0,0.11,0])))
    Tw0_eL = dot(linalg.inv(T0_crankcrank),T0_LH1)
    #Tw0_eR = MakeTransform(matrix(dot(rodrigues([-pi/2,0,0]),rodrigues([0,0,-pi/2]))),transpose(matrix([0,-0.11,0])))
    Tw0_eR = dot(linalg.inv(T0_w0R),T0_RH1)
    
    Bw0L = matrix([0,0,0,0,0,0,0,0,0,0,0,0])
    Bw0R = matrix([0,0,0,0,0,0,-pi,pi,0,0,0,0])    

    #h5=misc.DrawAxes(env,T0_w0L,0.05)
    #h6=misc.DrawAxes(env,T0_LH1)
    #h71=misc.DrawAxes(env,Tw0_eL,0.05)

    #h5=misc.DrawAxes(env,T0_w0R,0.05)
    #h6=misc.DrawAxes(env,T0_RH1,0.3)
    #h72=misc.DrawAxes(env,Tw0_eR,0.05)

    #h20 = misc.DrawAxes(env,T0_crankcrank)

    print "Look at the axes"
    sys.stdin.readline()
    
    
    # defined relative to crank
    #
    # T0_w0R defines the crank in the world coord frame (should this be the grasping point of crank with the right gripper?)
    #
    # Tw0_eR defines where the right gripper is wrt crank
    #T0_w0R = matrix(eye(4))
    #T0_w0R = matrix([[1,0,0,0.5],[0,1,0,-0.1],[0,0,1,1.2],[0,0,0,1]])
    #Tw0_eR = dot(linalg.inv(T0_crankcrank),T0_RH1)
    #print "Tw0_eR"
    #print Tw0_eR
    #Bw0R = matrix([0,0,0,0,0,0,0,0,0,0,0,0]) #note: in frame of crank

    #h8=misc.DrawAxes(env,T0_crankcrank)
    #h8=misc.DrawAxes(env,T0_w0R)
    #h9=misc.DrawAxes(env,T0_RH1)
    #h10=misc.DrawAxes(env,Tw0_eR)
    
    TSRstring1 = SerializeTSR(4,'NULL',T0_w0R,Tw0_eR,Bw0R) # TSR for the right gripper
    TSRstring2 = SerializeTSR(7,'crank crank',T0_w0L,Tw0_eL,Bw0L) # TSR for the left gripper

    #TSRstring3 = SerializeTSR(4,'NULL',)
    
    
    TSRChainString = SerializeTSRChain(0,0,1,1,TSRstring1,'crank',matrix([crankjointind]))+' '+SerializeTSRChain(0,0,1,1,TSRstring2,'NULL',[])

    #TSRChainString = SerializeTSRChain(0,0,1,1,TSRstring2,'NULL',[])
    

    print TSRChainString
   

    
    # Clean the trajectory files so that we don't execute an old trajectory
    for i in range(2):
        try:
            print "Removing movetraj"+str(i)+".txt"
            os.remove("movetraj"+str(i)+".txt")
        except OSError, e:
            print e    
   
    
    # Set the current goal to RH1 and LH1        
    goaljoints = str2num(startik);
    #print goaljoints
    print "Press enter to continue 2.1.."
    sys.stdin.readline()
    
    robotid.SetActiveDOFValues(initconfig)
    crankid.SetDOFValues([0],[crankjointind])

    #print goaljoints
    print "Press enter to continue 2.2..."
    sys.stdin.readline()

    # Create a trajectory (from initial configuration --> to LH1 RH1 configuration)

    TSRString_G1 = SerializeTSR(4,'NULL',T0_RH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    TSRString_G2 = SerializeTSR(7,'NULL',T0_LH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    TSRChainString_G = SerializeTSRChain(0,0,1,1,TSRString_G1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString_G2,'NULL',[])

    try:
        answer = probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_G))
        print "runcbirrt successful"
        print answer
    except openrave_exception, e:
        print "Cannot send command runcbirrt"
        print e

    # Rename the file so that we can keep the data w/o overwriting it with a new trajectory
    try:
        os.rename("cmovetraj.txt","movetraj0.txt")
        
        print "Executing trajectory 0"
        try:
            answer= probs[0].SendCommand('traj movetraj0.txt');
            # debug
            print "traj call successful"
            print answer 
        except openrave_exception, e:
            print e
        
        robotid.WaitForController(0)
        sys.stdin.readline()
    except OSError, e:
        print e
     
    
    #robotid.SetDOFValues(rhandclosevals,rhanddofs)
    #robotid.SetDOFValues(lhandclosevals,lhanddofs)


    goaljoints = str2num(goalik)
    print goaljoints
    print "Press enter to continue 3.1.."
    sys.stdin.readline()

    goaljoints = append(goaljoints,0)
    print goaljoints
    print "Press enter to continue 3.2..."
    sys.stdin.readline()

    # Crate a trajectory (from LH1 RH1 configuration --> to LH2 RH2 configuration)
    try:
        answer = probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString))
        print "runcbirrt successful"
        print answer
    except openrave_exception, e:
        print "Cannot send command runcbirrt"
        print e

    sys.stdin.readline()
    # Rename the file so that we can keep the data w/o overwriting it with a new trajectory
    try:
        os.rename("cmovetraj.txt","movetraj1.txt")
    except OSError, e:
        print e
    
    robotid.SetActiveDOFValues(initconfig)
    
    while True:
        
        
        print "Press enter to execute trajectory 0"
        sys.stdin.readline()        

        try:
            answer1= probs[0].SendCommand('traj movetraj0.txt')
            # debug
            # print "traj call successful"
            # print answer1
            # print answer2
        except openrave_exception, e:
            print e

        robotid.WaitForController(0)
        sys.stdin.readline()
        
        print "Press enter to execute trajectory 1"
        sys.stdin.readline()
        try:
            answer1= probs[0].SendCommand('traj movetraj1.txt')
            answer2= probs[1].SendCommand('traj movetraj1.txt')
            # debug
            # print "traj call successful"
            # print answer1
            # print answer2
        except openrave_exception, e:
            print e

        robotid.WaitForController(0)
        print "Press enter to restart"
        sys.stdin.readline()
        
    print "press enter to exit"
    sys.stdin.readline()




if __name__ == "__main__":
    run()
