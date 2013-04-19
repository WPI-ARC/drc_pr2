# Ben Suay, RAIL
# November 2012
# Worcester Polytechnic Institute
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
    env.Load('../../models/driving_wheel.robot.xml')
    robotid = env.ReadRobotURI('robots/pr2-beta-static.zae')
    crankid = env.GetRobots()[0]

    crankid.SetTransform(array(MakeTransform(matrix(dot(rodrigues([0,0,pi/2]),rodrigues([pi/2,0,0]))), transpose(matrix([0.5,0,0.8128])))))

    env.Add(robotid)
    env.SetViewer('qtcoin')

    probs_cbirrt = RaveCreateModule(env,'CBiRRT')
    probs_crankmover = RaveCreateModule(env,'CBiRRT')

    manips = robotid.GetManipulators()
    crankmanip = crankid.GetManipulators()

    try:
        env.AddModule(probs_cbirrt,'pr2')
        env.AddModule(probs_crankmover,'crank')
    except openrave_exception, e:
        print e

    print "Getting Loaded Problems"
    probs = env.GetLoadedProblems()
    print probs[0] # --> probs_cbirrt
    print probs[1] # --> probs_crankmover

    robotid.SetActiveDOFs([15,16,17,18,19,20,21,27,28,29,30,31,32,33])

    robotid.SetDOFValues([-0.85,-1.00,-1.57,-1.00,-1.00,0.548],[27,29,30,32,33,34])

    robotid.SetDOFValues([0.85,1.00,-1.57,-1.00,1.00,0.548],[15,17,18,20,21,22])
   
    robotid.SetActiveDOFs([15,16,17,18,19,20,21,22,27,28,29,30,31,32,33,34])

    initconfig = robotid.GetActiveDOFValues()
    
    print "Press enter to continue 1..."
    sys.stdin.readline()
    
    links = robotid.GetLinks()
        
    crankjointind = 0

    jointtm = probs[0].SendCommand('GetJointTransform name crank jointind '+str(crankjointind))
    
    maniptm = crankid.GetManipulators()[0].GetTransform()

    jointtm = jointtm.replace(" ",",")
    
    jointtm = eval('['+jointtm+']')

    rhanddofs = [34] # Right gripper links here
    rhandclosevals = [0.35] # What are the closed joint values for the right hand's links?--> Obtained experimentally
    rhandopenvals = [0.548] # What are the open joint values for the right hand's links? --> Obtained experimentally
    lhanddofs = [22] # Left gripper links here
    lhandclosevals = [0.35] # Same as rhandclosevals for the left gripper --> Obtained experimentally
    lhandopenvals = [0.548] # Same as rhandopenvals for the left gripper --> Obtained experimentally

    robotid.SetActiveDOFs([15,16,17,18,19,20,21,22,27,28,29,30,31,32,33,34])

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
    
    arg1 = str(GetRot(T0_LH1)).strip("[]")+str(GetTrans(T0_LH1)).strip("[]")
    arg1 = arg1.replace("\n"," ")
   
    arg2 = str(GetRot(T0_RH1)).strip("[]")+str(GetTrans(T0_RH1)).strip("[]")
    arg2 = arg2.replace("\n"," ")        

   
    startik = probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 6 '+arg1+' maniptm 4 '+arg2)

    print "Got startik:"
    print startik
    print "------"
    
    robotid.SetActiveDOFValues(str2num(startik)) # Note: startik: T0_LH1 and T0_RH1
    print "go to startik"
    sys.stdin.readline()
    
    cranklinks = crankid.GetLinks();
    
    T0_crankcrank = crankid.GetManipulators()[0].GetTransform()

    T0_w0L = eye(4)
    T0_w0R = MakeTransform(rodrigues([0,0.4,0]),matrix([[jointtm[9]],[jointtm[10]],[jointtm[11]]]))

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

    arg1 = str(GetRot(T0_LH2)).strip("[]")+str(GetTrans(T0_LH2)).strip("[]")
    arg2 = str(GetRot(T0_RH2)).strip("[]")+str(GetTrans(T0_RH2)).strip("[]")

    crankid.SetDOFValues([crank_rot],[crankjointind])

    goalik = probs[0].SendCommand('DoGeneralIK exec nummanips 2 maniptm 6 '+arg1+' maniptm 4 '+arg2)
    print "Got goalik:"
    print goalik
    print "------"
    robotid.SetActiveDOFValues(str2num(goalik)) # Note: goalik is T0_LH2 and T0_RH2
    print "go to goalik"
    sys.stdin.readline()
    
    crankid.SetDOFValues([crank_rot],[crankjointind])

    crankid.SetDOFValues([crankjointind])
    
    robotid.SetActiveDOFValues(str2num(startik))
    
    T0_w0L = MakeTransform(matrix(rodrigues([0.4,0,0])),transpose(matrix([0,0,0])))
    
    T0_w0R = MakeTransform(rodrigues([0,0.4,0]),matrix([[jointtm[9]],[jointtm[10]],[jointtm[11]]]))
   
    Tw0_eL = dot(linalg.inv(T0_crankcrank),T0_LH1)

    Tw0_eR = dot(linalg.inv(T0_w0R),T0_RH1)
    
    Bw0L = matrix([0,0,0,0,0,0,0,0,0,0,0,0])
    Bw0R = matrix([0,0,0,0,0,0,-pi,pi,0,0,0,0])    

    print "Look at the axes"
    sys.stdin.readline()
    
    TSRstring1 = SerializeTSR(4,'NULL',T0_w0R,Tw0_eR,Bw0R) # TSR for the right gripper
    TSRstring2 = SerializeTSR(6,'crank crank',T0_w0L,Tw0_eL,Bw0L) # TSR for the left gripper    
    
    TSRChainString = SerializeTSRChain(0,0,1,1,TSRstring1,'crank',matrix([crankjointind]))+' '+SerializeTSRChain(0,0,1,1,TSRstring2,'NULL',[])   

    for i in range(2):
        try:
            print "Removing movetraj"+str(i)+".txt"
            os.remove("movetraj"+str(i)+".txt")
        except OSError, e:
            print e    
            
    ################################################################################
    goaljoints = str2num(startik);

    print "Press enter to continue 2.1.."
    sys.stdin.readline()
    
    robotid.SetActiveDOFValues(initconfig)
    crankid.SetDOFValues([0],[crankjointind])

    print "Press enter to continue 2.2..."
    sys.stdin.readline()

    TSRString_G1 = SerializeTSR(4,'NULL',T0_RH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    TSRString_G2 = SerializeTSR(6,'NULL',T0_LH1,eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    TSRChainString_G = SerializeTSRChain(0,0,1,1,TSRString_G1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString_G2,'NULL',[])

    try:
        answer = probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_G))
        print "runcbirrt successful"
        print answer
    except openrave_exception, e:
        print "Cannot send command runcbirrt"
        print e

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
     

    ########################################################################
    goaljoints = str2num(goalik)
    print goaljoints
    print "Press enter to continue 3.1.."
    sys.stdin.readline()

    goaljoints = append(goaljoints,0)
    print goaljoints
    print "Press enter to continue 3.2..."
    sys.stdin.readline()

    try:
        answer = probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString))
        print "runcbirrt successful"
        print answer
    except openrave_exception, e:
        print "Cannot send command runcbirrt"
        print e

    sys.stdin.readline()

    try:
        os.rename("cmovetraj.txt","movetraj1.txt")
        try:
            answer= probs[0].SendCommand('traj movetraj1.txt');
            # debug
            print "traj call successful"
            print answer 
        except openrave_exception, e:
            print e
        
        robotid.WaitForController(0)
        sys.stdin.readline()
    except OSError, e:
        print e          

    ##########################################################################
        
    goaljoints = str2num(startik)
    print "Press enter to continue 4.1.."
    sys.stdin.readline()

    goaljoints = append(goaljoints,0)
    print "Press enter to continue 4.2..."
    sys.stdin.readline()
    
    crankid.SetDOFValues([0],[crankjointind])

    try:
        answer = probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString))
        print "runcbirrt successful"
        print answer
    except openrave_exception, e:
        print "Cannot send command runcbirrt"
        print e

    sys.stdin.readline()

    try:
        os.rename("cmovetraj.txt","movetraj2.txt")
        try:
            answer= probs[0].SendCommand('traj movetraj2.txt');
            # debug
            print "traj call successful"
            print answer 
        except openrave_exception, e:
            print e
        
        robotid.WaitForController(0)
        sys.stdin.readline()
    except OSError, e:
        print e          

    ########################################################################

    goaljoints = initconfig

    robotid.SetActiveDOFValues(initconfig)

    TSRString_I1 = SerializeTSR(4,'NULL',robotid.GetManipulators()[4].GetTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    TSRString_I2 = SerializeTSR(6,'NULL',robotid.GetManipulators()[6].GetTransform(),eye(4),matrix([0,0,0,0,0,0,0,0,0,0,0,0]))
    TSRChainString_I = SerializeTSRChain(0,0,1,1,TSRString_I1,'NULL',[])+' '+SerializeTSRChain(0,0,1,1,TSRString_I2,'NULL',[])
    
    sys.stdin.readline()
    
    try:
        answer = probs[0].SendCommand('RunCBiRRT jointgoals %d %s %s'%(len(goaljoints),Serialize1DMatrix(matrix(goaljoints)),TSRChainString_I))
        print "runcbirrt successful"
        print answer
    except openrave_exception, e:
        print "Cannot send command runcbirrt"
        print e

    try:
        os.rename("cmovetraj.txt","movetraj3.txt")
        
        print "Executing trajectory 3"
        try:
            answer= probs[0].SendCommand('traj movetraj3.txt');
            # debug
            print "traj call successful"
            print answer 
        except openrave_exception, e:
            print e
        
        robotid.WaitForController(0)
        sys.stdin.readline()
    except OSError, e:
        print e


    ########################################################################
    robotid.SetActiveDOFValues(initconfig)
    
    while True:
        
        
        print "Press enter to execute trajectory 0"
        sys.stdin.readline()        

        try:
            answer1= probs[0].SendCommand('traj movetraj0.txt')
        except openrave_exception, e:
            print e

        robotid.WaitForController(0)
        sys.stdin.readline()

        ###################
        
        print "Press enter to execute trajectory 1"
        sys.stdin.readline()
        try:
            answer1= probs[0].SendCommand('traj movetraj1.txt')
            answer2= probs[1].SendCommand('traj movetraj1.txt')
        except openrave_exception, e:
            print e

        robotid.WaitForController(0)
        print "Press enter to restart"
        sys.stdin.readline()

        ###################
        
        print "Press enter to execute trajectory 2"
        sys.stdin.readline()
        try:
            answer1= probs[0].SendCommand('traj movetraj2.txt')
            answer2= probs[1].SendCommand('traj movetraj2.txt')
        except openrave_exception, e:
            print e

        robotid.WaitForController(0)
        print "Press enter to restart"
        sys.stdin.readline()

        ###################
        
        print "Press enter to execute trajectory 1"
        sys.stdin.readline()
        try:
            answer1= probs[0].SendCommand('traj movetraj1.txt')
            answer2= probs[1].SendCommand('traj movetraj1.txt')
        except openrave_exception, e:
            print e

        robotid.WaitForController(0)
        print "Press enter to restart"
        sys.stdin.readline()

        ###################
        
        print "Press enter to execute trajectory 3"
        sys.stdin.readline()
        try:
            answer1= probs[0].SendCommand('traj movetraj3.txt')
            answer2= probs[1].SendCommand('traj movetraj3.txt')
        except openrave_exception, e:
            print e

        robotid.WaitForController(0)
        print "Press enter to restart"
        sys.stdin.readline()
        
    print "press enter to exit"
    sys.stdin.readline()




if __name__ == "__main__":
    run()
