%AS OF OPENRAVE r1315, the collision-checking doesn't work during the
%planning, probably has to do with the weird models of the sub room

%tests TSR chaining
clear all;
close all;
orEnvLoadScene('',1)
robotid = orEnvCreateRobot('HRP3','models/hrp3/hrp3d_rightleg.robot.xml');
orEnvCreateKinBody('subroom','models/subroom.kinbody.xml');
crankid = orEnvCreateRobot('crank','models/crank.robot.xml');
xoffset = 0.07;
orBodySetTransform(robotid,[0.9969    0.0783         0   -0.0783    0.9969         0         0         0    1.0000      -1.5602+xoffset    2.5666    0.1545]);
orBodySetTransform(crankid,[reshape(rodrigues([0 pi/2 0]),1,9)';[-0.9566+xoffset    2.7186    1.2225]'])


orEnvSetOptions('debug 3')
orEnvSetOptions('publishanytime 0');

probs.cbirrt = orEnvCreateProblem('CBiRRT','HRP3');
probs.crankmover = orEnvCreateProblem('CBiRRT','crank',0);

orProblemSendCommand('SetCamView -0.00328895 0.00649324 0.768259 -0.640098 -1.30788 4.99505 1.22135',probs.cbirrt)
manips = orRobotGetManipulators(robotid);

jointdofs = 0:41;
activedofs = setdiff(jointdofs,[manips{1}.armjoints,manips{4}.joints,manips{5}.joints]);

%dofvals = [flipud([0.0093;-0.1802;-0.5118;0.9943;-0.4307;0.1804]);-0.0093;0.1801;-0.5129;0.9965;-0.4319;-0.1804;0;0;0;0;0;0;0;-0.8;0;0;0;0;0;0;0;0;0;0;0;0;-0.8;0;0;0;0;0;0;0;0;0;]';
dofvals = [0.1804;-0.4307;0.9943;-0.5118;-0.1802;0.0093;-0.0093;0.1801;-0.5129;0.9965;-0.4319;-0.1804;0;0;0;0;0;0;0;-0.8;0;0;0;0;0;0;0;0.4;0.5;0;0;0;-0.8;0;0;0;0;0;0;0;-0.4;-0.5;];
orRobotSetDOFValues(robotid,dofvals,jointdofs);
%orRobotSetDOFValues(robotid,-[0 0 -0.0 0 -0.4 -0.5],manips{5}.joints);
%orRobotSetDOFValues(robotid,[0 0 -0.0 0 -0.4 -0.5],manips{4}.joints);

normalsmoothingitrs = 150;
fastsmoothingitrs = 20;

links = orBodyGetLinks(robotid);

Tee = cell(1,4);

for i = 1:4
    Tlink = MakeTransform(links(1:9,manips{i+1}.eelink+1),links(10:12,manips{i+1}.eelink+1));
    Tgrasp = [manips{i+1}.Tgrasp;[0 0 0 1]];

    Tee{i} = Tlink*Tgrasp;
end


crankjointind = 0;
%jointaxis = str2num(orProblemSendCommand(['GetJointAxis name kitchen jointind fridgejointind'],probs.tcbirrt));
jointtm = str2num(orProblemSendCommand(['GetJointTransform name crank jointind ' num2str(crankjointind)],probs.cbirrt));


handrot = rodrigues([0 -pi/2 0]);
transoffset = [0 0.135 0];

Rhandtrans = jointtm(10:12)-[0.05 0 0];
Lhandtrans = jointtm(10:12)-[0.05 0 0];
T0_RH1 = MakeTransform(handrot,Rhandtrans'-transoffset');
T0_LH1 = MakeTransform(handrot,Lhandtrans'+transoffset');


cogtarg = [0 0 0];

%orRobotSetActiveDOFs(robotid,setdiff(activedofs,[manips{2}.armjoints]));
orRobotSetActiveDOFs(robotid,activedofs);

startik = orProblemSendCommand(['DoGeneralIK exec supportlinks 2 RAnkleR LAnkleR ' ' movecog ' num2str(cogtarg)  ' nummanips 3 ' ' maniptm 3 ' num2str([GetRot(T0_LH1),GetTrans(T0_LH1)]) ' maniptm 4 ' num2str([GetRot(T0_RH1),GetTrans(T0_RH1)]) ' maniptm 1 ' num2str([GetRot(Tee{1}),GetTrans(Tee{1})])],probs.cbirrt);
orRobotSetDOFValues(robotid,str2num(startik));



%left hand is evaluated first so need to make right hand relative to crank!

T0_w0L = MakeTransform([1 0 0 0 1 0 0 0 1]',jointtm(10:12));
Tw0_eL = inv(T0_w0L)*T0_LH1;
Bw0L = [0 0   0 0   0 0  0 pi   0 0   0 0];
TSRstring1 = SerializeTSR(3,'NULL',T0_w0L,Tw0_eL,Bw0L);


cranklinks = orBodyGetLinks(crankid);
T0_crankcrank = MakeTransform(cranklinks(1:9,2),cranklinks(10:12,2));
%defined relative to crank
T0_w0R = eye(4);
Tw0_eR = inv(T0_crankcrank)*T0_RH1;
Bw0R = [0 0   0 0   0 0   0 0   0 0   0 0]; %note: in frame of crank
TSRstring2 = SerializeTSR(4,'crank crank',T0_w0R,Tw0_eR,Bw0R);


TSRstring3 = SerializeTSR(1,'NULL',Tee{1},eye(4),zeros(1,12));

TSRChainString = [SerializeTSRChain(0,0,1,1,TSRstring1,'crank',crankjointind) ' ' SerializeTSRChain(0,0,1,1,TSRstring2,'NULL',[]) ' ' SerializeTSRChain(0,0,1,1,TSRstring3,'NULL',[])];
%TSRChainString = [SerializeTSRChain(0,0,1,1,TSRstring1,'crank',crankjointind) ' ' SerializeTSRChain(0,0,1,1,TSRstring3,'NULL',[])];

crank_rot = pi/2;
TSRChainMimicDOF = 1;


Tcrank_rot = MakeTransform(rodrigues([crank_rot 0 0]),[0 0 0]');
Tcrank_rot2 = MakeTransform(rodrigues([0 0 crank_rot]),[0 0 0]');

T0_cranknew = T0_w0L*Tcrank_rot;

T0_LH2 = T0_cranknew*inv(T0_w0L)*T0_LH1;
T0_RH2 = T0_crankcrank*Tcrank_rot2*inv(T0_crankcrank)*T0_RH1;

goalik = orProblemSendCommand(['DoGeneralIK exec supportlinks 2 RAnkleR LAnkleR ' ' movecog ' num2str(cogtarg)  ' nummanips 3 ' ' maniptm 3 ' num2str([GetRot(T0_LH2),GetTrans(T0_LH2)]) ' maniptm 4 ' num2str([GetRot(T0_RH2),GetTrans(T0_RH2)]) ' maniptm 1 ' num2str([GetRot(Tee{1}),GetTrans(Tee{1})])],probs.cbirrt);
orRobotSetDOFValues(robotid,str2num(goalik));


orRobotSetDOFValues(crankid,0,crankjointind)
%orRobotSetDOFValues(robotid,str2num(startik));
orRobotSetDOFValues(robotid,dofvals,jointdofs);
TSRChainStringFootOnly = SerializeTSRChain(0,0,1,1,TSRstring3,'NULL',[]);
goaljoints = str2num(startik);
orProblemSendCommand(['RunCBiRRT smoothingitrs ' num2str(normalsmoothingitrs) ' jointgoals '  num2str(numel(goaljoints)) ' ' num2str(goaljoints) ' supportlinks 2 RAnkleR LAnkleR ' TSRChainStringFootOnly],probs.cbirrt);
!mv ../cmovetraj.txt ../movetraj0.txt
orProblemSendCommand(['traj movetraj0.txt'],probs.cbirrt);
orEnvWait(robotid);

goaljoints = [str2num(goalik), 0*ones(1,TSRChainMimicDOF)];
orProblemSendCommand(['RunCBiRRT smoothingitrs ' num2str(fastsmoothingitrs) ' jointgoals '  num2str(numel(goaljoints)) ' ' num2str(goaljoints) ' supportlinks 2 RAnkleR LAnkleR ' TSRChainString],probs.cbirrt);

!mv ../cmovetraj.txt ../movetraj1.txt
orProblemSendCommand(['traj movetraj1.txt'],probs.cbirrt);
orProblemSendCommand(['traj movetraj1.txt'],probs.crankmover);
orEnvWait(robotid);

goaljoints = str2num(startik);
orProblemSendCommand(['RunCBiRRT smoothingitrs ' num2str(normalsmoothingitrs) ' jointgoals '  num2str(numel(goaljoints)) ' ' num2str(goaljoints) ' supportlinks 2 RAnkleR LAnkleR ' TSRChainStringFootOnly],probs.cbirrt);

!mv ../cmovetraj.txt ../movetraj2.txt
orProblemSendCommand(['traj movetraj2.txt'],probs.cbirrt);
orEnvWait(robotid);


orBodySetJointValues(crankid,pi/2)
cranklinks = orBodyGetLinks(crankid);
T0_crankcrank = MakeTransform(cranklinks(1:9,2),cranklinks(10:12,2));
%defined relative to crank
T0_w0R = eye(4);
Tw0_eR = inv(T0_crankcrank)*T0_RH1;
Bw0R = [0 0   0 0   0 0   0 0   0 0   0 0]; %note: in frame of crank
TSRstring2 = SerializeTSR(4,'crank crank',T0_w0R,Tw0_eR,Bw0R);


TSRstring3 = SerializeTSR(1,'NULL',Tee{1},eye(4),zeros(1,12));

TSRChainString = [SerializeTSRChain(0,0,1,1,TSRstring1,'crank',crankjointind) ' ' SerializeTSRChain(0,0,1,1,TSRstring2,'NULL',[]) ' ' SerializeTSRChain(0,0,1,1,TSRstring3,'NULL',[])];
%TSRChainString = [SerializeTSRChain(0,0,1,1,TSRstring1,'crank',crankjointind) ' ' SerializeTSRChain(0,0,1,1,TSRstring3,'NULL',[])];

crank_rot = pi/2;
TSRChainMimicDOF = 1;


Tcrank_rot = MakeTransform(rodrigues([crank_rot 0 0]),[0 0 0]');
Tcrank_rot2 = MakeTransform(rodrigues([0 0 crank_rot]),[0 0 0]');

T0_cranknew = T0_w0L*Tcrank_rot;

T0_LH2 = T0_cranknew*inv(T0_w0L)*T0_LH1;
T0_RH2 = T0_crankcrank*Tcrank_rot2*inv(T0_crankcrank)*T0_RH1;

orRobotSetDOFValues(robotid,str2num(startik));
goalik = orProblemSendCommand(['DoGeneralIK exec supportlinks 2 RAnkleR LAnkleR ' ' movecog ' num2str(cogtarg)  ' nummanips 3 ' ' maniptm 3 ' num2str([GetRot(T0_LH2),GetTrans(T0_LH2)]) ' maniptm 4 ' num2str([GetRot(T0_RH2),GetTrans(T0_RH2)]) ' maniptm 1 ' num2str([GetRot(Tee{1}),GetTrans(Tee{1})])],probs.cbirrt);
orRobotSetDOFValues(robotid,str2num(goalik));


orRobotSetDOFValues(robotid,str2num(startik));
%TSRChainMimicDOF = 0;

goaljoints = [str2num(goalik), pi/2*ones(1,TSRChainMimicDOF)];
orProblemSendCommand(['RunCBiRRT smoothingitrs ' num2str(fastsmoothingitrs) ' jointgoals '  num2str(numel(goaljoints)) ' ' num2str(goaljoints) ' supportlinks 2 RAnkleR LAnkleR ' TSRChainString],probs.cbirrt);

!mv ../cmovetraj.txt ../movetraj3.txt
orProblemSendCommand(['traj movetraj3.txt'],probs.cbirrt);
orProblemSendCommand(['traj movetraj3.txt'],probs.crankmover);
orEnvWait(robotid);

goaljoints = dofvals(activedofs+1)';
orProblemSendCommand(['RunCBiRRT smoothingitrs ' num2str(normalsmoothingitrs) ' jointgoals '  num2str(numel(goaljoints)) ' ' num2str(goaljoints) ' supportlinks 2 RAnkleR LAnkleR ' TSRChainStringFootOnly],probs.cbirrt);
!mv ../cmovetraj.txt ../movetraj4.txt
orProblemSendCommand(['traj movetraj4.txt'],probs.cbirrt);
orEnvWait(robotid);