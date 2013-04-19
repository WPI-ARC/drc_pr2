clear all;
close all;
orEnvLoadScene('',1)


normalsmoothingitrs = 150;
fastsmoothingitrs = 20;


robottype = 1; %set which robot you are using here

if(robottype == 1)
    robotname = 'huboplus/rlhuboplus_mit';
elseif(robottype == 2) 
    robotname = 'jaemi/rljaemiHubo';
end


robotid = orEnvCreateRobot('Hubo',['../../../openHubo/' robotname '.robot.xml']);
crankid = orEnvCreateRobot('crank','../../models/driving_wheel.robot.xml');

if(robottype == 1)    
    orBodySetTransform(crankid,[reshape(rodrigues([0 0 pi/2])*rodrigues([pi/2 0 0]),1,9)';[0.3 0.09 0.9]']) % Note that when mounted on our foldable table the height of the wheel is 0.8128
elseif(robottype == 2)    
    orBodySetTransform(crankid,[reshape(rodrigues([0 pi/2 0]),1,9)';[0.35 0.1 0.8]'])
end

orEnvSetOptions('debug 3')
orEnvSetOptions('publishanytime 0');

probs.cbirrt = orEnvCreateProblem('CBiRRT','Hubo');
probs.crankmover = orEnvCreateProblem('CBiRRT','crank',0);

manips = orRobotGetManipulators(robotid);

%jointdofs = 0:29;
%activedofs = setdiff(jointdofs,[manips{1}.joints,manips{2}.joints,manips{3}.joints,manips{4}.joints]); %red rid of fingers/toes
%activedofs = jointdofs;
if(robottype == 1)
    activedofs = sort([0,manips{1}.armjoints,manips{2}.armjoints,manips{3}.armjoints,manips{4}.armjoints]);
    %orRobotSetDOFValues(robotid,[0.182 -0.15 -0.15 0.182], [7 9 6 8]); %knees     
    orRobotSetDOFValues(robotid,[-0.95 -0.95], [19 20]); %elbows
    orRobotSetDOFValues(robotid,[1 1], [41 56]); %thumb
elseif(robottype == 2)
    activedofs = sort([6,manips{1}.armjoints,manips{2}.armjoints,manips{3}.armjoints,manips{4}.armjoints]);
    orRobotSetDOFValues(robotid,[-0.14 0.23 0.23 -0.14], [1 2 13 14]); %knees
    orRobotSetDOFValues(robotid,[-0.95 -0.95], [19 26]); %elbows
    orRobotSetDOFValues(robotid,[-1 -1], [44 59]); %thumb
end
orRobotSetActiveDOFs(robotid,activedofs);
initconfig = orRobotGetDOFValues(robotid);

links = orBodyGetLinks(robotid);

Tee = cell(1,4);

for i = 1:4
    Tlink = MakeTransform(links(1:9,manips{i}.eelink+1),links(10:12,manips{i}.eelink+1));
    Tgrasp = [manips{i}.Tgrasp;[0 0 0 1]];

    Tee{i} = Tlink*Tgrasp;
end


crankjointind = 0;
%jointaxis = str2num(orProblemSendCommand(['GetJointAxis name kitchen jointind fridgejointind'],probs.tcbirrt));
jointtm = str2num(orProblemSendCommand(['GetJointTransform name crank jointind ' num2str(crankjointind)],probs.cbirrt));





cogtarg = [-0.05 0.085 0];

%orRobotSetActiveDOFs(robotid,setdiff(activedofs,[manips{2}.armjoints]));

%orEnvPlot(GetTrans(T0_RH1),'size',10)
%orEnvPlot(GetTrans(T0_LH1),'size',10)
%orEnvPlot(cogtarg,'size',20)

if(robottype == 1)
    %TODO
elseif(robottype == 2)
    rhanddofs = [45 46 47 48 49 50 51 52 53 54 55 56 57 58 59];
    rhandclosevals = [0.439 0.683 0.497  0.439 0.683 0.497   0.439 0.683 0.497   0.439 0.683 0.497  -0.194 -0.565 -1.242];
    rhandopenvals = [zeros(1,size(rhanddofs,2)-1),-1];
    lhanddofs = [30 31 32 33 34 35 36 37 38 39 40 41 42 43 44];
    lhandclosevals = [0.439 0.683 0.497  0.439 0.683 0.497   0.439 0.683 0.497   0.439 0.683 0.497  -0.194 -0.565 -1.242];
    lhandopenvals = [zeros(1,size(lhanddofs,2)-1),-1];
end


if(robottype == 1)    
    footlinknames = ' Body_RAR Body_LAR ';
elseif(robottype == 2)    
    footlinknames = ' rightFoot leftFoot ';
end

%%
if(robottype == 1)    
    footlinknames = ' Body_RAR Body_LAR polyscale 0.7 0.5 0 polytrans -0.015 0 0 ';
elseif(robottype == 2)    
    footlinknames = ' rightFoot leftFoot ';
end

handrot = rodrigues([0 -pi/2 0]);
%transoffset = [0 0.2 0];

if(robottype == 1)
    transoffset = [0 0.15 0];
    Rhandtrans = jointtm(10:12)-[0.075 0 0];
    Lhandtrans = jointtm(10:12)-[0.075 0 0];
    T0_RH1 = MakeTransform(rodrigues([0 pi/2 0])*handrot,Rhandtrans'-transoffset');
    T0_LH1 = MakeTransform(rodrigues([0 pi/2 0])*handrot,Lhandtrans'+transoffset');
elseif(robottype == 2)
    transoffset = [0 0.165 0];
    Rhandtrans = jointtm(10:12)-[0.035 0 0];
    Lhandtrans = jointtm(10:12)-[0.035 0 0];
    T0_RH1 = MakeTransform(rodrigues([-pi/8 0 0])*rodrigues([0 -pi/5 0])*rodrigues([0 0 pi/4])*handrot,Rhandtrans'-transoffset');
    T0_LH1 = MakeTransform(rodrigues([pi/8 0 0])*rodrigues([0 -pi/5 0])*rodrigues([0 0 -pi/4])*handrot,Lhandtrans'+transoffset');
end


TSRString1 = SerializeTSR(0,'NULL',T0_LH1,eye(4),[0,0,0,0,0,0,0,0,0,0,0,0]);
TSRString2 = SerializeTSR(1,'NULL',T0_RH1,eye(4),[0,0,0,0,0,0,0,0,0,0,0,0]);
TSRString3 = SerializeTSR(2,'NULL',Tee{3},eye(4),[0,0,0,0,0,0,0,0,0,0,0,0]);


T0_w0H =  MakeTransform(links(1:9,manips{5}.eelink+1),links(10:12,manips{5}.eelink+1));
Tw0_eH = eye(4);
%Bw0H = [-0.1,0.1,-0.1,0.1,-0.1,0.01,0,0,-pi,pi,0,0];
Bw0H = [0,0,-0.1,0.1,-0.1,0.01,0,0,0,0,0,0];
TSRString4 = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H);

TSRChainStringGrasping = [SerializeTSRChain(0,1,0,1,TSRString1,'NULL',[]) ' ' SerializeTSRChain(0,1,0,1,TSRString2,'NULL',[]) ' '  SerializeTSRChain(0,1,1,1,TSRString3,'NULLL',[]) ' ' SerializeTSRChain(0,1,1,1,TSRString4,'NULL',[])];


orProblemSendCommand(['RunCBiRRT psample 0.2 timelimit 60 supportlinks 2 ' footlinknames  ' smoothingitrs ' num2str(normalsmoothingitrs) ' ' TSRChainStringGrasping],probs.cbirrt);
!mv cmovetraj.txt movetraj0.txt

orProblemSendCommand(['traj movetraj0.txt'],probs.cbirrt);

orEnvWait(robotid);
startik = orRobotGetDOFValues(robotid)';
%startik = orProblemSendCommand(['DoGeneralIK exec supportlinks 2 ' footlinknames ' movecog ' num2str(cogtarg) ' nummanips 3 maniptm 0 ' num2str([GetRot(T0_LH1),GetTrans(T0_LH1)]) ' maniptm 1 ' num2str([GetRot(T0_RH1),GetTrans(T0_RH1)]) ' maniptm 2 ' num2str([GetRot(Tee{3}),GetTrans(Tee{3})])],probs.cbirrt);
%startik = orProblemSendCommand(['DoGeneralIK exec nummanips 3 maniptm 0 ' num2str([GetRot(T0_LH1),GetTrans(T0_LH1)]) ' maniptm 1 ' num2str([GetRot(T0_RH1),GetTrans(T0_RH1)]) ' maniptm 2 ' num2str([GetRot(Tee{3}),GetTrans(Tee{3})])],probs.cbirrt);
%orRobotSetDOFValues(robotid,str2num(startik));

%%

%left hand is evaluated first so need to make right hand relative to crank!

T0_w0L = MakeTransform([1 0 0 0 1 0 0 0 1]',jointtm(10:12));

Tw0_eL = inv(T0_w0L)*T0_LH1;
Bw0L = [0 0   0 0   0 0  0 pi   0 0   0 0];
TSRstring1 = SerializeTSR(0,'NULL',T0_w0L,Tw0_eL,Bw0L);


cranklinks = orBodyGetLinks(crankid);
T0_crankcrank = MakeTransform(cranklinks(1:9,2),cranklinks(10:12,2));
%defined relative to crank
T0_w0R = eye(4);
Tw0_eR = inv(T0_crankcrank)*T0_RH1;
Bw0R = [0 0   0 0   0 0   0 0   0 0   0 0]; %note: in frame of crank
TSRstring2 = SerializeTSR(1,'crank crank',T0_w0R,Tw0_eR,Bw0R);

TSRstring3 = SerializeTSR(2,'NULL',Tee{3},eye(4),zeros(1,12)); %left foot
TSRChainStringFootOnly = SerializeTSRChain(0,0,1,1,TSRstring3,'NULL',[]);

T0_w0H =  MakeTransform(links(1:9,manips{5}.eelink+1),links(10:12,manips{5}.eelink+1));
Tw0_eH = eye(4);
Bw0H = [-0.05,0.05,-0.1,0.1,-100,100,-pi,pi,-pi,pi,-pi,pi];
TSRstring4 = SerializeTSR(4,'NULL',T0_w0H,Tw0_eH,Bw0H);

TSRChainStringFootandHead = [TSRChainStringFootOnly ' ' SerializeTSRChain(0,0,1,1,TSRstring4,'NULL',[])];
TSRChainString = [SerializeTSRChain(0,0,1,1,TSRstring1,'crank',crankjointind) ' ' SerializeTSRChain(0,0,1,1,TSRstring2,'NULL',[]) ' '  TSRChainStringFootandHead];

%TSRChainString = [SerializeTSRChain(0,0,1,1,TSRstring1,'crank',crankjointind) ' ' SerializeTSRChain(0,0,1,1,TSRstring2,'NULL',[]) ' '  TSRChainStringFootOnly];
%TSRChainString = [SerializeTSRChain(0,0,1,1,TSRstring1,'crank',crankjointind) ' ' SerializeTSRChain(0,0,1,1,TSRstring3,'NULL',[])];

crank_rot = pi/6;
TSRChainMimicDOF = 1;


Tcrank_rot = MakeTransform(rodrigues([crank_rot 0 0]),[0 0 0]');
Tcrank_rot2 = MakeTransform(rodrigues([0 0 crank_rot]),[0 0 0]');

T0_cranknew = T0_w0L*Tcrank_rot;

T0_LH2 = T0_cranknew*inv(T0_w0L)*T0_LH1;
T0_RH2 = T0_crankcrank*Tcrank_rot2*inv(T0_crankcrank)*T0_RH1;

goalik = orProblemSendCommand(['DoGeneralIK exec supportlinks 2 ' footlinknames ' movecog ' num2str(cogtarg) ' nummanips 3 maniptm 0 ' num2str([GetRot(T0_LH2),GetTrans(T0_LH2)]) ' maniptm 1 ' num2str([GetRot(T0_RH2),GetTrans(T0_RH2)]) ' maniptm 2 ' num2str([GetRot(Tee{3}),GetTrans(Tee{3})])],probs.cbirrt);
orRobotSetDOFValues(robotid,str2num(goalik));


%%

orRobotSetDOFValues(crankid,0,crankjointind)
%orRobotSetDOFValues(robotid,str2num(startik));
orRobotSetDOFValues(robotid,initconfig);

%goaljoints = str2num(startik);
%orProblemSendCommand(['RunCBiRRT supportlinks 2 ' footlinknames  ' smoothingitrs ' num2str(normalsmoothingitrs) ' jointgoals '  num2str(numel(goaljoints)) ' ' num2str(goaljoints) ' ' TSRChainStringFootandHead],probs.cbirrt);


%%
orProblemSendCommand(['traj movetraj0.txt'],probs.cbirrt);
%%
orEnvWait(robotid);

%orRobotSetDOFValues(robotid,rhandclosevals,rhanddofs);
%orRobotSetDOFValues(robotid,lhandclosevals,lhanddofs);

%%

%orRobotSetDOFValues(robotid,str2num(startik));
goaljoints = [str2num(goalik), 0*ones(1,TSRChainMimicDOF)];
orProblemSendCommand(['RunCBiRRT supportlinks 2 ' footlinknames  ' smoothingitrs ' num2str(fastsmoothingitrs) ' jointgoals '  num2str(numel(goaljoints)) ' ' num2str(goaljoints) ' ' TSRChainString],probs.cbirrt);
!mv cmovetraj.txt movetraj1.txt
orProblemSendCommand(['traj movetraj1.txt'],probs.cbirrt);
orProblemSendCommand(['traj movetraj1.txt'],probs.crankmover);
orEnvWait(robotid);


%%
%open hands
%orRobotSetDOFValues(robotid,rhandopenvals,rhanddofs);
%orRobotSetDOFValues(robotid,lhandopenvals,lhanddofs);


goaljoints = startik;
orProblemSendCommand(['RunCBiRRT supportlinks 2 ' footlinknames  ' smoothingitrs ' num2str(normalsmoothingitrs) ' jointgoals '  num2str(numel(goaljoints)) ' ' num2str(goaljoints) ' ' TSRChainStringFootandHead],probs.cbirrt);

!mv cmovetraj.txt movetraj2.txt
orProblemSendCommand(['traj movetraj2.txt'],probs.cbirrt);
orEnvWait(robotid);


%%
%open hands
%orRobotSetDOFValues(robotid,rhandopenvals,rhanddofs);
%orRobotSetDOFValues(robotid,lhandopenvals,lhanddofs);

orRobotSetDOFValues(robotid,str2num(goalik));
goaljoints = initconfig';
orProblemSendCommand(['RunCBiRRT supportlinks 2 ' footlinknames  ' smoothingitrs ' num2str(normalsmoothingitrs) ' jointgoals '  num2str(numel(goaljoints)) ' ' num2str(goaljoints) ' ' TSRChainStringFootandHead],probs.cbirrt);
!mv cmovetraj.txt movetraj3.txt
orProblemSendCommand(['traj movetraj3.txt'],probs.cbirrt);
orEnvWait(robotid);


%% playback
pausetime = 0.2;
orRobotSetDOFValues(robotid,initconfig);
%open hands
%orRobotSetDOFValues(robotid,rhandopenvals,rhanddofs);
%orRobotSetDOFValues(robotid,lhandopenvals,lhanddofs);
pause(pausetime)

%start recording here
orProblemSendCommand(['traj movetraj0.txt'],probs.cbirrt);
orEnvWait(robotid);
pause(pausetime)
%orRobotSetDOFValues(robotid,rhandclosevals,rhanddofs);
%orRobotSetDOFValues(robotid,lhandclosevals,lhanddofs);


orProblemSendCommand(['traj movetraj1.txt'],probs.cbirrt);
orProblemSendCommand(['traj movetraj1.txt'],probs.crankmover)
orEnvWait(robotid);
pause(pausetime)
%open hands
%orRobotSetDOFValues(robotid,rhandopenvals,rhanddofs);
%orRobotSetDOFValues(robotid,lhandopenvals,lhanddofs);

for i = 1:3
    orProblemSendCommand(['traj movetraj2.txt'],probs.cbirrt);
    orEnvWait(robotid);
    pause(pausetime)
    %orRobotSetDOFValues(robotid,rhandclosevals,rhanddofs);
    %orRobotSetDOFValues(robotid,lhandclosevals,lhanddofs);


    orProblemSendCommand(['traj movetraj1.txt'],probs.cbirrt);
    orProblemSendCommand(['traj movetraj1.txt'],probs.crankmover);
    orEnvWait(robotid);
    pause(pausetime)
    
    %open hands
    %orRobotSetDOFValues(robotid,rhandopenvals,rhanddofs);
    %orRobotSetDOFValues(robotid,lhandopenvals,lhanddofs);
end


orProblemSendCommand(['traj movetraj3.txt'],probs.cbirrt);
orEnvWait(robotid);
pause(pausetime)
