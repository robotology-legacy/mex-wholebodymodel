function [dChi, visualizeIkinParam] = inverseKinematics(t,Chi,CONFIG)
%INVERSEKINEMATICS is the function that generates the derivative of
%                  the robot state from inverse kinematics.
%   INVERSEKINEMATICS calculates the state derivative using a three
%   task stack-of-task controller. The first task is the contact constraint
%   dynamics, the second is the achievement of a desired momentum trajectory,
%   the third is to keep the initial posture.
%
%   [dChiIkin,visualizeIkinParam]=INVERSEKINEMATICS(t,Chi,config)
%   takes as input the integration time T, the robot state CHI and the
%   structure CONFIG which contains all the utility parameters. The output
%   are the state derivative DCHI  [13+2ndof x 1] and the structure 
%   which contains the parameters for visualization, VISUALIZEIKINPARAM.
%   
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
waitbar(t/CONFIG.tEnd,CONFIG.wait)

% Config parameters
pinv_tol          = CONFIG.pinv_tol;
feet_on_ground    = CONFIG.feet_on_ground;
ndof              = CONFIG.ndof;  
initState         = CONFIG.initState;
initDynamics      = CONFIG.initDynamics;
initForKin        = CONFIG.initForKin;

qjInit            = initState.qj;
xCoMInit          = initForKin.xCoM;
JcInit            = initDynamics.Jc;
JHInit            = initDynamics.JH;
JhReduced         = JHInit(:,7:end) -JHInit(:,1:6)*(eye(6)/JcInit(1:6,1:6))*JcInit(1:6,7:end);

% State parameters
STATE             = robotState(Chi,CONFIG);
basePose          = STATE.basePose;
qj                = STATE.qj;
RotBase           = STATE.RotBase;
Nu                = STATE.Nu;
dqj               = STATE.dqj;
NuBase            = Nu(1:6);  

% Dynamics parameters
DYNAMICS          = robotDynamics(STATE,CONFIG);
m                 = DYNAMICS.M(1,1);
Jc                = DYNAMICS.Jc;
JH                = DYNAMICS.JH;
JCoMPose          = DYNAMICS.JCoM;
JCoM              = JCoMPose(1:3,:);
JAng              = JH(4:6,:);
dJCoMNuPose       = DYNAMICS.dJCoMNu;
dJCoMNu           = dJCoMNuPose(1:3);
dJHNu             = DYNAMICS.dJHNu;
dJcNu             = DYNAMICS.dJcNu;

% Forward kinematics
FORKINEMATICS     = robotForKinematics(STATE,DYNAMICS);
xCoM              = FORKINEMATICS.xCoM;
TLfoot            = FORKINEMATICS.TLfoot;
TRfoot            = FORKINEMATICS.TRfoot;
RFootPoseEul      = FORKINEMATICS.RFootPoseEul;
LFootPoseEul      = FORKINEMATICS.LFootPoseEul;

% Gains for inverse kinematics
gainsInit         = CONFIG.gains;
KPosFeet          = gainsInit.CorrPosFeet;
KPosMom           = gainsInit.intMomentumGains;
KPosPost          = gainsInit.impedances;
KVelFeet          = 2*sqrt(KPosFeet);
KVelMom           = gainsInit.MomentumGains;
KVelPost          = gainsInit.dampings;

%% CoM trajectory generator
desired_x_dx_ddx_CoM = trajectoryGenerator(xCoMInit,t,CONFIG);

%% Joint limits check
% jointLimitsCheck(qj,t);

%% ERRORS AT FEET, COM AND MOMENTUM
% angular momentum integral
deltaIntAng        = JhReduced(4:6,:)*(qj-qjInit); 
% CoM position
deltaPosCoM        = xCoM - desired_x_dx_ddx_CoM(:,1);
% feet pose
deltaPoseRFoot     = TRfoot*(RFootPoseEul-initForKin.RFootPoseEul); 
deltaPoseLFoot     = TLfoot*(LFootPoseEul-initForKin.LFootPoseEul);

if     feet_on_ground(1) == 1 && feet_on_ground(2) == 0

deltaPoseFeet      = deltaPoseLFoot;
      
elseif feet_on_ground(1) == 0 && feet_on_ground(2) == 1
     
deltaPoseFeet      = deltaPoseRFoot;          

elseif feet_on_ground(1) == 1 && feet_on_ground(2) == 1
    
deltaPoseFeet      = [deltaPoseLFoot;deltaPoseRFoot];    
end

%% Velocity errors
deltaVelFeet       = Jc*Nu;
deltaVelCoM        = JCoM*Nu - desired_x_dx_ddx_CoM(:,2);
deltaAngMom        = JAng*Nu;

%% STACK OF TASK INVERSE KINEMATICS
% first task: not break the constraints at feet 
feetErrorDynamics  = -dJcNu -KPosFeet*deltaPoseFeet -KVelFeet*deltaVelFeet;
NullFeet           =  eye(ndof+6) -pinv(Jc,pinv_tol)*Jc;

% second task: generate a desired Momentum trajectory
MomErrorDynamics   = -dJHNu +[m*desired_x_dx_ddx_CoM(:,3);zeros(3,1)] -KPosMom*[m*deltaPosCoM; deltaIntAng] ...
                     -KVelMom*[m*deltaVelCoM; deltaAngMom] -JH*pinv(Jc,pinv_tol)*feetErrorDynamics;
JMomTaks           =  JH*NullFeet;
NullMom            =  eye(ndof+6) - pinv(JMomTaks,pinv_tol)*JMomTaks;

% third task: keep the initial posture
JPost              = [zeros(ndof,6) eye(ndof)];

postErrorDynamics  = -KVelPost*Nu(7:end) -KPosPost*(qj-initState.qj)...
                     -JPost*pinv(Jc,pinv_tol)*feetErrorDynamics -JPost*NullFeet*pinv(JMomTaks,pinv_tol)*MomErrorDynamics;
JPostTask          =  JPost*NullFeet*NullMom;

% accelerations from inverse kinematics
dNuPost            = pinv(JPostTask,pinv_tol)*postErrorDynamics;
dNuMom             = pinv(JMomTaks,pinv_tol)*MomErrorDynamics + NullMom*dNuPost;
dNu                = pinv(Jc,pinv_tol)*feetErrorDynamics + NullFeet*dNuMom;

%% State derivative
dQuatBase          = quaternionDerivative(transpose(RotBase)*NuBase(4:end), basePose(4:end));
dChi               = [NuBase(1:3); dQuatBase; dqj; dNu];

%% Parameters for visualization
ObtainedCoMTraj                       = [xCoM; JCoM*Nu; (JCoM*dNu+dJCoMNu)];
DesiredCoMTraj                        = [desired_x_dx_ddx_CoM(:,1); desired_x_dx_ddx_CoM(:,2); desired_x_dx_ddx_CoM(:,3)];
visualizeIkinParam.CoMTrajectoryError = [DesiredCoMTraj; ObtainedCoMTraj];
visualizeIkinParam.feetError          = deltaPoseFeet;
visualizeIkinParam.momentumError      = [m*deltaVelCoM; JAng*Nu];

end
