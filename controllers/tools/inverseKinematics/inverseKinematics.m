function [dchi, visualizeIkinParam] = inverseKinematics(t,chi,CONFIG)
%INVERSEKINEMATICS is the function that generates the derivative of
%                  the robot state from inverse kinematics.
%
% INVERSEKINEMATICS calculates the state derivative using a three
% task stack-of-task controller. The first task is the contact constraint
% dynamics, the second is the achievement of a desired momentum trajectory,
% the third is to keep the initial posture.
%
% [dchi,visualizeIkinParam]=INVERSEKINEMATICS(t,chi,CONFIG)
% takes as input the integration time t, the robot state chi and the
% structure CONFIG which contains all the utility parameters. The output
% are the state derivative dchi [13+2ndof x 1] and the structure
% which contains the parameters for visualization, visualizeIkinParam.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
import WBM.utilities.dquat;
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
STATE             = robotState(chi,CONFIG);
basePose          = STATE.basePose;
qj                = STATE.qj;
w_R_b             = STATE.w_R_b;
nu                = STATE.nu;
dqj               = STATE.dqj;
nuBase            = nu(1:6);

% Dynamics parameters
DYNAMICS          = robotDynamics(STATE,CONFIG);
m                 = DYNAMICS.M(1,1);
Jc                = DYNAMICS.Jc;
JH                = DYNAMICS.JH;
JCoMPose          = DYNAMICS.JCoM;
JCoM              = JCoMPose(1:3,:);
JAng              = JH(4:6,:);
dJCoM_nuPose      = DYNAMICS.dJCoM_nu;
dJCoM_nu          = dJCoM_nuPose(1:3);
dJH_nu            = DYNAMICS.dJH_nu;
dJc_nu            = DYNAMICS.dJc_nu;

% Forward kinematics
FORKINEMATICS     = robotForKinematics(STATE,DYNAMICS);
xCoM              = FORKINEMATICS.xCoM;
TL                = FORKINEMATICS.TL;
TR                = FORKINEMATICS.TR;
poseRFoot_ang     = FORKINEMATICS.poseRFoot_ang;
poseLFoot_ang     = FORKINEMATICS.poseLFoot_ang;

% Gains for inverse kinematics
gainsInit         = CONFIG.gainsInit;
KPosFeet          = gainsInit.corrPosFeet;
KPosMom           = gainsInit.intMomentumGains;
KPosPost          = gainsInit.impedances;
KVelFeet          = 2*sqrt(KPosFeet);
KVelMom           = gainsInit.momentumGains;
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
deltaPoseRFoot     = TR*(poseRFoot_ang-initForKin.poseRFoot_ang);
deltaPoseLFoot     = TL*(poseLFoot_ang-initForKin.poseLFoot_ang);

if     feet_on_ground(1) == 1 && feet_on_ground(2) == 0
    
    deltaPoseFeet      = deltaPoseLFoot;
    
elseif feet_on_ground(1) == 0 && feet_on_ground(2) == 1
    
    deltaPoseFeet      = deltaPoseRFoot;
    
elseif feet_on_ground(1) == 1 && feet_on_ground(2) == 1
    
    deltaPoseFeet      = [deltaPoseLFoot;deltaPoseRFoot];
end

%% Velocity errors
deltaVelFeet       = Jc*nu;
deltaVelCoM        = JCoM*nu - desired_x_dx_ddx_CoM(:,2);
deltaAngMom        = JAng*nu;

%% STACK OF TASK INVERSE KINEMATICS
% first task: not break the constraints at feet
feetErrorDynamics  = -dJc_nu -KPosFeet*deltaPoseFeet -KVelFeet*deltaVelFeet;
NullFeet           =  eye(ndof+6) -pinv(Jc,pinv_tol)*Jc;

% second task: generate a desired Momentum trajectory
MomErrorDynamics   = -dJH_nu +[m*desired_x_dx_ddx_CoM(:,3);zeros(3,1)] -KPosMom*[m*deltaPosCoM; deltaIntAng] ...
                             -KVelMom*[m*deltaVelCoM; deltaAngMom] -JH*pinv(Jc,pinv_tol)*feetErrorDynamics;
JMomTasks          =  JH*NullFeet;
NullMom            =  eye(ndof+6) - pinv(JMomTasks,pinv_tol)*JMomTasks;

% third task: keep the initial posture
JPost              = [zeros(ndof,6) eye(ndof)];

postErrorDynamics  = -KVelPost*nu(7:end) -KPosPost*(qj-initState.qj)...
                     -JPost*pinv(Jc,pinv_tol)*feetErrorDynamics -JPost*NullFeet*pinv(JMomTasks,pinv_tol)*MomErrorDynamics;
JPostTask          =  JPost*NullFeet*NullMom;

% accelerations from inverse kinematics
dnuPost            = pinv(JPostTask,pinv_tol)*postErrorDynamics;
dnuMom             = pinv(JMomTasks,pinv_tol)*MomErrorDynamics + NullMom*dnuPost;
dnu                = pinv(Jc,pinv_tol)*feetErrorDynamics + NullFeet*dnuMom;

%% State derivative
dqt_base           = dquat(basePose(4:end),transpose(w_R_b)*nuBase(4:end));
dchi               = [nuBase(1:3); dqt_base; dqj; dnu];

%% Parameters for visualization
ObtainedCoMTraj                       = [xCoM; JCoM*nu; (JCoM*dnu+dJCoM_nu)];
DesiredCoMTraj                        = [desired_x_dx_ddx_CoM(:,1); desired_x_dx_ddx_CoM(:,2); desired_x_dx_ddx_CoM(:,3)];
visualizeIkinParam.CoMTrajectoryError = [DesiredCoMTraj; ObtainedCoMTraj];
visualizeIkinParam.feetError          = deltaPoseFeet;
visualizeIkinParam.momentumError      = [m*deltaVelCoM; JAng*nu];

end
