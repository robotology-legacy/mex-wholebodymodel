function controlParam = stackOfTaskController(config,gains,trajectory,dynamics,forKinematics,state)
%STACKOFTASKCONTROLLER is a task-based balancing controller for the humanoid
%                      robot iCub.
%   STACKOFTASKCONTROLLER computes the control torques at joints using a 
%   two task-based approach. The first task is the control of robot
%   momentum, while the second task is a postural task.
%
%   controlParam = STACKOFTASKCONTROLLER(config, gains, trajectory,
%   dynamics, forKinematics, state) takes as input the structure CONFIG,
%   which contains all the utility parameters, and all the structures 
%   related to the robot dynamics, forward kinematics, gains, ecc... .
%   The output is the structure CONTROLPARAM which contains the desired
%   control torques and others parameters used for visualization and QP 
%   solver.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% Config parameters
pinv_tol            = config.pinv_tol;
feet_on_ground      = config.feet_on_ground;
ndof                = config.ndof;
InitDynamics        = config.InitDynamics;
InitState           = config.InitState;
%pinv_damp          = config.pinv_damp;

% Gains
impedances          = gains.impedances;
dampings            = gains.dampings;
posturalCorr        = gains.posturalCorr;
VelGainsMom         = gains.VelGainsMom;
PosGainsMom         = gains.PosGainsMom;

% Dynamics
M                   = dynamics.M;
g                   = dynamics.g;
CNu                 = dynamics.CNu;
dJcNu               = dynamics.dJcNu;
H                   = dynamics.H;
Jc                  = dynamics.Jc;
h                   = g + CNu;
m                   = M(1,1);
Mb                  = M(1:6,1:6);
Mbj                 = M(1:6,7:end);
Mj                  = M(7:end,7:end);

% Forward kinematics
xCoM                = forKinematics.xCoM;
dxCoM               = forKinematics.dxCoM;
RFootPoseEul        = forKinematics.RFootPoseEul;
LFootPoseEul        = forKinematics.LFootPoseEul;

% State
qj                  = state.qj;
dqj                 = state.dqj;

% Trajectory
x_dx_ddx_CoMDes     = trajectory.desired_x_dx_ddx_CoM;
ddqjRef             = trajectory.JointReferences.ddqjRef;
dqjRef              = trajectory.JointReferences.dqjRef;
qjRef               = trajectory.JointReferences.qjRef;
qjTilde             = qj-qjRef;
dqjTilde            = dqj-dqjRef;
   
% General parameters
gravAcc             = 9.81;
S                   = [zeros(6,ndof);
                       eye(ndof,ndof)];
f_grav              = [zeros(2,1);
                      -m*gravAcc;
                       zeros(3,1)];
                    
%% Multiplier of contact wrenches at CoM
posRFoot            = RFootPoseEul(1:3);
posLFoot            = LFootPoseEul(1:3);
distRF              = posRFoot - xCoM;       % Application point of the contact force on the right foot w.r.t. CoM
distLF              = posLFoot - xCoM;       % Application point of the contact force on the left  foot w.r.t. CoM
AL                  = [eye(3),     zeros(3);
                       skew(distLF),eye(3)];
AR                  = [eye(3),     zeros(3);
                       skew(distRF),eye(3)];
                   
% One foot or two feet on ground selector
if      sum(feet_on_ground) == 2
    
    A      = [AL,AR];
    pinvA  = pinv(A,pinv_tol);    
else    
    if      feet_on_ground(1) == 1 
    
    A      = AL;
    elseif  feet_on_ground(2) == 1

    A      = AR;
    end 
    pinvA  = eye(6)/A;
end

%% Desired momentum derivative
% closing the loop on angular momentum integral
deltaPhi           =  InitDynamics.JhReduced(4:end,:)*(qj-InitState.qj);

accDesired         =  [m.*x_dx_ddx_CoMDes(:,3); zeros(3,1)];
velDesired         = -VelGainsMom*[m.*(dxCoM-x_dx_ddx_CoMDes(:,2)); H(4:end)];
posDesired         = -PosGainsMom*[m.*(xCoM-x_dx_ddx_CoMDes(:,1)); deltaPhi];
HDotDes            =  accDesired + velDesired + posDesired;

%% Control torques equation
JcMinv          = Jc/M;
Lambda          = JcMinv*S;
JcMinvJct       = JcMinv*transpose(Jc);
pinvLambda      = pinv(Lambda,pinv_tol);
%pinvLambda     = Lambda'/(Lambda*Lambda' + pinv_damp*eye(size(Lambda,1)));

% multiplier of f in tau0
JBar            = transpose(Jc(:,7:end)) - Mbj'/Mb*transpose(Jc(:,1:6));   

% nullspaces
Nullfc             = eye(6*config.numConstraints)-pinvA*A;
NullLambda         = eye(ndof)-pinvLambda*Lambda;

Sigma              = -(pinvLambda*JcMinvJct + NullLambda*JBar);
SigmaNA            =  Sigma*Nullfc;
tauModel           =  pinvLambda*(JcMinv*h - dJcNu) +NullLambda*(h(7:end) -Mbj'/Mb*h(1:6)...
                     +Mj*ddqjRef -impedances*posturalCorr*qjTilde -dampings*posturalCorr*dqjTilde);

%% Desired contact forces computation
fcHDot             = pinvA*(HDotDes - f_grav);

% Desired f0 without Quadratic Programming
f0                 = zeros(6,1); 

if  sum(feet_on_ground) == 2

f0                 = -pinv(SigmaNA,pinv_tol)*(tauModel+Sigma*fcHDot);
end

fcDes              = fcHDot + Nullfc*f0;

%% Output parameters
controlParam.fcHDot   = fcHDot;
controlParam.fcDes    = fcDes;
controlParam.tauModel = tauModel;
controlParam.Sigma    = Sigma;
controlParam.f0       = f0;
controlParam.Nullfc   = Nullfc;
controlParam.A        = A;

end
