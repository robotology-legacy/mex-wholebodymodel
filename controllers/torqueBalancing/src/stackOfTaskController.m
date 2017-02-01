function controlParam = stackOfTaskController(CONFIG,gain,trajectory,DYNAMICS,FORKINEMATICS,STATE)
%STACKOFTASKCONTROLLER implements a momentum-based control algorithm for
%                      floating base robots.
%
% STACKOFTASKCONTROLLER computes the desired control torques at joints
% using a task-based approach. The first task is the achievement of a 
% desired robot momentum, while the second task is a postural task.
%
% controlParam = STACKOFTASKCONTROLLER(CONFIG, gain, trajectory,
% DYNAMICS,FORKINEMATICS,STATE) takes as input the robot configuration,
% control gains, reference trajectory, and robot forward kinematics,
% dynamics and state.
% The output controlParam contains desired control torques and contact 
% forces and others parameters used for visualization and QP solver.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
import WBM.utilities.skewm;

%% Config parameters
pinv_tol            = CONFIG.pinv_tol;
pinv_damp           = CONFIG.pinv_damp;
feet_on_ground      = CONFIG.feet_on_ground;
ndof                = CONFIG.ndof;

%% Gains
impedances          = gain.impedances;
dampings            = gain.dampings;
momentumGains       = gain.momentumGains;
intMomentumGains    = gain.intMomentumGains;

%% Dynamics
M                   = DYNAMICS.M;
g                   = DYNAMICS.g;
C_nu                = DYNAMICS.C_nu;
dJc_nu              = DYNAMICS.dJc_nu;
H                   = DYNAMICS.H;
Jc                  = DYNAMICS.Jc;
h                   = g + C_nu;
m                   = M(1,1);
Mb                  = M(1:6,1:6);
Mbj                 = M(1:6,7:end);
Mj                  = M(7:end,7:end);
Mjb                 = M(7:end,1:6);
Mbar                = Mj - Mjb/Mb*Mbj;
% The centroidal momentum jacobian is reduced to the joint velocity. This
% is then used to compute the approximation of the angular momentum integral
JH                  = DYNAMICS.JH;
JG                  = JH(:,7:end) - JH(:,1:6)*(eye(6)/Jc(1:6,1:6))*Jc(1:6,7:end);

%% Forward kinematics
xCoM                = FORKINEMATICS.xCoM;
dxCoM               = FORKINEMATICS.dxCoM;
poseRFoot_ang       = FORKINEMATICS.poseRFoot_ang;
poseLFoot_ang       = FORKINEMATICS.poseLFoot_ang;

%% Robot State
qj                  = STATE.qj;
dqj                 = STATE.dqj;

% Trajectory
x_dx_ddx_CoMDes     = trajectory.desired_x_dx_ddx_CoM;
ddqjRef             = trajectory.jointReferences.ddqjRef;
dqjRef              = trajectory.jointReferences.dqjRef;
qjRef               = trajectory.jointReferences.qjRef;
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
x_RFoot              = poseRFoot_ang(1:3);
x_LFoot              = poseLFoot_ang(1:3);
r_RF                 = x_RFoot - xCoM;       % Application point of the contact force on the right foot w.r.t. CoM
r_LF                 = x_LFoot - xCoM;       % Application point of the contact force on the left  foot w.r.t. CoM
AL                   = [eye(3),    zeros(3);
                        skewm(r_LF),eye(3)];
AR                   = [eye(3),    zeros(3);
                        skewm(r_RF),eye(3)];

% One foot or two feet on ground selector
if      sum(feet_on_ground) == 2
    
    A      = [AL,AR];
    pinvA  = pinv(A,pinv_tol);
else
    if      feet_on_ground(1) == 1
        
        A  = AL;
    elseif  feet_on_ground(2) == 1
        
        A  = AR;
    end
    pinvA  = eye(6)/A;
end

%% Desired momentum derivative
% closing the loop on angular momentum integral
deltaPhi           =  JG(4:end,:)*(qj-qjRef);
accDesired         =  [m.*x_dx_ddx_CoMDes(:,3); zeros(3,1)];
velDesired         = -momentumGains*[m.*(dxCoM-x_dx_ddx_CoMDes(:,2)); H(4:end)];
posDesired         = -intMomentumGains*[m.*(xCoM-x_dx_ddx_CoMDes(:,1)); deltaPhi];
HDotDes            =  accDesired + velDesired + posDesired;

%% Control torques equation
JcMinv             = Jc/M;
Lambda             = JcMinv*S;
JcMinvJct          = JcMinv*transpose(Jc);
pinvLambda         = pinv(Lambda,pinv_tol);

% multiplier of contact wrenches in tau0
JBar               = transpose(Jc(:,7:end)) - Mbj'/Mb*transpose(Jc(:,1:6));

% nullspaces
NullA              = eye(6*CONFIG.numConstraints)-pinvA*A;
NullLambda         = eye(ndof)-pinvLambda*Lambda;

Sigma              = -(pinvLambda*JcMinvJct + NullLambda*JBar);
SigmaNA            =  Sigma*NullA;

% Postural task correction
pinvLambdaDamp     =  Lambda'/(Lambda*Lambda' + pinv_damp*eye(size(Lambda,1)));
NullLambdaDamp     =  eye(ndof)-pinvLambdaDamp*Lambda;
posturalCorr       =  NullLambdaDamp*Mbar;
impedances         =  impedances*pinv(posturalCorr,pinv_tol) + 0.01*eye(ndof);
dampings           =  dampings*pinv(posturalCorr,pinv_tol) + 0.01*eye(ndof);

tauModel           =  pinvLambda*(JcMinv*h - dJc_nu) + NullLambda*(h(7:end) -Mbj'/Mb*h(1:6)...
                      + Mbar*ddqjRef - impedances*posturalCorr*qjTilde - dampings*posturalCorr*dqjTilde);

%% Desired contact forces computation
fcHDot             = pinvA*(HDotDes - f_grav);

% Desired f0 without Quadratic Programming
f0                 = zeros(6,1);

if  sum(feet_on_ground) == 2
    
    f0             = -pinv(SigmaNA,pinv_tol)*(tauModel+Sigma*fcHDot);
end

fcDes              = fcHDot + NullA*f0;

%% Output parameters
controlParam.fcHDot   = fcHDot;
controlParam.HDotDes  = HDotDes;
controlParam.fcDes    = fcDes;
controlParam.f_grav   = f_grav;
controlParam.tauModel = tauModel;
controlParam.Sigma    = Sigma;
controlParam.f0       = f0;
controlParam.NullA    = NullA;
controlParam.A        = A;

end
