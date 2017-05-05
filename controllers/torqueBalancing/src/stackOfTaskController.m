function CONTROLLER = stackOfTaskController(MODEL,GAINS,TRAJECTORY,DYNAMICS,FORKINEMATICS,STATE)
%STACKOFTASKCONTROLLER implements a momentum-based control algorithm for
%                      controlling constrained floating base robots.
%
% Format: CONTROLLER = STACKOFTASKCONTROLLER(MODEL,GAINS,TRAJECTORY,DYNAMICS,FORKINEMATICS,STATE)
%
% Inputs:  - MODEL is a structure defining the robot model;
%          - GAINS is a structure containing all control gains;
%          - TRAJECTORY stores the CoM and joints reference trajectories;
%          - DYNAMICS contains current robot dynamics;
%          - FORKINEMATICS stores position, orientation and velocity of
%            points in cartesian space;
%          - STATE contains the current system state;
%
% Output:  - CONTROLLER is a structure containing the control torques, 
%            and other parameters for controlling the robot.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% config parameters
pinv_tol            = MODEL.CONFIG.pinv_tol;
pinv_damp           = MODEL.CONFIG.pinv_damp;
feet_on_ground      = MODEL.CONFIG.feet_on_ground;
ndof                = MODEL.ndof;
numConstraints      = sum(MODEL.CONFIG.feet_on_ground);
gravAcc             = 9.81;
S                   = [zeros(6,ndof);
                       eye(ndof,ndof)];

%% Gains
impedances          = GAINS.impedances;
dampings            = GAINS.dampings;
momentumGains       = GAINS.momentumGains;
intMomentumGains    = GAINS.intMomentumGains;
reg_gains           = GAINS.reg_gains;

%% Robot dynamics
M                   = DYNAMICS.M;
% decompose mass matrix
m                   = M(1,1);
Mb                  = M(1:6,1:6);
Mbj                 = M(1:6,7:end);
Mj                  = M(7:end,7:end);
Mjb                 = M(7:end,1:6);
Mbar                = Mj - Mjb/Mb*Mbj;
g                   = DYNAMICS.g;
C_nu                = DYNAMICS.C_nu;
dJc_nu              = DYNAMICS.dJc_nu;
H                   = DYNAMICS.H;
Jc                  = DYNAMICS.Jc;
h                   = g + C_nu;
f_grav              = [zeros(2,1);
                      -m*gravAcc;
                       zeros(3,1)];
% the centroidal momentum jacobian is reduced to the joint velocity. This
% is then used to compute the approximation of the angular momentum integral
JH                  = DYNAMICS.JH;
JG                  = JH(:,7:end) - JH(:,1:6)*(eye(6)/Jc(1:6,1:6))*Jc(1:6,7:end);

%% Forward kinematics
xCoM                = FORKINEMATICS.xCoM;
dxCoM               = FORKINEMATICS.dxCoM;
poseRFoot_ang       = FORKINEMATICS.poseRFoot_ang;
poseLFoot_ang       = FORKINEMATICS.poseLFoot_ang;

%% Robot state
qj                  = STATE.qj;
dqj                 = STATE.dqj;

%% CoM and joints reference trajectory
x_dx_ddx_CoMDes     = TRAJECTORY.desired_x_dx_ddx_CoM;
ddqjRef             = TRAJECTORY.jointReferences.ddqjRef;
dqjRef              = TRAJECTORY.jointReferences.dqjRef;
qjRef               = TRAJECTORY.jointReferences.qjRef;
qjTilde             = qj-qjRef;
dqjTilde            = dqj-dqjRef;

%% Multiplier of contact wrenches at CoM
x_RFoot              = poseRFoot_ang(1:3);
x_LFoot              = poseLFoot_ang(1:3);
r_RF                 = x_RFoot - xCoM;    % Application point of the contact force on the right foot w.r.t. CoM
r_LF                 = x_LFoot - xCoM;    % Application point of the contact force on the left  foot w.r.t. CoM
AL                   = [eye(3),    zeros(3);
                        skewm(r_LF),eye(3)];
AR                   = [eye(3),    zeros(3);
                        skewm(r_RF),eye(3)];

% One foot or two feet on ground selector
if sum(feet_on_ground) == 2  
    A      = [AL,AR];
    pinvA  = pinv(A,pinv_tol);
else
    if feet_on_ground(1) == 1  
        % left foot balancing
        A  = AL;
    elseif feet_on_ground(2) == 1
        %right foot balancing
        A  = AR;
    end
    pinvA  = eye(6)/A;
end

%% Desired momentum derivative
% closing the loop on angular momentum integral
deltaPhi           = JG(4:end,:)*(qj-qjRef);
HDotDes            = [m.*x_dx_ddx_CoMDes(:,3); zeros(3,1)];
HDes               = [m.*(dxCoM-x_dx_ddx_CoMDes(:,2)); H(4:end)];
intHDes            = [m.*(xCoM-x_dx_ddx_CoMDes(:,1)); deltaPhi];
HDotStar           = HDotDes -momentumGains*HDes -intMomentumGains*intHDes;

%% Control torques multipliers
JcMinv             = Jc/M;
Lambda             = JcMinv*S;
JcMinvJct          = JcMinv*transpose(Jc);
pinvLambda         = pinv(Lambda,pinv_tol);
% multiplier of contact wrenches
JBar               = transpose(Jc(:,7:end)) - transpose(Mbj)/Mb*transpose(Jc(:,1:6));
% nullspaces
NullA              = eye(6*numConstraints)-pinvA*A;
NullLambda         = eye(ndof)-pinvLambda*Lambda;
NullLambda_damp    = eye(ndof)-pinvDamped(Lambda,pinv_damp)*Lambda;
Sigma              = -(pinvLambda*JcMinvJct + NullLambda*JBar);
SigmaNA            =  Sigma*NullA;
% Postural task correction
posturalCorr       =  NullLambda_damp*Mbar;
impedances         =  impedances*pinv(posturalCorr,pinv_tol) + reg_gains.*eye(ndof);
dampings           =  dampings*pinv(posturalCorr,pinv_tol) + reg_gains.*eye(ndof);

%% Control torques (without all terms that depend on fc)
% in case of elastic control model, this is part of the motor reference
% velocities. Otherwise tuaModel is directly used in the calculation of
% control torques
if MODEL.CONFIG.use_SEA
    KS             = DYNAMICS.KS;
    KD             = DYNAMICS.KD;
    xi             = STATE.xi;
    tauModel       = pinvLambda*(JcMinv*h - dJc_nu) +KS*(qj-xi) +KD*dqj +NullLambda*(h(7:end) -transpose(Mbj)/Mb*h(1:6)...
                     +Mbar*ddqjRef -impedances*posturalCorr*qjTilde -dampings*posturalCorr*dqjTilde);
else
    tauModel       = pinvLambda*(JcMinv*h - dJc_nu) +NullLambda*(h(7:end) -transpose(Mbj)/Mb*h(1:6)...
                     +Mbar*ddqjRef -impedances*posturalCorr*qjTilde -dampings*posturalCorr*dqjTilde);
end

%% Desired contact forces
% contact forces (no nullspace)
fcHStar            = pinvA*(HDotStar-f_grav);

% Desired f0 (contact forces nullspace) without Quadratic Programming
f0                 = zeros(6,1);

if  sum(feet_on_ground) == 2    
    f0             = -pinv(SigmaNA,pinv_tol)*(tauModel+Sigma*fcHStar);
end

fcDes              = fcHStar + NullA*f0;

%% Output parameters
CONTROLLER.fcHStar   = fcHStar;
CONTROLLER.HDotStar  = HDotStar;
CONTROLLER.fcDes     = fcDes;
CONTROLLER.f_grav    = f_grav;
CONTROLLER.tauModel  = tauModel;
CONTROLLER.Sigma     = Sigma;
CONTROLLER.f0        = f0;
CONTROLLER.NullA     = NullA;
CONTROLLER.A         = A;

end
