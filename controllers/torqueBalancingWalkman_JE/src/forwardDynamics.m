function [dchi_total,visualization] = forwardDynamics(t,chi_total,CONFIG)
%FORWARDDYNAMICS is the function that will be integrated in the forward
%                dynamics integrator.
%
% [dchi,visualization] = FORWARDDYNAMICS(t,chi,CONFIG) takes as input the
% current time step, t; the robot state, chi [13+2*ndof x 1]; the structure
% CONFIG which contains the user-defined parameters.
% The output are the vector to be integrated, dchi [13+2*ndof x1] and the
% structure visualization which contains all the parameters used for
% generating the plots in the visualizer.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% update the waitbar
import WBM.utilities.dquat;
waitbar(t/CONFIG.tEnd,CONFIG.wait)

%% Robot Configuration
ndof                  = CONFIG.ndof;
chi                   = chi_total(1:(13+2*ndof));

if CONFIG.consider_el_joints == 1
     % motor configuration
     theta                 = chi_total(14+2*ndof:13+3*ndof);
     dtheta                = chi_total(14+3*ndof:end);
     ELASTICITY            = addElasticJoints(CONFIG);
end

gain                  = CONFIG.gainsInit;
qjInit                = CONFIG.qjInit;

% CoM initial position
CONFIG.xCoMRef        = CONFIG.initForKinematics.xCoM;

%% Robot State
STATE                 = robotState(chi,CONFIG);
w_R_b                 = STATE.w_R_b;
w_omega_b             = STATE.w_omega_b;
qt_b                  = STATE.qt_b;
dx_b                  = STATE.dx_b;
dqj                   = STATE.dqj;
qj                    = STATE.qj;
x_b                   = STATE.x_b;

%% Set the robot state (for wbm functions in 'optimized mode')
wbm_setWorldFrame(w_R_b,x_b,[0 0 -9.81]')
wbm_updateState(qj,dqj,[dx_b;w_omega_b]);

%% Robot Dynamics
DYNAMICS              = robotDynamics(STATE,CONFIG);
Jc                    = DYNAMICS.Jc;
M                     = DYNAMICS.M;
h                     = DYNAMICS.h;
H                     = DYNAMICS.H;
m                     = M(1,1);

%% Robot Forward kinematics
FORKINEMATICS         = robotForKinematics(STATE,DYNAMICS);
poseRFoot_ang         = FORKINEMATICS.poseRFoot_ang;
poseLFoot_ang         = FORKINEMATICS.poseLFoot_ang;
xCoM                  = FORKINEMATICS.xCoM;

%% Joint limits check
% jointLimitsCheck(qj,t);

%% CoM and joints trajectory generator
trajectory.jointReferences.ddqjRef = zeros(ndof,1);
trajectory.jointReferences.dqjRef  = zeros(ndof,1);
trajectory.jointReferences.qjRef   = qjInit;
trajectory.desired_x_dx_ddx_CoM    = trajectoryGenerator(CONFIG.xCoMRef,t,CONFIG);

%% Torque balancing controller
controlParam    = runController(gain,trajectory,DYNAMICS,FORKINEMATICS,CONFIG,STATE,dtheta,theta,ELASTICITY);
tau             = controlParam.tau;
fc              = controlParam.fc;

%% State derivative (dchi) computation
b_omega_w       = transpose(w_R_b)*w_omega_b;
dq_b            = dquat(qt_b,b_omega_w);
nu              = [dx_b;dq_b;dqj];

if CONFIG.consider_el_joints == 1

    dnu             = M\(Jc'*fc + [zeros(6,1);(ELASTICITY.KS*(theta-qj)+ELASTICITY.KD*(dtheta-dqj))]-h);
    % state derivative
    dchi            = [nu;dnu];
    % motor dynamics
    ddtheta         = ELASTICITY.B\(tau+ELASTICITY.KS*(qj-theta)+ELASTICITY.KD*(dqj-dtheta));
    % total state derivative
    dchi_total      = [dchi;dtheta;ddtheta];

else
    dnu             = M\(Jc'*fc + [zeros(6,1);tau]-h);
    % state derivative
    dchi            = [nu;dnu];
    dchi_total      = dchi;
end
    
%% Parameters for visualization
visualization.qj          = qj;
visualization.jointRef    = trajectory.jointReferences;
visualization.xCoM        = xCoM;
visualization.poseFeet    = [poseLFoot_ang;poseRFoot_ang];
visualization.H           = H;
visualization.HRef        = [m*trajectory.desired_x_dx_ddx_CoM(:,2);zeros(3,1)];
visualization.fc          = fc;
visualization.f0          = controlParam.f0;
visualization.tau         = tau;

end
