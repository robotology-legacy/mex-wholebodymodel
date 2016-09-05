function [dchi,visualization] = forwardDynamics(t,chi,CONFIG)
%FORWARDDYNAMICS is the function that will be integrated in the forward
%                dynamics integrator.
%
%   [dchi,visualization] = FORWARDDYNAMICS(t,chi,CONFIG) takes as input the
%   current time step, t; the robot state, chi [13+2*ndof x 1]; the structure
%   CONFIG which contains the user-defined parameters.
%   The output are the vector to be integrated, dchi [13+2*ndof x1] and the
%   structure visualization which contains all the parameters used to generate 
%   the plots in the visualizer.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% update the waitbar
waitbar(t/CONFIG.tEnd,CONFIG.wait)

%% Robot Configuration
ndof                  = CONFIG.ndof;
gain                  = CONFIG.gainsInit;
qjInit                = CONFIG.qjInit;
xCoMRef               = CONFIG.xCoMRef;

%% Robot State
STATE                 = robotState(chi,CONFIG);
R_b                   = STATE.RotBase;
w_omega_b             = STATE.omegaBaseWorld;
q_b                   = STATE.quatBase;
v_b                   = STATE.VelBase;
dqj                   = STATE.dqj;
qj                    = STATE.qj;
x_b                   = STATE.PosBase;

%% Set the robot state (for wbm functions)
wbm_setWorldFrame(R_b,x_b,[0 0 -9.81]')
wbm_updateState(qj,dqj,[v_b;w_omega_b]);

%% Robot Dynamics
DYNAMICS              = robotDynamics(STATE,CONFIG);
Jc                    = DYNAMICS.Jc;
M                     = DYNAMICS.M;
h                     = DYNAMICS.h;
H                     = DYNAMICS.H;
m                     = M(1,1);

%% Robot Forward kinematics
FORKINEMATICS         = robotForKinematics(STATE,DYNAMICS);
RFootPoseEul          = FORKINEMATICS.RFootPoseEul;
LFootPoseEul          = FORKINEMATICS.LFootPoseEul;
xCoM                  = FORKINEMATICS.xCoM;

%% Joint limits check
% jointLimitsCheck(qj,t);

%% CoM and joints trajectory generator
trajectory.JointReferences.ddqjRef = zeros(ndof,1);
trajectory.JointReferences.dqjRef  = zeros(ndof,1);
trajectory.JointReferences.qjRef   = qjInit;
trajectory.desired_x_dx_ddx_CoM    = trajectoryGenerator(xCoMRef,t,CONFIG);

%% Torque balancing controller
controlParam    =  runController(gain,trajectory,DYNAMICS,FORKINEMATICS,CONFIG,STATE);
tau             =  controlParam.tau;
fc              =  controlParam.fc;

%% State derivative (dchi) computation
b_omega_w       = transpose(R_b)*w_omega_b;
dq_b            = quaternionDerivative(b_omega_w,q_b);
nu_q            = [v_b;dq_b;dqj];
dnu             = M\(Jc'*fc + [zeros(6,1); tau]-h);
% state derivative
dchi            = [nu_q;dnu];

%% Parameters for visualization
visualization.qj          = qj;
visualization.JointRef    = trajectory.JointReferences;
visualization.xCoM        = xCoM;
visualization.poseFeet    = [LFootPoseEul;RFootPoseEul];
visualization.H           = H;
visualization.HRef        = [m*trajectory.desired_x_dx_ddx_CoM(:,2);zeros(3,1)];
visualization.fc          = fc;
visualization.f0          = controlParam.f0;
visualization.tau         = tau;

end
