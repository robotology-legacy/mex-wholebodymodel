function [dchi,visualization] = forwardDynamics(t,chi,CONFIG)
%FORWARDDYNAMICS is the function that will be integrated in the forward 
%                dynamics integrator.
%                [dchi,visualization] = FORWARDDYNAMICS(t,chi,config) takes
%                as input the current time step, T; the robot state, CHI
%                [13+2ndof x 1]; the structure CONFIG which contains the
%                user-defined parameters.
%                The output are the vector to be integrated, DCHI [13+2ndof x1]
%                and the structure VISUALIZATION which contains all the parameters 
%                used to generate the plots in the visualizer.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% waitbar
waitbar(t/CONFIG.tEnd,CONFIG.wait)

%% Robot Configuration
ndof                  = CONFIG.ndof;
gains                 = CONFIG.gains;
initState             = CONFIG.initState;
initForKinematics     = CONFIG.initForKinematics;

%% Robot State
STATE                 = robotState(chi,CONFIG);
RotBase               = STATE.RotBase;
omegaBaseWorld        = STATE.omegaBaseWorld;
quatBase              = STATE.quatBase;
VelBase               = STATE.VelBase;
dqj                   = STATE.dqj;
qj                    = STATE.qj;

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

%% Interpolation for joint reference trajectory with ikin
if CONFIG.jointRef_with_ikin == 1
trajectory.JointReferences         = interpInverseKinematics(t,CONFIG.ikin);
else
trajectory.JointReferences.ddqjRef = zeros(ndof,1);
trajectory.JointReferences.dqjRef  = zeros(ndof,1);
trajectory.JointReferences.qjRef   = initState.qj;
end

%% CoM trajectory generator
trajectory.desired_x_dx_ddx_CoM    = trajectoryGenerator(initForKinematics.xCoM,t,CONFIG);

%% Balancing controller
controlParam        =  initController(gains,trajectory,DYNAMICS,FORKINEMATICS,CONFIG,STATE);
tau                 =  controlParam.tau;
fc                  =  controlParam.fc;

%% State derivative computation
omegaWorldBase     = transpose(RotBase)*omegaBaseWorld;                               
dquatBase          = quaternionDerivative(omegaWorldBase,quatBase);      
NuQuat             = [VelBase;dquatBase;dqj];
dNu                = M\(Jc'*fc + [zeros(6,1); tau]-h);

% state derivative 
dchi               = [NuQuat;dNu];

%% Parameters for visualization
visualization.ddqjNonLin  = dNu(7:end);
visualization.dqj         = dqj;
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