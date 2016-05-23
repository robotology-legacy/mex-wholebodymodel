function [dchi,visualization] = forwardDynamics(t,chi,config)
%FORWARDDYNAMICS is the function that will be integrated in the forward 
%                dynamics integrator.
%                [dchi,visualization] = FORWARDDYNAMICS(t,chi,config) takes
%                as input the current time step, T; the robot state, CHI
%                [13+2ndof x 1]; the structure CONFIG which contains the
%                user-defined parameters. The output are the vector to be
%                integrated, DCHI [13+2ndof x1] and the structure
%                VISUALIZATION which contains all the parameters used to
%                generate the plots in the visualizer.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% waitbar
waitbar(t/config.tEnd,config.wait)

%% Configuration
ndof                  = config.ndof;
gains                 = config.gains;
InitDynamics          = config.InitDynamics;
InitState             = config.InitState;
InitForKinematics     = config.InitForKinematics;
jointRef_with_ikin    = config.jointRef_with_ikin;

%% State
state                 = robotState(chi,config);
RotBase               = state.RotBase;
omegaBaseWorld        = state.omegaBaseWorld;
quatBase              = state.quatBase;
VelBase               = state.VelBase;
dqj                   = state.dqj;
qj                    = state.qj;

%% Dynamics
dynamics              = robotDynamics(state,config);
Jc                    = dynamics.Jc;
M                     = dynamics.M;
h                     = dynamics.h;
H                     = dynamics.H;
m                     = M(1,1);

%% Forward kinematics
forKinematics         = robotForKinematics(state,dynamics);
RFootPoseEul          = forKinematics.RFootPoseEul;
LFootPoseEul          = forKinematics.LFootPoseEul;
xCoM                  = forKinematics.xCoM;

%% Joint limits check
% jointLimitsCheck(qj,t);

%% Interpolation for joint reference trajectory with ikin
if jointRef_with_ikin == 1
ikin                               = config.ikin;
trajectory.JointReferences         = interpInverseKinematics(t,ikin);
else
trajectory.JointReferences.ddqjRef = zeros(ndof,1);
trajectory.JointReferences.dqjRef  = zeros(ndof,1);
trajectory.JointReferences.qjRef   = InitState.qj;
end

%% CoM trajectory generator
trajectory.desired_x_dx_ddx_CoM    = trajectoryGenerator(InitForKinematics.xCoM,t,config);
desired_x_dx_ddx_CoM               = trajectory.desired_x_dx_ddx_CoM;

%% Balancing controller
controlParam      =  initController(gains,trajectory,dynamics,forKinematics,config,state);
tau               =  controlParam.tau;
f0                =  controlParam.f0;
fc                =  controlParam.fc;

%% State derivative computation
omegaWorldBase   = transpose(RotBase)*omegaBaseWorld;                               
dquatBase        = quaternionDerivative(omegaWorldBase,quatBase);       
NuQuat           = [VelBase;dquatBase;dqj];
dNu              = M\(Jc'*fc + [zeros(6,1); tau]-h);

% state to be integrated
dchi            = [NuQuat;dNu];

%% Visualization parameters
visualization.ddqjNonLin = dNu(7:end);
visualization.dqj        = dqj;
visualization.JointRef   = trajectory.JointReferences;
visualization.HRef       = [m*desired_x_dx_ddx_CoM(:,2);zeros(3,1)];
visualization.H          = H;
visualization.poseFeet   = [LFootPoseEul;RFootPoseEul];
visualization.fc         = fc;
visualization.tau        = tau;
visualization.qj         = qj;
visualization.f0         = f0;
visualization.xCoM       = xCoM;

end
