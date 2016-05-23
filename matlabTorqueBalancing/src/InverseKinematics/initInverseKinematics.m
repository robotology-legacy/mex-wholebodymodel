function [ikinParam, ChiInit, ContFig] = initInverseKinematics(config,state)
%INITINVERSEKINEMATICS generates the initial condition for inverse kinematics 
%                      integration of the robot iCub in MATLAB.
%   INITINVERSEKINEMATICS evaluates the desired Momentum trajectory for the
%   robot and generates proper joint references. The joints accelerations are
%   calculated using a task-based structure: the first task is to guarantee 
%   the contact constraints, the second task to follow a desired Momentum 
%   trajectory while the third task is to keep the initial posture of the 
%   robot. Positions and velocities are obtained by means of a double fixed
%   step integrator.
%
%   [ikinParam, ChiInit, ContFig] = INITINVERSEKINEMATICS(config,state) takes
%   as input the structure CONFIG containing all the utility parameters and
%   the robot initial state, STATE. 
%   The output are the initial conditions for the forward dynamics integration,
%   CHIINIT [13+2ndof x 1], the structure IKINPARAM which contains the joint
%   references and the figures number counter, CONTFIG.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% Config parameters
pinv_tol               = config.pinv_tol;
ContFig                = config.ContFig;
ndof                   = config.ndof;

% State parameters
config.initState       = state;
BasePose               = state.BasePose;
qj                     = state.qj;

% Dynamics parameters
dynamics               = robotDynamics(state,config);
m                      = dynamics.M(1,1);
Jc                     = dynamics.Jc;
JH                     = dynamics.JH;
config.initDynamics    = dynamics;

% Forward kinematics
forKinematics          = robotForKinematics(state,dynamics);
xCoM                   = forKinematics.xCoM;
config.initForKin      = forKinematics;

%% Generate the CoM reference fot the time t = 0
desired_x_dx_ddx_CoM   = trajectoryGenerator(xCoM,0,config);

%% GENERATE THE INITIAL CONDITIONS FOR NUMERICAL INTEGRATION
% define the first task for state velocities: respect the contact constraints
pinvJc                 = pinv(Jc,pinv_tol);
feetErrorDynamics      = zeros(6*config.numConstraints,1);
Nullfeet               = eye(ndof+6) - pinvJc*Jc;

% define the second task for state velocities: follow a desired momentum
% dynamics
pinvJH                 = pinv(JH*Nullfeet,pinv_tol);
CentroidalErrDynamics  = [m*(desired_x_dx_ddx_CoM(:,3)+desired_x_dx_ddx_CoM(:,2)-(xCoM-desired_x_dx_ddx_CoM(:,1)));...
                          zeros(3,1)] - JH*pinvJc*feetErrorDynamics;
NullH                  = eye(ndof+6) - pinvJH*(JH*Nullfeet);

% define the third task for joints velocities: follow a desired joints
% trajectory
JPost                  = [zeros(ndof,6) eye(ndof)];
JointErrDynamics       = zeros(ndof,1)-JPost*(pinvJc*feetErrorDynamics+Nullfeet*pinvJH*CentroidalErrDynamics);
pinvJPost              = pinv(JPost*Nullfeet*NullH,pinv_tol);
  
%% TASK-BASED INITIAL STATE VELOCITY 
NuThirdTask            = pinvJPost*JointErrDynamics;
NuSecondTask           = pinvJH*CentroidalErrDynamics + NullH*NuThirdTask;
NuFirstTask            = pinvJc*feetErrorDynamics + Nullfeet*NuSecondTask;

% initial condition for state integration 
ChiInit                = [BasePose; qj; NuFirstTask];

%% Inverse kinematics integrator
config.wait            = waitbar(0,'Inverse kinematics integration in progress...');
ikinParam              = integrateInverseKinematics(config,ChiInit);
delete(config.wait)       
 
%% Visualize the results of the inverse kinematics
if config.visualize_ikin_results == 1
    
ContFig  =  visualizeInverseKin(config,ikinParam);
end

