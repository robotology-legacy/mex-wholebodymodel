function [ikinParam, ChiInit, ContFig] = initInverseKin(params)
%INITINVERSEKIN generates the initial condition for inverse kinematics 
%               integration of the robot iCub in MATLAB.
%   INITINVERSEKIN evaluates the desired Momentum trajectory for the robot
%   and generates proper joint references. The joints accelerations are
%   calculated using a task-based structure: the first task is to guarantee 
%   the contact constraints, the second task to follow a desired Momentum 
%   trajectory while the third task is to keep the initial posture of the 
%   robot. Positions and velocities are obtained by means of a double fixed
%   step integrator.
%
%   [ikinParam, ChiInit] = INITINVERSEKIN(params) takes as input the
%   structure params containing all the utility parameters. The output are
%   the initial robot state, ChiInit [13+2ndof x 1], the structure
%   ikinParam which contains the joint references and the figures number
%   counter, ContFig.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% initial parameters
pinv_tol               = params.pinv_tol;
m                      = params.MInit(1,1);
ContFig                = params.ContFig;

%% Desired CoM dynamics
desired_x_dx_ddx_CoM   = generTraj(params.CoMInit(1:3), 0, params.trajectory);  

%% Velocity initial conditions for inverse kinematics integration
% define the first task for state velocities: respect the contact constraints
pinvJcInit             = pinv(params.JcInit,pinv_tol);
feetErrorDynamics      = zeros(6*params.numConstraints,1);
Nullfeet               = eye(params.ndof+6) - pinvJcInit*params.JcInit;

% define the second task for state velocities: follow a desired momentum
% dynamics
JCentrMom              = params.JhInit;
pinvJCentrMom          = pinv(JCentrMom*Nullfeet,pinv_tol);
CentroidalErrDynamics  = [m*(desired_x_dx_ddx_CoM(:,3)+desired_x_dx_ddx_CoM(:,2)-(params.CoMInit(1:3)-desired_x_dx_ddx_CoM(:,1)));...
                          zeros(3,1)] - JCentrMom*pinvJcInit*feetErrorDynamics;
NullCentrMom           = eye(params.ndof+6) - pinvJCentrMom*(JCentrMom*Nullfeet);

% define the third task for joints velocities: follow a desired joints
% trajectory
Jjoint                 = [zeros(params.ndof,6) eye(params.ndof)];
JointErrDynamics       = zeros(params.ndof,1)-Jjoint*(pinvJcInit*feetErrorDynamics+Nullfeet*pinvJCentrMom*CentroidalErrDynamics);
pinvJjoint             = pinv(Jjoint*Nullfeet*NullCentrMom,pinv_tol);
  
%% TASK-BASED INITIAL STATE VELOCITY 
NuThirdTask            = pinvJjoint*JointErrDynamics;
NuSecondTask           = pinvJCentrMom*CentroidalErrDynamics + NullCentrMom*NuThirdTask;
NuFirstTask            = pinvJcInit*feetErrorDynamics + Nullfeet*NuSecondTask;

% initial condition for state integration 
ChiInit                = [params.BasePoseInit; params.qjInit; NuFirstTask];

%% Inverse kinematics integrator
params.wait            = waitbar(0,'Inverse kinematics integration in progress...');
ikinParam              = integrateInverseKin(params,ChiInit);
delete(params.wait)       
 
%% Visualize the results of the inverse kinematics
if params.visualize_ikin_results == 1
    
ContFig  =  visInverseKin(params,ikinParam);
end

