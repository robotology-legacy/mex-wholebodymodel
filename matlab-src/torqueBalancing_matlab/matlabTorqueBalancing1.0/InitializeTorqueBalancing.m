%% InitializeTorqueBalancing
% this is the initialization script for torque balancing simulations of the robot 
% iCub using Matlab. 
% The user can set the parameters below to generate different simulations. 
% The integration is available for both the robot balancing on one foot and
% two feet, and for the robot standing or moving, following a CoM trajectory.
% There are different balancing controllers that can be used to achieve the
% desired control objective.
% For the "stack of task" controller, it is possible to use a QP program to
% assure the contact forces are inside the friction cone.
% A linearization setup for both analysis and gains tuning purpose
% is available, too.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
clear all
close all
clc
%% %%%%%%%%%%%%%%%%%%%%%%%%%%% BASIC SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% Integration setup 
params.demo_movements                       = 1;                           %either 0 or 1
params.feet_on_ground                       = [1,1];                       %either 0 or 1; [left,right]
params.BalancingController                  = 'StackOfTask';               %either 'StackOfTask' or 'JointSpace'

%% Linearized system analysis and gain tuning and QP solver. Both these tools
%% are available only for the 'Stack of Task' controller
params.use_QPsolver                         = 0;                           %either 0 or 1
params.linearize_for_stability_analysis     = 1;                           %either 0 or 1
params.linearize_for_gains_tuning           = 1;                           %either 0 or 1

% if params.linearize_for_gains_tuning = 1, choose between two different
% optimization algorithms: either the nonlinear least squares, 'lsq' or the
% vectorization using Kronecher product, 'kronecher'
params.optimization_algorithm               = 'kronecher'; 

%% Visualization setup 
params.visualize_robot_simulator            = 1;                           %either 0 or 1
params.visualize_integration_results        = 1;                           %either 0 or 1
params.visualize_joints_dynamics            = 1;                           %either 0 or 1
params.visualize_gains_tuning_results       = 1;                           %either 0 or 1; available only if linearize_for_gains_tuning        = 1

% if the visualization of the stability analysis results is activated, the 
% inverse kinematics solver will be used, too. This is because it is necessary 
% for the joints references to be always consistent with respect to the first
% task trajectory, otherwise the linearization fails.
params.visualize_stability_analysis_results  = 1;                          %either 0 or 1; available only if linearize_for_gains_tuning        = 1 
                                                                           %                              or linearize_for_stability_analysis  = 1                                    
%% Integration time [s]
params.tStart                                = 0;   
params.tEnd                                  = 10;   
params.sim_step                              = 0.01;

%% Allow the joints references generation using inverse kinematics
% it will be automatically activated if the "Joint Space" controller is used, 
% or if the visualization of stability analysis results is active,too.
params.jointRef_with_ikin                    = 1;                           %either 0 or 1
params.visualize_ikin_results                = 0;                           %either 0 or 1  
params.ikin_integration_step                 = 0.01; 

%% %%%%%%%%%%%%%%%%%%%%%%%%%% ADVANCED SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% If the robot is not moving, i.e. params.demo_movements = 0, the integration 
% can be very long. In this case try changing the integration options below.
%
% If it still doesn't work well, set "params.integrateWithFixedStep = 1": 
% A forward Euler fixed step integrator will be used instead of Ode15s.
%
if params.demo_movements == 0 

  params.integrateWithFixedStep  = 0;    
  params.options                 = odeset('RelTol',1e-3,'AbsTol', 1e-4);
else
  params.integrateWithFixedStep  = 0;
  params.options                 = odeset('RelTol',1e-6,'AbsTol',1e-6);
end

%% Initialize the model
% this is assuming a 25DoF iCub
params.ndof        = 25;
wbm_modelInitialise('icubGazeboSim');

%% Initial joints position [deg]
if       params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1

% initial conditions for balancing on two feet 
params.torsoInit    = [  0       0      -3]';    
params.leftArmInit  = [ -35.97   29.97   0.06    50.00   0.0]';          
params.rightArmInit = [ -35.97   29.97   0.06    50.00   0.0]'; 
params.leftLegInit  = [ 12  5   0  -10   -1.6   -4]';
params.rightLegInit = [ 12  5   0  -10   -2.8   -5]';

elseif   params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 0
     
% initial conditions for the robot standing on the left foot
params.torsoInit    = [ 0   17  -1 ]';    
params.leftArmInit  = [-37  94   0    50    0 ]';          
params.rightArmInit = [-37  51   0    35    0 ]'; 
params.leftLegInit  = [ 20  23   0   -21   -3    -5 ]';
params.rightLegInit = [ 12  17   0   -10   -6     4  ]';
 
elseif   params.feet_on_ground(1) == 0 && params.feet_on_ground(2) == 1
  
% initial conditions for the robot standing on the right foot
params.torsoInit    = [ 0  -17   1 ]';    
params.leftArmInit  = [-37  51   0    35    0]';          
params.rightArmInit = [-37  94   0    50    0 ]'; 
params.leftLegInit  = [ 12  17   0   -10   -6     4]';
params.rightLegInit = [ 20  23   0   -21   -3    -5]';
end

%% %%%%%%%%%%%%%%%%%%%%% FORWARD DYNAMICS INTEGRATION %%%%%%%%%%%%%%%%%% %%
addpath('./src');
addpath('./src/Linearization');
addpath('./src/Visualization');
addpath('./src/Controllers');
addpath('./src/ForwardDynamics');
addpath('./src/GainsAndTrajectory');
addpath('./src/InverseKinematics');
integrateForwardDynamics(params);
