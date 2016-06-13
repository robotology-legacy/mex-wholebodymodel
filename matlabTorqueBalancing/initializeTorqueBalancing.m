%% INITIALIZETORQUEBALANCING
% this is the initialization script for torque balancing simulation of the robot 
% iCub using Matlab. 
% The user can set the parameters below to generate different simulations. 
% The integration is available for both the robot balancing on one foot and
% two feet, and for the robot standing or moving, following a CoM trajectory.
% There are different balancing controllers that can be used to achieve the
% desired control objective.
% For the "stack of task" controller, it is possible to use a QP program to
% assure the contact forces are inside the friction cones.
% A linearization setup for both analysis and gains tuning purpose
% is available, too.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
clear all
close all
clc
%% %%%%%%%%%%%%%%%%%%%%%%%%%%% BASIC SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% Integration setup 
CONFIG.demo_movements                       = 1;                           %either 0 or 1
CONFIG.feet_on_ground                       = [1,1];                       %either 0 or 1; [left,right]
CONFIG.controller                           = 'StackOfTask';               %either 'StackOfTask' or 'JointSpace'

%% Linearized system analysis, gains tuning and QP solver. All these tools
%% are available only for the 'Stack of Task' controller
CONFIG.use_QPsolver                         = 0;                           %either 0 or 1
CONFIG.linearize_for_stability_analysis     = 1;                           %either 0 or 1
CONFIG.linearize_for_gains_tuning           = 1;                           %either 0 or 1

% if params.linearize_for_gains_tuning = 1, choose between two different
% optimization algorithms: either the nonlinear least squares, 'NonLinLsq' 
% or the vectorization using Kronecher product, 'kronecher'
CONFIG.optimization_algorithm               = 'NonLinLsq'; 

%% Visualization setup 
CONFIG.visualize_robot_simulator            = 1;                           %either 0 or 1
CONFIG.visualize_integration_results        = 1;                           %either 0 or 1
CONFIG.visualize_joints_dynamics            = 1;                           %either 0 or 1
CONFIG.visualize_gains_tuning_results       = 0;                           %either 0 or 1; available only if linearize_for_gains_tuning        = 1

% if the visualization of stability analysis results is activated, the 
% inverse kinematics solver will be used, too. This is because it is necessary 
% for the joints references to be always consistent with respect to the first
% task trajectory, otherwise the linearization is no longer valid.
CONFIG.visualize_stability_analysis_results  = 0;                          %either 0 or 1; available only if linearize_for_gains_tuning        = 1 
                                                                           %                              or linearize_for_stability_analysis  = 1                                    
%% Integration time [s]
CONFIG.tStart                                = 0;   
CONFIG.tEnd                                  = 10;   
CONFIG.sim_step                              = 0.01;

%% Allow the joints references generation using inverse kinematics
% it will be automatically activated if the "Joint Space" controller is used, 
% or the visualization of stability analysis results is active.
CONFIG.jointRef_with_ikin                    = 1;                          %either 0 or 1
CONFIG.visualize_ikin_results                = 1;                          %either 0 or 1  
CONFIG.ikin_integration_step                 = 0.01; 

%% %%%%%%%%%%%%%%%%%%%%%%%%%% ADVANCED SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% tolerances for pseudoinverse and Qp solver, postural task and
% mass matrix correction
CONFIG.massCorr           = 0;
CONFIG.pinv_tol           = 1e-8;
CONFIG.pinv_damp          = 5e-6;
CONFIG.reg_HessianQP      = 1e-3;
CONFIG.postCorrection     = 1;
% if the robot is not moving, i.e. CONFIG.demo_movements = 0, the integration 
% can be very long. In this case try changing the integration options below.
%
% if it still doesn't work well, set "params.integrateWithFixedStep = 1": 
% A forward Euler fixed step integrator will be used instead of Ode15s.
%
if CONFIG.demo_movements == 0 

  CONFIG.integrateWithFixedStep  = 0;    
  CONFIG.options                 = odeset('RelTol',1e-3,'AbsTol',1e-4);
else
  CONFIG.integrateWithFixedStep  = 0;
  CONFIG.options                 = odeset('RelTol',1e-6,'AbsTol',1e-6);
end

%% Initialize the model
% this is assuming a 25DoF iCub
CONFIG.ndof        = 25;
wbm_modelInitialise('icubGazeboSim');

%% Initial joints position [deg]
if       CONFIG.feet_on_ground(1) == 1 && CONFIG.feet_on_ground(2) == 1

% initial conditions for balancing on two feet 
CONFIG.torsoInit    = [   0    0   -5]';    
CONFIG.leftArmInit  = [ -35   30    0    50    0]';          
CONFIG.rightArmInit = [ -35   30    0    50    0]'; 
CONFIG.leftLegInit  = [  12    5    0   -10   -1.5  -5]';
CONFIG.rightLegInit = [  12    5    0   -10   -2.5  -5]';

elseif   CONFIG.feet_on_ground(1) == 1 && CONFIG.feet_on_ground(2) == 0
     
% initial conditions for the robot standing on the left foot
CONFIG.torsoInit    = [ 0   15  -1 ]';    
CONFIG.leftArmInit  = [-40  95   0    50    0 ]';          
CONFIG.rightArmInit = [-40  50   0    35    0 ]'; 
CONFIG.leftLegInit  = [ 20  25   0   -20   -5    -5 ]';
CONFIG.rightLegInit = [ 10  15   0   -10   -5     5 ]';
 
elseif   CONFIG.feet_on_ground(1) == 0 && CONFIG.feet_on_ground(2) == 1
  
% initial conditions for the robot standing on the right foot
CONFIG.torsoInit    = [  0  -15   1 ]';    
CONFIG.leftArmInit  = [-40   50   0    35    0]';          
CONFIG.rightArmInit = [-40   95   0    50    0 ]'; 
CONFIG.leftLegInit  = [ 10   15   0   -10   -5   5]';
CONFIG.rightLegInit = [ 20   25   0   -20   -5  -5]';
end

%% %%%%%%%%%%%%%%%%%%%%% FORWARD DYNAMICS INTEGRATION %%%%%%%%%%%%%%%%%% %%
addpath('./src');
addpath('./src/Visualization');
addpath('./src/Controllers');
addpath('./src/RobotFunctions');
addpath('./src/UtilityFunctions');
addpath('./src/ForwardDynamics');
addpath('./src/GainsAndTrajectory');
addpath('./src/InverseKinematics');
initForwardDynamics(CONFIG);
