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
params.feet_on_ground                       = [1,0];                       %either 0 or 1; [left,right]
params.BalancingController                  = 'StackOfTask';               %either 'StackOfTask' or 'JointSpace'

%% Linearized system analysis and gain tuning and QP solver (only for "Stack of Task") 
params.linearize_for_stability_analysis     = 1;                           %either 0 or 1
params.linearize_for_gains_tuning           = 1;                           %either 0 or 1
params.use_QPsolver                         = 0;                           %either 0 or 1

%% Visualization setup 
params.visualize_robot_demo                 = 0;                           %either 0 or 1
params.visualize_integration_plot           = 0;                           %either 0 or 1
params.visualize_joints_position            = 1;                           %either 0 or 1; available only if visualize_integration_plot        = 1
params.visualize_joints_error               = 1;                           %either 0 or 1; available only if visualize_integration_plot        = 1
params.visualize_gains_tuning_plot          = 1;                           %either 0 or 1; available only if linearize_for_gains_tuning        = 1
params.visualize_stability_analysis_plot    = 1;                           %either 0 or 1; available only if linearize_for_gains_tuning        = 1 
                                                                           %                              or linearize_for_stability_analysis  = 1                                    
%% Integration time [s]
params.tStart                               = 0;   
params.tEnd                                 = 1;   
params.sim_step                             = 0.01;

%% Allow the joints references generation using inverse kinematics
% This is suggested if the user wants to compare the linearized joint space
% dynamics with the nonlinear one; it will be always active if the user
% wants to use the "Joint Space" controller
params.jointRef_with_ikin                   = 0;                           %either 0 or 1
params.visualize_ikin_results               = 0;                           %either 0 or 1  
params.ikin_integration_step                = 0.01; 

%% %%%%%%%%%%%%%%%%%%%%%%%%%% ADVANCED SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% If the robot is not moving, i.e. params.demo_movements = 0, the integration 
% can be very long. In this case try changing the integration options below.
%
% If it still doesn't work well, set "params.integrateWithFixedStep = 1": 
% A forward Eulrer fixed step integrator will be used instead of Ode15s.
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
params.leftArmInit  = [ -20   30  0.0  45   0.0]';          
params.rightArmInit = [ -20   30  0.0  45   0.0]'; 
params.torsoInit    = [ -10.0   0.0    0.0]';
 
if       params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1
     
% initial conditions for balancing on two feet 
 params.leftLegInit  = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';
 params.rightLegInit = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';

elseif   params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 0
     
% initial conditions for the robot standing on the left foot
 params.leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
 params.rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1]'; 
 
elseif   params.feet_on_ground(1) == 0 && params.feet_on_ground(2) == 1
  
% initial conditions for the robot standing on the right foot
 params.leftLegInit  = [  25.5   5.0    0.0  -40    -5.5  -0.1]';
 params.rightLegInit = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
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
 