%% INITIALIZETORQUEBALANCING
%
% This is the configuration file for balancing simulations of torque 
% controlled floating base robots with Series Elastic Actuators using MATLAB. 
%
% List of available simulations:
%
%  - One or two feet balancing about a reference point;
%  - One or two feet tracking of a CoM reference trajectory; 
%  - Highly dynamic Tai Chi (see also https://www.youtube.com/watch?v=9XRI4BeXN78);
%  
% List of available robots:
%
%  - iCub;
%  - Walkman;
%  - Walkman-only legs;
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
clear all 
close all
clc

%% Global variables (for state machine)
global stanceFootForce state CoMError;
% initialize global variables
stanceFootForce    = 0;
state              = 1;
CoMError           = zeros(3,1);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%% BASIC SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% Configure the simulation
CONFIG.feet_on_ground                        = [1,0];                      %either 0 or 1; [left foot,right foot]
% use QP for optimizing contact forces and for applying friction cones
% constaints
CONFIG.use_QPsolver                          = 0;                          %either 0 or 1
% available robot names: 'icubGazeboSim', 'bigman', 'bigman_only_legs'
CONFIG.robot_name                            = 'icubGazeboSim';  
% available demos: 'yoga', 'balancing', 'movements'
CONFIG.demo_type                             = 'yoga';     

%% Visualization setup
% robot simulator using iDyntree visualizer
CONFIG.visualize_robot_simulator             = 1;                          %either 0 or 1
% forward dynamics integration results
CONFIG.visualize_integration_results         = 0;                          %either 0 or 1
% robot simulation is performed during numerical integration. 
% WARNING: THIS WILL CONSIDERABLY SLOW DOWN NUMERICAL INTEGRATION!
CONFIG.visualize_robot_simulator_ONLINE      = 1;                          %either 0 or 1

%% Integration time [s]
CONFIG.tStart                                = 0;
CONFIG.tEnd                                  = 80;
CONFIG.sim_step                              = 0.01;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% DEBUGGING %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% the controller assumes the robot has rigid joints (comparison with
% elastic joint controller)
CONFIG.assume_rigid_joints                   = 0;                          %either 0 or 1
% the robot is balancing, and upper body joints reference is a configurable
% step. Not available for 'bigman_only_legs'
CONFIG.upperBody_stepReference               = 0;                          %either 0 or 1
CONFIG.stepAmplitude                         = 5;                          %[deg]
% normalize quaternions to ensure |q_t| == 1
CONFIG.normalize_quaternions                 = 0;                          %either 0 or 1
% correct mass matrix using motor reflected inertia
CONFIG.use_motorReflectedInertia             = 0;                          %either 0 or 1

%% %%%%%%%%%%%%%%%%%%%%%%%%%% ADVANCED SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% ONLY FOR DEVELOPERS
% tolerances for pseudoinverse and QP
CONFIG.pinv_tol                              = 1e-4;
CONFIG.pinv_damp                             = 5e-4;
CONFIG.reg_HessianQP                         = 1e-3;
% torques saturation
CONFIG.satTorque                             = inf;                        %[Nm]
% initialize plots
plot_set;

%% Forward dynamics integration setup
% CONFIG.integrateWithFixedStep will use a Euler forward integrator instead
% of ODE15s to integrate the forward dynamics. It may be useful for debug.
CONFIG.integrateWithFixedStep = 0;                                         %either 0 or 1

% event function. Currently, if a discrete event is detected, the integration 
% stops and it restarts with updated initial conditions, just after the event. 
eventFunction                 = @(t,chi) eventState(t,chi);

% integration options. If the integration is slow, try to modify these options.
CONFIG.options                = odeset('RelTol',1e-6,'AbsTol',1e-6,'Events',eventFunction,'InitialStep',CONFIG.sim_step);

%% Resolve option conflicts
if strcmp(CONFIG.demo_type,'yoga')
    % fixed step is not available for yoga movements (event function is
    % required)
    CONFIG.integrateWithFixedStep  = 0;  
    % stability test is not available for yoga demo
    CONFIG.upperBody_stepReference = 0;
    % perform yoga positions (default) or skip them 
    CONFIG.yogaMovements           = 1;                                    %either 0 or 1
    % yoga demo using right foot
    CONFIG.demoOnlyRightFoot       = 0;                                    %either 0 or 1
    % yoga demo on both feet
    CONFIG.demoAlsoRightFoot       = 1;                                    %either 0 or 1
end

if CONFIG.upperBody_stepReference
    % for performing a regulation task, the robot should not move
    CONFIG.demo_type               = 'balancing';
end

if strcmp(CONFIG.robot_name ,'bigman_only_legs')
    % this robot does not have an upper body!
    CONFIG.upperBody_stepReference = 0;
end

%% Add required paths
% this procedure will make the paths consistent for any starting folder.
CONFIG.codyco_root  = getenv('CODYCO_SUPERBUILD_ROOT');
CONFIG.tools_root   = [CONFIG.codyco_root, filesep, '/main/mexWholeBodyModel/controllers/tools'];
CONFIG.robot_root   = [CONFIG.tools_root,  filesep, '/robotFunctions'];
CONFIG.plots_root   = [CONFIG.tools_root,  filesep, '/visualization'];
CONFIG.src_root     = [CONFIG.codyco_root, filesep, '/main/mexWholeBodyModel/controllers/torqueBalancingElasticity/src'];
CONFIG.elastic_root = [CONFIG.tools_root,  filesep, '/elasticJoints'];
CONFIG.state_root   = [CONFIG.tools_root,  filesep, '/stateMachine'];
CONFIG.config_root  = [CONFIG.codyco_root, filesep, '/main/mexWholeBodyModel/controllers/torqueBalancingElasticity/',...
                       filesep, 'app/robots/', filesep, CONFIG.robot_name];
% add the paths
addpath(CONFIG.tools_root);
addpath(CONFIG.robot_root);
addpath(CONFIG.plots_root);
addpath(CONFIG.src_root);
addpath(CONFIG.elastic_root);
addpath(CONFIG.state_root);
addpath(CONFIG.config_root);

%% %%%%%%%%%%%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% initialize the robot model
wbm_modelInitialize(CONFIG.robot_name);
% initialize the forward dynamics integration
initForwardDynamics(CONFIG);
