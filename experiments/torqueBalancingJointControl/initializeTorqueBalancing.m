%% INITIALIZETORQUEBALANCING
%
% This is the initialization script for torque balancing simulation of the robot
% iCub using Matlab.
% The user can set the parameters below to generate different simulations.
% The integration is available for both the robot balancing on one foot and
% two feet, and for the robot standing or moving, following a CoM trajectory.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
clear
close all
clc
%% %%%%%%%%%%%%%%%%%%%%%%%%%%% BASIC SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% Configure the simulation
CONFIG.demo_movements                        = 1;                          %either 0 or 1
CONFIG.feet_on_ground                        = [1,1];                      %either 0 or 1; [left,right]

%% Visualization setup
% robot simulator
CONFIG.visualize_robot_simulator             = 1;                          %either 0 or 1
% forward dynamics integration results
CONFIG.visualize_integration_results         = 1;                          %either 0 or 1
CONFIG.visualize_joints_dynamics             = 1;                          %either 0 or 1

%% Integration time [s]
CONFIG.tStart                                = 0;
CONFIG.tEnd                                  = 10;
CONFIG.sim_step                              = 0.01;

%% Visualize the inverse kinematics results
CONFIG.visualize_ikin_results                = 1;                          %either 0 or 1
CONFIG.ikin_integration_step                 = 0.01;

%% %%%%%%%%%%%%%%%%%%%%%%%%%% ADVANCED SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% ONLY FOR DEVELOPERS
% tolerances for pseudoinverse, mass matrix and postural task correction
CONFIG.pinv_tol           = 1e-8;
CONFIG.pinv_damp          = 5e-6;
CONFIG.reg_HessianQP      = 1e-3;
CONFIG.massCorr           = 0;

%% Forward dynamics integration setup
if CONFIG.demo_movements == 0
    
    CONFIG.options          = odeset('RelTol',1e-3,'AbsTol',1e-4);
else
    CONFIG.options          = odeset('RelTol',1e-6,'AbsTol',1e-6);
end

%% Initialize the robot model
wbm_modelInitialise('icubGazeboSim');
plot_set
CONFIG.figureCont = 1;
CONFIG.ndof       = 25;

%% Initial joints position [deg]
leftArmInit  = [ -20  30  0  45  0]';
rightArmInit = [ -20  30  0  45  0]';
torsoInit    = [ -10   0  0]';

if       CONFIG.feet_on_ground(1) == 1 && CONFIG.feet_on_ground(2) == 1
    
    % initial conditions for balancing on two feet
    leftLegInit  = [  25.5   0.1   0  -18.5  -5.5  -0.1]';
    rightLegInit = [  25.5   0.1   0  -18.5  -5.5  -0.1]';
    
elseif   CONFIG.feet_on_ground(1) == 1 && CONFIG.feet_on_ground(2) == 0
    
    % initial conditions for the robot standing on the left foot
    leftLegInit  = [  25.5   15   0  -18.5  -5.5  -0.1]';
    rightLegInit = [  25.5    5   0  -40    -5.5  -0.1]';
    
elseif   CONFIG.feet_on_ground(1) == 0 && CONFIG.feet_on_ground(2) == 1
    
    % initial conditions for the robot standing on the right foot
    leftLegInit  = [  25.5    5   0  -40    -5.5  -0.1]';
    rightLegInit = [  25.5   15   0  -18.5  -5.5  -0.1]';
end

% joint configuration [rad]
CONFIG.qjInit = [torsoInit;leftArmInit;rightArmInit;leftLegInit;rightLegInit]*(pi/180);

%% %%%%%%%%%%%%%%%%%%%%% FORWARD DYNAMICS INTEGRATION %%%%%%%%%%%%%%%%%% %%
codyco_root  = getenv('CODYCO_SUPERBUILD_ROOT');
utility_root = [codyco_root, filesep, '/main/mexWholeBodyModel/controllers/utilityMatlabFunctions'];
robot_root   = [utility_root, filesep, '/RobotFunctions'];
plots_root   = [utility_root, filesep, '/Visualization'];
ikin_root    = [utility_root, filesep, '/InverseKinematics'];
centr_root   = [utility_root, filesep, '/CentroidalTransformation'];
src_root     = [codyco_root, filesep, '/main/mexWholeBodyModel/controllers/experiments/torqueBalancingJointControl/src'];
addpath(utility_root);
addpath(robot_root);
addpath(plots_root);
addpath(ikin_root);
addpath(centr_root);
addpath(src_root);

initForwardDynamics(CONFIG);

