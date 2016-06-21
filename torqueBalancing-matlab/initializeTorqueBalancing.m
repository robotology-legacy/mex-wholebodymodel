%% INITIALIZETORQUEBALANCING
%  This is the initialization script for torque balancing simulation of the robot 
%  iCub using Matlab. 
%  The user can set the parameters below to generate different simulations. 
%  The integration is available for both the robot balancing on one foot and
%  two feet, and for the robot standing or moving, following a CoM trajectory.
%  There are different balancing controllers that can be used to achieve the
%  desired control objective.
%  For the "stack of task" controller, it is also possible to use a QP program 
%  to ensure the contact forces are inside the friction cones. A linearization
%  setup for both analysis and gains tuning purpose is available, too.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
clear all
close all
clc
%% %%%%%%%%%%%%%%%%%%%%%%%%%%% BASIC SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% Configure the simulation 
CONFIG.demo_movements                        = 1;                          %either 0 or 1
CONFIG.feet_on_ground                        = [1,1];                      %either 0 or 1; [left,right]
% choose the balancing controller algorithm. The possibilities for now are
% either 'StackOfTask' or 'JointSpace' controller
CONFIG.controller                            = 'StackOfTask';               

%% Only for 'Stack of Task': QP solver and gains tuning procedure 
CONFIG.use_QPsolver                          = 0;                          %either 0 or 1
CONFIG.gains_tuning                          = 1;                          %either 0 or 1
% if CONFIG.gains_tuning = 1, CONFIG.numberOfPoints is the number of points
% along the joint reference trajectory in which the gains tuning procedure
% is performed
CONFIG.numberOfPoints                        = 5;

%% Visualization setup 
% robot simulator
CONFIG.visualize_robot_simulator             = 1;                          %either 0 or 1
% forward dynamics integration results
CONFIG.visualize_integration_results         = 1;                          %either 0 or 1
CONFIG.visualize_joints_dynamics             = 1;                          %either 0 or 1
% linearization and gains tuning
CONFIG.visualize_gains_tuning_results        = 1;                          %either 0 or 1; available only if gains_tuning = 1

%% Verify the joint space linearization; stability analysis
% run the simulation with CONFIG.linearizationDebug = 1 to verify that the
% joint space linearization is performed properly, and to check the
% controlled system's stability
CONFIG.linearizationDebug                    = 0;                          %either 0 or 1

%% Integration time [s]
CONFIG.tStart                                = 0;   
CONFIG.tEnd                                  = 10;   
CONFIG.sim_step                              = 0.01;

%% Generate the joint references with the inverse kinematics solver
% it will be automatically activated if the "Joint Space" controller is used 
CONFIG.jointRef_with_ikin                    = 1;                          %either 0 or 1
CONFIG.visualize_ikin_results                = 1;                          %either 0 or 1  
CONFIG.ikin_integration_step                 = 0.01; 

%% %%%%%%%%%%%%%%%%%%%%%%%%%% ADVANCED SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% ONLY FOR DEVELOPERS
% tolerances for pseudoinverse, mass matrix and postural task correction
CONFIG.pinv_tol           = 1e-8;
CONFIG.pinv_damp          = 5e-6;
CONFIG.reg_HessianQP      = 1e-3;
CONFIG.postCorrection     = 0;                                             %either 0 or 1
CONFIG.massCorr           = 0;

%% Forward dynamics integration setup
% CONFIG.integrateWithFixedStep will use a Euler forward integrator instead
% of ODE15s to integrate the forward dynamics. 
if CONFIG.demo_movements == 0 

  CONFIG.integrateWithFixedStep  = 0;    
  CONFIG.options                 = odeset('RelTol',1e-3,'AbsTol',1e-4);
else
  CONFIG.integrateWithFixedStep  = 0;
  CONFIG.options                 = odeset('RelTol',1e-6,'AbsTol',1e-6);
end

%% Initialize the robot model
wbm_modelInitialise('icubGazeboSim');
CONFIG.ndof  = 25;

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
addpath('./src');
addpath('./src/Visualization');
addpath('./src/Controllers');
addpath('./src/RobotFunctions');
addpath('./src/UtilityFunctions');
addpath('./src/ForwardDynamics');
addpath('./src/GainsAndTrajectory');
addpath('./src/InverseKinematics');
CONFIG = configCorrection(CONFIG);
initForwardDynamics(CONFIG);
