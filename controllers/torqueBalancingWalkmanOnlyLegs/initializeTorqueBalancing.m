%% INITIALIZETORQUEBALANCING
%
% This is the initialization script for torque balancing simulations of 
% floating base robots using Matlab.
%
% Forward dynamics integration is available for the robot balancing on one 
% foot or two feet (6 or 12 contact constraints, respectively). The controller 
% ensures stability properties of the system around any set point in case of 
% k=1 (see [Nava et al, IROS 2016]. 
% Contact forces are evaluated though QP solver to ensure unilateral constraints, 
% and to constrain them inside the friction cones.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
clear  all
close  all
clc
%% %%%%%%%%%%%%%%%%%%%%%%%%%%% BASIC SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% Configure the simulation
CONFIG.demo_movements                        = 1;                          %either 0 or 1
CONFIG.feet_on_ground                        = [1,1];                      %either 0 or 1; [left foot,right foot]
CONFIG.use_QPsolver                          = 1;                          %either 0 or 1
CONFIG.robot_name                            = 'bigman_only_legs';

%% Visualization setup
% robot simulator
CONFIG.visualize_robot_simulator             = 1;                          %either 0 or 1
% forward dynamics integration results
CONFIG.visualize_integration_results         = 1;                          %either 0 or 1
CONFIG.visualize_joints_dynamics             = 1;                          %either 0 or 1

%% Integration time [s]
CONFIG.tStart                                = 0;
CONFIG.tEnd                                  = 5;
CONFIG.sim_step                              = 0.01;

%% %%%%%%%%%%%%%%%%%%%%%%%%%% ADVANCED SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% ONLY FOR DEVELOPERS
% tolerances for pseudoinverse and QP
CONFIG.pinv_tol           = 1e-8;
CONFIG.pinv_damp          = 5e-6;
CONFIG.reg_HessianQP      = 1e-3;

%% Forward dynamics integration setup
% CONFIG.integrateWithFixedStep will use a Euler forward integrator instead
% of ODE15s to integrate the forward dynamics. It may be useful for debug.
CONFIG.integrateWithFixedStep = 0;                                         %either 0 or 1

% The fixed step integration needs a desingularization of system mass matrix
% in order to converge to a solution
if CONFIG.integrateWithFixedStep == 1
    
    CONFIG.massCorr = 0.05;
else
    CONFIG.massCorr = 0;
end

% Integration options. If the intrgration is slow, try to modify these
% options.
if CONFIG.demo_movements == 0
    CONFIG.options                   = odeset('RelTol',1e-3,'AbsTol',1e-3);
else
    CONFIG.options                   = odeset('RelTol',1e-6,'AbsTol',1e-6);
end

%% Visualization setup
% this script modifies the default MATLAB options for figures and graphics.
% This will result in a better visualization of the plots.
plot_set

% this is the figure counter. It is used to automatically adapt the figure
% number in case new figures are added
CONFIG.figureCont = 1;

%% Initialize the robot model
wbm_modelInitialize(CONFIG.robot_name);
CONFIG.ndof = 12;

%% Initial joints position [deg]
if sum(CONFIG.feet_on_ground) == 2
    
    % initial conditions for balancing on two feet
    leftLegInit  = [ 0   0   0   0   0  0 ]';
    rightLegInit = [ 0   0   0   0   0  0 ]';
    
elseif CONFIG.feet_on_ground(1) == 1 && CONFIG.feet_on_ground(2) == 0
    
    % initial conditions for the robot standing on the left foot
    leftLegInit  = [  25.5   15   0  -18.5  -5.5  0]';
    rightLegInit = [  25.5    5   0  -40    -5.5  0]';
    
elseif CONFIG.feet_on_ground(1) == 0 && CONFIG.feet_on_ground(2) == 1
    
    % initial conditions for the robot standing on the right foot
    leftLegInit  = [  25.5    5   0  -40    -5.5  0]';
    rightLegInit = [  25.5   15   0  -18.5  -5.5  0]';
end

% feet size
CONFIG.footSize  = [-0.16 0.16;       % xMin, xMax
                    -0.075 0.075];    % yMin, yMax
% joints configuration [rad]
CONFIG.qjInit    = [leftLegInit;rightLegInit]*(pi/180);

%% Paths definition and initialize the forward dynamics integration
% add the required paths. This procedure will make the paths consistent for
% any starting folder.
codyco_root  = getenv('CODYCO_SUPERBUILD_ROOT');
utility_root = [codyco_root, filesep, '/main/mexWholeBodyModel/controllers/tools'];
robot_root   = [utility_root, filesep, '/robotFunctions'];
plots_root   = [utility_root, filesep, '/visualization'];
src_root     = [codyco_root, filesep, '/main/mexWholeBodyModel/controllers/torqueBalancingWalkmanOnlyLegs/src'];
config_root  = [codyco_root, filesep, '/main/mexWholeBodyModel/controllers/torqueBalancingWalkmanOnlyLegs/config'];
init_root    = [codyco_root, filesep, '/main/mexWholeBodyModel/controllers/torqueBalancingWalkmanOnlyLegs/init'];

% add the paths
addpath(utility_root);
addpath(robot_root);
addpath(plots_root);
addpath(src_root);
addpath(config_root);
addpath(init_root);

%% INITIALIZATION
% initialize the forward dynamics
initForwardDynamics(CONFIG);
