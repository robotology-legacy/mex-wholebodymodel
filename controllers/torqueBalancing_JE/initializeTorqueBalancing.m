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
% This version has been modified in order to consider joints elasticity in
% the model.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
clear  all
close  all
clc

%% Global variables
global force_feet state com_error;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%% BASIC SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% Configure the simulation
CONFIG.demo_movements                        = 1;                          %either 0 or 1
CONFIG.yoga                                  = 1;                          %either 0 or 1
CONFIG.feet_on_ground                        = [1,1];                      %either 0 or 1; [left foot,right foot]
CONFIG.use_QPsolver                          = 0;                          %either 0 or 1
CONFIG.robot_name                            = 'icubGazeboSim';                       
CONFIG.assume_rigid_joints                   = 1;                          %either 0 or 1
CONFIG.p                                     = 100;                        %transmission ratio (p = 1/eta)

%% Visualization setup
% robot simulator
CONFIG.visualize_robot_simulator             = 1;                          %either 0 or 1
% forward dynamics integration results
CONFIG.visualize_integration_results         = 1;                          %either 0 or 1
CONFIG.visualize_joints_dynamics             = 0;                          %either 0 or 1
CONFIG.visualize_motors_dynamics             = 1;

%% Integration time [s]
CONFIG.tStart                                = 0;
CONFIG.tEnd                                  = 4;
CONFIG.sim_step                              = 0.01;

%% %%%%%%%%%%%%%%%%%%%%%%%%%% ADVANCED SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% ONLY FOR DEVELOPERS
% tolerances for pseudoinverse and QP
CONFIG.pinv_tol           = 1e-8;
CONFIG.pinv_damp          = 5e-6;
CONFIG.reg_HessianQP      = 1e-3;

% a first testing framework for extending YOGA++ demo to matlab controller 
force_feet = zeros(6,1);
state      = 1;
com_error  = zeros(3,1);

if CONFIG.yoga == 1
    % overwrite configuration 
    CONFIG.demo_movements  = 0;                      
    CONFIG.feet_on_ground  = [1,1];
    CONFIG.left_right_yoga = [0,1];                                        %[left foot, right foot]
end

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

% Integration options. If the integration is slow, try to modify these
% options.
eventFunction                        = @(t,chi)event(t,chi);

if CONFIG.demo_movements == 0
    CONFIG.options                   = odeset('RelTol',1e-3,'AbsTol',1e-3,'Events',eventFunction);
else
    CONFIG.options                   = odeset('RelTol',1e-6,'AbsTol',1e-6,'Events',eventFunction);
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
CONFIG.ndof = 25;

%% Initial joints position [deg]
leftArmInit  = [ -20  30  0  45  0]';
rightArmInit = [ -20  30  0  45  0]';
torsoInit    = [ -10   0  0]';

if sum(CONFIG.feet_on_ground) == 2
    
    % initial conditions for balancing on two feet
    leftLegInit  = [  25.5   0   0  -18.5  -5.5  0]';
    rightLegInit = [  25.5   0   0  -18.5  -5.5  0]';
    
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
CONFIG.footSize  = [-0.07 0.07;       % xMin, xMax
                    -0.03 0.03];      % yMin, yMax
% joints configuration [rad]
CONFIG.qjInit    = [torsoInit;leftArmInit;rightArmInit;leftLegInit;rightLegInit]*(pi/180);

%% Paths definition and initialize the forward dynamics integration
% add the required paths. This procedure will make the paths consistent for
% any starting folder.
codyco_root  = getenv('CODYCO_SUPERBUILD_ROOT');
utility_root = [codyco_root, filesep, '/main/mexWholeBodyModel/controllers/tools'];
robot_root   = [utility_root, filesep, '/robotFunctions'];
plots_root   = [utility_root, filesep, '/visualization'];
EJoints_root = [utility_root, filesep, '/elasticJoints'];
src_root     = [codyco_root, filesep, '/main/mexWholeBodyModel/controllers/torqueBalancing_JE/src'];
config_root  = [codyco_root, filesep, '/main/mexWholeBodyModel/controllers/torqueBalancing_JE/config'];
init_root    = [codyco_root, filesep, '/main/mexWholeBodyModel/controllers/torqueBalancing_JE/init'];
centr_root   = [utility_root, filesep, '/centroidalTransformation'];

% add the paths
addpath(utility_root);
addpath(centr_root);
addpath(robot_root);
addpath(plots_root);
addpath(src_root);
addpath(config_root);
addpath(init_root);
addpath(EJoints_root);

%% INITIALIZATION
% initialize the forward dynamics
initForwardDynamics(CONFIG);
