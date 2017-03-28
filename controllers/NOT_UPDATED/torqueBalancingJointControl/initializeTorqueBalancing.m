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
CONFIG.robot_name                            = 'icubGazeboSim';            

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

%% Visualize the inverse kinematics results
CONFIG.visualize_ikin_results                = 0;                          %either 0 or 1
CONFIG.ikin_integration_step                 = 0.01;

%% %%%%%%%%%%%%%%%%%%%%%%%%%% ADVANCED SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% ONLY FOR DEVELOPERS
% tolerances for pseudoinverse
CONFIG.pinv_tol           = 1e-8;
CONFIG.pinv_damp          = 5e-6;

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

% Integration options. If the integration is slow, try to modify these options.
if CONFIG.demo_movements
    CONFIG.options = odeset('RelTol',1e-6,'AbsTol',1e-6,'InitialStep',CONFIG.sim_step);
else
    CONFIG.options = odeset('RelTol',1e-3,'AbsTol',1e-3,'InitialStep',CONFIG.sim_step);
end

%% Visualization setup
% this script modifies the default MATLAB options for figures and graphics.
% This will result in a better visualization of the plots.
plot_set

% this is the figure counter. It is used to automatically adapt the figure
% number in case new figures are added
CONFIG.figureCont = 1;

%% Paths definition and initialize the forward dynamics integration
% add the required paths. This procedure will make the paths consistent for
% any starting folder.
CONFIG.codyco_root  = getenv('CODYCO_SUPERBUILD_ROOT');
CONFIG.utility_root = [CONFIG.codyco_root, filesep, '/main/mexWholeBodyModel/controllers/tools'];
CONFIG.robot_root   = [CONFIG.utility_root, filesep, '/robotFunctions'];
CONFIG.plots_root   = [CONFIG.utility_root, filesep, '/visualization'];
CONFIG.ikin_root    = [CONFIG.utility_root, filesep, '/inverseKinematics'];
CONFIG.centr_root   = [CONFIG.utility_root, filesep, '/centroidalTransformation'];
CONFIG.src_root     = [CONFIG.codyco_root, filesep, '/main/mexWholeBodyModel/controllers/torqueBalancingJointControl/src'];

CONFIG.config_root  = [CONFIG.codyco_root,  filesep, '/main/mexWholeBodyModel/controllers/torqueBalancingJointControl/',...
                       filesep, 'app/robots/', filesep, CONFIG.robot_name];

% add the paths
addpath(CONFIG.utility_root);
addpath(CONFIG.robot_root);
addpath(CONFIG.plots_root);
addpath(CONFIG.src_root);
addpath(CONFIG.centr_root);
addpath(CONFIG.ikin_root);
addpath(CONFIG.config_root);

%% Initialize the robot model
wbm_modelInitialize(CONFIG.robot_name);

% robot initial configuration
[CONFIG.ndof,CONFIG.qjInit,CONFIG.footSize] = configRobot(CONFIG);

%% INITIALIZATION
% initialize the forward dynamics
initForwardDynamics(CONFIG);
