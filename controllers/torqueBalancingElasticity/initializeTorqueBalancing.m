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

%% Global variables definition
global force_feet state com_error;

% first testing framework for extending YOGA++ demo to matlab controller 
force_feet = 0;
state      = 1;
com_error  = zeros(3,1);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%% BASIC SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% Configure the simulation
CONFIG.feet_on_ground                        = [1,1];                      %either 0 or 1; [left foot,right foot]
CONFIG.use_QPsolver                          = 1;                          %either 0 or 1                    
CONFIG.assume_rigid_joints                   = 1;                          %either 0 or 1

% robot names: icubGazeboSim, bigman, bigman_only_legs
CONFIG.robot_name                            = 'icubGazeboSim';  

% available demos: yoga, balacing, movements
CONFIG.demo_type                             = 'yoga';                 

%% Visualization setup
% robot simulator
CONFIG.visualize_robot_simulator             = 1;                          %either 0 or 1
% forward dynamics integration results
CONFIG.visualize_integration_results         = 0;                          %either 0 or 1
CONFIG.visualize_joints_dynamics             = 0;                          %either 0 or 1
CONFIG.visualize_motors_dynamics             = 0;

%% Integration time [s]
CONFIG.tStart                                = 0;
CONFIG.tEnd                                  = 15;
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

if strcmp(CONFIG.demo_type,'yoga') == 1
    % for now yoga demo is available only for icub
    if strcmp(CONFIG.robot_name,'icubGazeboSim')
    else
        error('For now, yoga demo is available only for robot iCub')
    end
    % fixed step is not available if yoga is on
    CONFIG.integrateWithFixedStep = 0;
    % choose the stance leg for yoga++
    CONFIG.left_right_yoga = [1,0];                                        %[left foot, right foot]
end

% The fixed step integration needs a desingularization of system mass matrix
% in order to converge to a solution
if CONFIG.integrateWithFixedStep == 1
    
    CONFIG.massCorr = 0.05;
else
    CONFIG.massCorr = 0;
end

% Integration options. If the integration is slow, try to modify these options.
eventFunction  = @(t,chi)eventState(t,chi);

if strcmp(CONFIG.demo_type,'balacing') == 1
    CONFIG.options  = odeset('RelTol',1e-3,'AbsTol',1e-3,'Events',eventFunction,'InitialStep',CONFIG.sim_step);
else
    CONFIG.options  = odeset('RelTol',1e-7,'AbsTol',1e-7,'Events',eventFunction,'InitialStep',CONFIG.sim_step);
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
CONFIG.elastic_root = [CONFIG.utility_root, filesep, '/elasticJoints'];
CONFIG.state_root   = [CONFIG.utility_root, filesep, '/stateMachine'];
CONFIG.src_root     = [CONFIG.codyco_root, filesep, '/main/mexWholeBodyModel/controllers/torqueBalancingElasticity/src'];

CONFIG.config_root  = [CONFIG.codyco_root,  filesep, '/main/mexWholeBodyModel/controllers/torqueBalancingElasticity/',...
                       filesep, 'app/robots/', filesep, CONFIG.robot_name];

% add the paths
addpath(CONFIG.utility_root);
addpath(CONFIG.robot_root);
addpath(CONFIG.plots_root);
addpath(CONFIG.src_root);
addpath(CONFIG.elastic_root);
addpath(CONFIG.state_root);
addpath(CONFIG.config_root);

%% Initialize the robot model
wbm_modelInitialize(CONFIG.robot_name);

%% INITIALIZATION
% initialize the forward dynamics
initForwardDynamics(CONFIG);
