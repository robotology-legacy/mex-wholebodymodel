%% INITIALIZETORQUEBALANCING
%  This is the configuration file for balancing simulations of torque 
%  controlled floating base robots using MATLAB. All the controllers
%  implemented here are model-based. The robot model is stored in a .urdf
%  file. The robot dynamics and forward kinematics are obtained using
%  wbm-toolbox library that is a wrapper of yarpWholeBodyModelV2 class.
%
%  LIST OF AVAILABLE SIMULATIONS:
%  - One or two feet balancing about a reference point;
%  - One or two feet tracking of a CoM reference trajectory; 
%  - Highly dynamic Tai Chi (see also https://www.youtube.com/watch?v=9XRI4BeXN78);
%  
%  LIST OF AVAILABLE ROBOTS:
%  - iCub;
%  - Walkman;
%  - Walkman (only legs);
%
%  LIST OF AVAILABLE TOOLS:
%  - elasticJoints: assumes all joints are driven by Series Elastic Actuators.
%                   Motors dynamics is taken into account in both model and control;
%  - linearization: the joint space is linearized about a set point. This
%                   utility is used for verifying the closed loop system 
%                   stability and for gian tuning; 
%  - gainTuning: a gain tuning procedure is performed for setting control gains.
%                It is based on a linearization of the joint space dynamics about the
%                starting joint configuration;
%  - inverseKinematics: a simple double integrator for setting the desired
%                       joint references;
%  - centroidalTransformation: a coordinates transformation is performed on
%                              the robot state. The advantage is that the 
%                              resulting mass matrix is block diagonal, i.e.
%                              floating base dynamics and joints dynamics
%                              are decoupled;
%  - QPSolver: a Quadratic Programming solver for optimizing contact forces
%              and for applying friction cones constaints at contact locations.
%
%  LIST OF SPECIAL TOOLS FOR DEBUGGING:
%  - onlineSimulator: the iDyntree simulator is enabled during the forward
%                     dynamics integration;
%  - stepReference: a reference step is used for verifying the behavior of
%                   system dynamics around a set point (it may be useful
%                   for tuning the joint space linearization);
%  - normalizeQuaterions: after integration, quaterions are normalized such
%                         that |qt| == 1;
%  - addMotorReflectedInertia: add to system mass matrix the motors
%                              reflected inertia (motor inertia is
%                              estimated to be ~10^-5 [Kgm^2]);
%  - considerJointLimits: error message if one of the joints is reaching
%                         the limit;
%  - recordFigures: all data used for figures are recorded and stored in a
%                   .mat file.
%  - makeVideo: make a video from the iDyntree simulator;
%
%  LIST OF AVAILABLE CONTROLLERS:
%  - stackOfTask: a momentum based balancing controller. The control
%                 objective is splitted into two tasks with different
%                 priority. The main control objective is the stabilization
%                 of centroidal momentum dynamics. The secondary task is a
%                 PD control + gravity compensation for the joints dynamics;
%  - jointSpaceController: a joint space controller. No tasks are considered:
%                          a simple inverse kinematics algorithm is used for 
%                          setting the desired joint references. Control torques
%                          are then used for stabilizing the joint space dynamics.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017
%

%% ------------Initialization----------------
clear all  %#ok<CLALL>
close all
clc

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%% DEMO SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% available robot names: 'icubGazeboSim', 'bigman', 'bigman_only_legs'
CONFIG.robot_name                            = 'icubGazeboSim';  
% available demos: 'yoga', 'balancing', 'movements'
CONFIG.demo_type                             = 'balancing';  
% available controllers: 'stackOfTask', 'jointControl'
CONFIG.control_type                          = 'stackOfTask';  

%% %%%%%%%%%%%%%%%%%%%%%%%%%%% TOOLS SETUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% use SEA at each controlled joint
CONFIG.use_SEA                               = false;                      %either true or false
% use the gain tuning procedure for setting the control gains
CONFIG.use_gainTuning                        = true;                       %either true or false
% use centroidal transformation
CONFIG.use_centroidalTransf                  = false;                      %either true or false
% use QP solver
CONFIG.use_QPsolver                          = true;                       %either true or false
% use inverse kinematics for computing the desired joint references
CONFIG.use_ikinSolver                        = true;                       %either true or false

%% %%%%%%%%%%%%%%%%%%%%%% DEMO-SPECIFIC SETUP %%%%%%%%%%%%%%%%%%%%%%%%%% %%
% this setup will affect only the demo specified in CONFIG.demo_type

% YOGA DEMO DEDICATED SETUP
% perform yoga movements while on one foot (default) or just switch between
% one and two contacts
CONFIG.yogaMovements                         = true;                       %either true or false
% yoga demo only on right foot (default: left foot)
CONFIG.demoOnlyRightFoot                     = false;                      %either true or false
% yoga demo on both feet 
CONFIG.demoAlsoRightFoot                     = true;                       %either true or false

% BALANCING AND MOVEMENTS DEMO DEDICATED SETUP
% specify the number of feet in contact
CONFIG.feet_on_ground                        = [0,1];                      %either 0 or 1; [left_foot,right_foot]        
% in case 'movements' demo is selected, specify the amplitude and frequency
% of CoM reference trajectory. The only trajectory available is a sine
% reference for CoM position.
CONFIG.frequencyOfOscillation                = 0.15;                       %[Hz]
CONFIG.amplitudeOfOscillation                = 0.015;                      %[m]
% if params.demo_movements = 1, the variable noOscillationTime is the time, 
% in seconds, that the robot waits before starting the left-and-right
CONFIG.directionOfOscillation                = [0;1;0];
CONFIG.noOscillationTime                     = 0;                          %[s]

%% %%%%%%%%%%%%%%%%%%%%%% VISUALIZATION SETUP %%%%%%%%%%%%%%%%%%%%%%%%%% %%
% activate robot simulator using iDyntree visualizer
CONFIG.visualize_robot_simulator             = true;                       %either true or false
% enable the visualization tool. The list of avaliable figure will be 
% displayed after numerical integration
CONFIG.enable_visualTool                     = true;                       %either true or false

%% %%%%%%%%%%%%%% FORWARD DYNAMICS INTEGRATION SETUP %%%%%%%%%%%%%%%%%%% %%
% integration time [s]
CONFIG.tStart                                = 0;
CONFIG.tEnd                                  = 1;
CONFIG.sim_step                              = 0.01;
% event function. Currently, if a discrete event is detected, the integration 
% stops and it restarts with new initial conditions, just after the event. 
CONFIG.eventFunction                         = @(t,chi) eventState(t,chi);
% integration options. If the numerical integration is slow, try to modify 
% the options 'RelTol' (default: 1e-3) and/or 'AbsTol'  (default: 1e-6).
CONFIG.options                               = odeset('RelTol',1e-3,'AbsTol',1e-4, ...
                                                      'Events',CONFIG.eventFunction,'InitialStep',CONFIG.sim_step,'Stats','on');
% choose the ODE solver. Remember: the funtion to be integrated is in the
% form: M(y)*dy = f(y). It is a stiff ODE system, and the mass matrix is close 
% to singular (this makes the system almost a DAE). For this reason, and
% because the mass matrix is not constant, it is STRONGLY recommended to use
% one of the two ODE solver specifically designed for stiff ODE/DAE: ode15s
% and ode23t. make use of other solvers not only slows down the numerical
% integration, but it can lead to numerical instablity (in particular, do
% not use a fixed step integrator)
CONFIG.odeSolver                             = 'ode15s';
if ~strcmp(CONFIG.odeSolver,'ode15s') && ~strcmp(CONFIG.odeSolver,'ode23t') 
    warning('ODE solver is not ''ode15s'' or ''ode23t''. This may lead to numerical instability!')
end
                                                  
%% %%%%%%%%%%%%%%%%%%%%% SPECIAL TOOLS (DEBUGGING) %%%%%%%%%%%%%%%%%%%%% %%
% WARNING: these tools must be used for debugging only. Enable them will affect
% the simulation setup, e.g. if CONFIG.stepReference == true, CONFIG.demo_type
% is setted to 'balancing' by default.

% the robot is balancing, and the joints reference is a configurable step at t = 0.
CONFIG.stepReference                         = false;                      %either true or false
CONFIG.stepAmplitude                         = 5;                          %[deg]
% normalize quaternions to ensure |qt| == 1
CONFIG.normalize_quaternions                 = false;                      %either true or false
% use motor reflected inertia (desingularization of system mass matrix)
CONFIG.use_motorReflectedInertia             = false;                      %either true or false
% consider joint limits during the simulation
CONFIG.use_jointLimits                       = false;                      %either true or false
% make a video of the iDyntree simulation (offline) 
CONFIG.makeVideo                             = false;                      %either true or false
% record figures data (stored in a .mat file) 
CONFIG.recordData                            = false;                      %either true or false
% robot simulator is active during numerical integration. 
CONFIG.visualize_robot_simulator_ONLINE      = false;                      %either true or false
if CONFIG.visualize_robot_simulator_ONLINE
    warning('Online simulator is active. This will considerably slow down integration!')
end
%% %%%%%%%%%%%%%%% ADVANCED SETUP (ONLY FOR DEVELOPERS) %%%%%%%%%%%%%%%% %%
% tolerances for pseudoinverse (pinv, pinvDamped) and QP hessian matrix
CONFIG.pinv_tol                              = 1e-4;
CONFIG.pinv_damp                             = 5e-4;
CONFIG.reg_HessianQP                         = 1e-3;
% torques saturation
CONFIG.satTorque                             = 60;                         %[Nm]

%% %%%%%%%%%%%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% add required paths. This is assuming the toolbox is installed using 
% codyco-superbuild (https://github.com/robotology/codyco-superbuild).
CONFIG.codyco_root  = getenv('CODYCO_SUPERBUILD_ROOT');
CONFIG.tools_root   = [CONFIG.codyco_root, filesep, '/main/mexWholeBodyModel/controllers/tools'];
CONFIG.src_root     = [CONFIG.codyco_root, filesep, '/main/mexWholeBodyModel/controllers/torqueBalancing/src'];
CONFIG.config_root  = [CONFIG.codyco_root, filesep, '/main/mexWholeBodyModel/controllers/torqueBalancing/',...
                       filesep, 'app/robots/', filesep, CONFIG.robot_name];
% add the folders to matlab path
addpath(genpath(CONFIG.tools_root));
addpath(CONFIG.src_root);
addpath(CONFIG.config_root);

% initialize the robot model
wbm_modelInitialize(CONFIG.robot_name);

%% Resolve option conflicts
if CONFIG.stepReference
    % for performing this regulation task, the robot should not move
    CONFIG.demo_type                         = 'balancing';
    disp('[Configuration]: ''CONFIG.stepReference'' option is TRUE, only ''balancing'' demo available');
end
if CONFIG.use_SEA
    % motor reflected inertia can be used only if not already considered in
    % the model, i.e. only if joint elasticity is not taken into account
    CONFIG.use_motorReflectedInertia         = false;    
    disp('[Configuration]:''CONFIG.use_SEA'' option is TRUE, setting ''CONFIG.use_motorReflectedInertia'' to FALSE');
end
if strcmp(CONFIG.control_type,'jointControl')
    % joint control necessarily requires the use of inverse kinematics and
    % centroidal transformation
    CONFIG.use_centroidalTransf              = true;                       
    CONFIG.use_ikinSolver                    = true; 
    % for the moment, the algorithm used for adding SEA in the dynamics is
    % specific for 'stackOfTask' controller
    CONFIG.use_SEA                           = false;
    % also gain tuning is specific for 'stackOfTask' controller
    CONFIG.use_gainTuning                    = false; 
    % the same for QPsolver
    CONFIG.use_QPsolver                      = false; 
    disp(['[Configuration]: balancing controller is ''jointControl'', the following tools are not available: '...
          'SEA, gainTuning, QP solver.' ...
          'Ikin solver and centroidal transformation has been activated by default']);    
end
if strcmp(CONFIG.demo_type,'yoga')
    % gain tuning and ikin solver are not available for this demo.
    % The reason is, the state machine is not available for the offline
    % integration they use.
    CONFIG.use_ikinSolver                    = false; 
    CONFIG.use_gainTuning                    = false; 
    % hence, the only controller available for this demo is Stack of Task
    % (joint controller requires ikin solver)
    CONFIG.control_type                      = 'stackOfTask';
    disp(['[Configuration]: running ''Yoga'' demo. ' ...
          'The following tools/controllers are not available: ikinSolver, gainTuning, ''jointControl''']);     
end
if strcmp(CONFIG.control_type,'stackOfTask')
    % centroidal transformation still not implemented for stack of task controller
    CONFIG.use_centroidalTransf              = false;                       
    disp('[Configuration]: balancing controller is ''stackOfTask'': centroidal transformation is not implemented.');
end
if CONFIG.enable_visualTool
    % create a folder and initialize stored values 
    CONFIG.outputDir = './media';
    if (~exist(CONFIG.outputDir, 'dir'))
        mkdir(CONFIG.outputDir);
        disp('[Visualization]: created the folder ''./media'' for storing data')
    end
    % saving the user-defined configuration
    save('./media/storedValuesFwdDyn','CONFIG','-v7.3')
end

% initialize the forward dynamics integration
initForwardDynamics(CONFIG);
