function [MODEL,INIT_CONDITIONS] = configureRobot(CONFIG)
%CONFIGROBOT setup or update the initial conditions for forward dynamics 
%            integration. This file configures robot iCub.
%
% Format: [MODEL,INIT_CONDITIONS] = CONFIGROBOT(CONFIG)
%
% Inputs:  - CONFIG it is the structure containing all user-defined parameters. 
%
% Output:  - MODEL is a structure defining the robot model;        
%          - INIT_CONDITIONS is a structure containing initial conditions
%            for integration.
%         
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
global state;

%% Include configuration in the model
MODEL.CONFIG   = CONFIG;
% number of robot DoFs
MODEL.ndof     = 25;
% feet size
MODEL.feetSize = [-0.05  0.10;    % xMin, xMax
                  -0.025 0.025];  % yMin, yMax  
% transmission ratio
MODEL.eta      = 0.01; 
% initialize waitbar
MODEL.wait     = waitbar(0,'Integrating forward dynamics...');
% if == 1, it disables online visualizer
MODEL.disableVisForGraphics = 0;
% define a time vector with fixed step
MODEL.timeTot = MODEL.CONFIG.tStart:MODEL.CONFIG.sim_step:MODEL.CONFIG.tEnd;

%% Initial joint and floating base velocities
INIT_CONDITIONS.dqjInit         = zeros(MODEL.ndof,1);
INIT_CONDITIONS.dx_bInit        = zeros(3,1);
INIT_CONDITIONS.w_omega_bInit   = zeros(3,1);              
         
%% Define joint positions and control gains or update them according to the 
%% current state (finite state machine)
if strcmp(MODEL.CONFIG.demo_type,'yoga')
    if state == 1
        % two feet balancing                   
        MODEL.CONFIG.feet_on_ground  = [1,1];   
    elseif state == 3
        % left foot balancing
        MODEL.CONFIG.feet_on_ground  = [1,0];
    elseif state == 7
        % two feet balancing
        MODEL.CONFIG.feet_on_ground  = [1,1];
    elseif state == 9
        % right foot balancing
        MODEL.CONFIG.feet_on_ground  = [0,1];
    elseif state == 13
        % two feet balancing
        MODEL.CONFIG.feet_on_ground  = [1,1]; 
    end 
    % initialize state machine: prefix 'init' indicates the function
    % is called BEFORE numerical integration
    SM  = initStateMachine(INIT_CONDITIONS, MODEL, state, 'init');
    % initial joint positions. WARNING: IF THIS FUNCTION IS CALLED AFTER A
    % DISCRETE EVENT, THESE INITIAL POSITIONS ARE USED ONLY AS REFERENCE
    INIT_CONDITIONS.qjInit    = SM.qjRef;
    % initial gains
    INIT_CONDITIONS.GAINS     = reshapeGains(SM.gainsVector,MODEL);
else
    % Initial joints position (NO YOGA)
    if sum(MODEL.CONFIG.feet_on_ground) == 2
        % initial conditions for balancing on two feet
        leftArmInit   = [ -20  30  0  45  0];
        rightArmInit  = [ -20  30  0  45  0];
        torsoInit     = [ -10   0  0];
        leftLegInit   = [  25.5   0   0  -18.5  -5.5  0];
        rightLegInit  = [  25.5   0   0  -18.5  -5.5  0];
        % joints configuration [rad]
        INIT_CONDITIONS.qjInit = [transpose(torsoInit);transpose(leftArmInit); ...
                                  transpose(rightArmInit);transpose(leftLegInit); ...
                                  transpose(rightLegInit)]*(pi/180);
    
    elseif MODEL.CONFIG.feet_on_ground(1) == 1 && MODEL.CONFIG.feet_on_ground(2) == 0
        % initial conditions for the robot standing on the left foot
        INIT_CONDITIONS.qjInit = transpose([ 0.0462,-0.5256,-0.0269, ...
                                             0.1874, 1.6258, 0.2462, 0.3053,-0.0948, ...
                                            -0.3553, 1.8546, 0.7323, 0.3905,-0.1169, ...
                                             0.1047, 0.2547, 0.0378, 0.0732, 0.0958, 0.1637, ...
                                             0.2923, 0.8611, 1.2866,-1.7688, 0.4568,-0.0163]);  
    
    elseif MODEL.CONFIG.feet_on_ground(1) == 0 && MODEL.CONFIG.feet_on_ground(2) == 1 
        % initial conditions for the robot standing on the right foot
        INIT_CONDITIONS.qjInit = transpose([ 0.0462,-0.5256,-0.0269, ...
                                            -0.3553, 1.8546, 0.7323, 0.3905,-0.1169, ...
                                             0.1874, 1.6258, 0.2462, 0.3053,-0.0948, ...
                                             0.2923, 0.8611, 1.2866,-1.7688, 0.4568,-0.0163, ...
                                             0.1047, 0.2547, 0.0378, 0.0732, 0.0958, 0.1637,]);         
    end
    % initial gains
    INIT_CONDITIONS.GAINS = gains(MODEL);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% %%%%%%%%%% INITIAL ROBOT AND FLOATING BASE CONFIGURATION %%%%%%%%%%%% %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% contact constraints
if sum(MODEL.CONFIG.feet_on_ground) == 2
    % two feet balancing
    MODEL.constraintLinkNames = {'l_sole','r_sole'};
    
elseif MODEL.CONFIG.feet_on_ground(1) == 1 && MODEL.CONFIG.feet_on_ground(2) == 0
    % left foot balancing
    MODEL.constraintLinkNames = {'l_sole'};
    
elseif MODEL.CONFIG.feet_on_ground(1) == 0 && MODEL.CONFIG.feet_on_ground(2) == 1
    % right foot balancing
    MODEL.constraintLinkNames = {'r_sole'};
end

%% Update robot state
wbm_updateState(INIT_CONDITIONS.qjInit,INIT_CONDITIONS.dqjInit,[INIT_CONDITIONS.dx_bInit;INIT_CONDITIONS.w_omega_bInit]);
% fixing the world reference frame w.r.t. the foot on ground position
[x_bInit,w_R_bInit]            = wbm_getWorldFrameFromFixLnk(MODEL.constraintLinkNames{1},INIT_CONDITIONS.qjInit);
% set world frame. This is done in case wbm_wrappers are called using
% optimized mode
wbm_setWorldFrame(w_R_bInit,x_bInit,[0; 0; -9.81])
[basePoseInit,~,~,~]           = wbm_getState();
% initial state (floating base + joints)
INIT_CONDITIONS.chi_robotInit  = [basePoseInit; INIT_CONDITIONS.qjInit; INIT_CONDITIONS.dx_bInit; INIT_CONDITIONS.w_omega_bInit; INIT_CONDITIONS.dqjInit];

%% Initial robot state (demuxed), dynamics and forward kinematics
% initial state (demuxed)
INIT_CONDITIONS.INITSTATE         = robotState(INIT_CONDITIONS.chi_robotInit,MODEL);
% initial dynamics
INIT_CONDITIONS.INITDYNAMICS      = robotDynamics(INIT_CONDITIONS.INITSTATE,MODEL);
% initial forward kinematics
INIT_CONDITIONS.INITFORKINEMATICS = robotForKinematics(INIT_CONDITIONS.INITSTATE,INIT_CONDITIONS.INITDYNAMICS);

end
