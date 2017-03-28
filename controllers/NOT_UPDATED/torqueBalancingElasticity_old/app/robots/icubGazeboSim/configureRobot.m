function INIT_CONDITIONS = configureRobot(CONFIG)
%CONFIGROBOT setup the initial conditions for forward dynamics integration.
%
% INIT_CONDITIONS = configureRobot(CONFIG) takes as an input the structure 
% CONFIG, which contains all the configuration parameters. The output is
% the structure INIT_CONDITIONS containing the initial condition for
% forward dynamics integration.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, February 2017
%

% ------------Initialization----------------
global state;
% counter for updating yoga movements

%% Include configuration in the initial conditions
INIT_CONDITIONS.CONFIG      = CONFIG;
% number of robot DoFs
INIT_CONDITIONS.CONFIG.ndof = 25;
% feet size
INIT_CONDITIONS.CONFIG.feetSize = [-0.050  0.100;     % xMin, xMax
                                   -0.025  0.025];    % yMin, yMax
% transmission ratio
INIT_CONDITIONS.CONFIG.eta  = 0.01; 
% initial joint and floating base velocities
INIT_CONDITIONS.dqjInit     = zeros(INIT_CONDITIONS.CONFIG.ndof,1);
dx_bInit                    = zeros(3,1);
w_omega_bInit               = zeros(3,1);              
         
%% UPDATE JOINT POSITIONS, GAINS AND REFERENCES ACCORDING TO CURRENT STATE (YOGA)
if strcmp(INIT_CONDITIONS.CONFIG.demo_type,'yoga')

    if state == 1
        % two feet balancing                   
        INIT_CONDITIONS.CONFIG.feet_on_ground  = [1,1];   
    elseif state == 3
        % left foot balancing
        INIT_CONDITIONS.CONFIG.feet_on_ground  = [1,0];
    elseif state == 7
        % again two feet balancing
        INIT_CONDITIONS.CONFIG.feet_on_ground  = [1,1];
    elseif state == 9
        % right foot balancing
        INIT_CONDITIONS.CONFIG.feet_on_ground  = [0,1];
    elseif state == 13
        % two feet balancing
        INIT_CONDITIONS.CONFIG.feet_on_ground  = [1,1]; 
    end 
    % initialize state machine: prefix 'init' indicates the initialization
    % is called before the numerical integration
    SM                        = initStateMachine(INIT_CONDITIONS, state, 'init');
    % initial joint positions. WARNING: IF THIS FUNCTION IS CALLED AFTER A
    % DISCRETE EVENT, THESE INITIAL POSITIONS ARE USED ONLY AS REFERENCE
    INIT_CONDITIONS.qjInit    = SM.qjRef;
    % initial gains
    INIT_CONDITIONS.gainsInit = reshapeGains(SM.gainsVector,INIT_CONDITIONS.CONFIG);
    % tresholds for state machine
    INIT_CONDITIONS.TRESHOLDS = SM.TRESHOLDS; 
else
    % Initial joints position (NO YOGA)
    if sum(INIT_CONDITIONS.CONFIG.feet_on_ground) == 2
        % initial conditions for balancing on two feet
        leftArmInit   = [ -20  30  0  45  0]';
        rightArmInit  = [ -20  30  0  45  0]';
        torsoInit     = [ -10   0  0]';
        leftLegInit   = [  25.5   0   0  -18.5  -5.5  0]';
        rightLegInit  = [  25.5   0   0  -18.5  -5.5  0]';
        % joints configuration [rad]
        INIT_CONDITIONS.qjInit = [torsoInit;leftArmInit;rightArmInit;leftLegInit;rightLegInit]*(pi/180);
    
    elseif INIT_CONDITIONS.CONFIG.feet_on_ground(1) == 1 && INIT_CONDITIONS.CONFIG.feet_on_ground(2) == 0
        % initial conditions for the robot standing on the left foot
        INIT_CONDITIONS.qjInit = transpose([ 0.0462,-0.5256,-0.0269, ...
                                             0.1874, 1.6258, 0.2462, 0.3053,-0.0948, ...
                                            -0.3553, 1.8546, 0.7323, 0.3905,-0.1169, ...
                                             0.1047, 0.2547, 0.0378, 0.0732, 0.0958, 0.1637, ...
                                             0.2923, 0.8611, 1.2866,-1.7688, 0.4568,-0.0163]);  
    
    elseif INIT_CONDITIONS.CONFIG.feet_on_ground(1) == 0 && INIT_CONDITIONS.CONFIG.feet_on_ground(2) == 1 
        % initial conditions for the robot standing on the right foot
        INIT_CONDITIONS.qjInit = transpose([ 0.0462,-0.5256,-0.0269, ...
                                            -0.3553, 1.8546, 0.7323, 0.3905,-0.1169, ...
                                             0.1874, 1.6258, 0.2462, 0.3053,-0.0948, ...
                                             0.2923, 0.8611, 1.2866,-1.7688, 0.4568,-0.0163, ...
                                             0.1047, 0.2547, 0.0378, 0.0732, 0.0958, 0.1637,]);         
    end
    % initial gains
    INIT_CONDITIONS.gainsInit = gains(INIT_CONDITIONS.CONFIG);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Contact constraints 
if sum(INIT_CONDITIONS.CONFIG.feet_on_ground) == 2
    
    INIT_CONDITIONS.CONFIG.constraintLinkNames = {'l_sole','r_sole'};
    
elseif INIT_CONDITIONS.CONFIG.feet_on_ground(1) == 1 && INIT_CONDITIONS.CONFIG.feet_on_ground(2) == 0
    
    INIT_CONDITIONS.CONFIG.constraintLinkNames = {'l_sole'};
    
elseif INIT_CONDITIONS.CONFIG.feet_on_ground(1) == 0 && INIT_CONDITIONS.CONFIG.feet_on_ground(2) == 1
    
    INIT_CONDITIONS.CONFIG.constraintLinkNames = {'r_sole'};
end

%% INITIAL ROBOT AND FLOATING BASE CONFIGURATION
% update robot state. This is done in case wbm_wrappers are called using
% optimized mode
wbm_updateState(INIT_CONDITIONS.qjInit,INIT_CONDITIONS.dqjInit,[dx_bInit;w_omega_bInit]);
% fixing the world reference frame w.r.t. the foot on ground position
[x_bInit,w_R_bInit]               = wbm_getWorldFrameFromFixLnk(INIT_CONDITIONS.CONFIG.constraintLinkNames{1},INIT_CONDITIONS.qjInit);
% set world frame. This is done in case wbm_wrappers are called using
% optimized mode
wbm_setWorldFrame(w_R_bInit,x_bInit,[0 0 -9.81]')
[basePoseInit,~,~,~]              = wbm_getState();
% initial state (floating base + joints)
INIT_CONDITIONS.chi_robotInit     = [basePoseInit; INIT_CONDITIONS.qjInit; dx_bInit; w_omega_bInit; INIT_CONDITIONS.dqjInit];

%% Initial state (demuxed), dynamics and forward kinematics
% initial state (demuxed)
INIT_CONDITIONS.initState         = robotState(INIT_CONDITIONS.chi_robotInit,INIT_CONDITIONS.CONFIG);
% initial dynamics
INIT_CONDITIONS.initDynamics      = robotDynamics(INIT_CONDITIONS.initState,INIT_CONDITIONS.CONFIG);
% initial forward kinematics
INIT_CONDITIONS.initForKinematics = robotForKinematics(INIT_CONDITIONS.initState,INIT_CONDITIONS.initDynamics);

end
