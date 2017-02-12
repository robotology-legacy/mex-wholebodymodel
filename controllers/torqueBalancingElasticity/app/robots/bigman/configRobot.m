function CONFIG = configRobot(CONFIG_old)
%CONFIGROBOT setup the initial configuration of the robot.
%
% CONFIG = CONFIGROBOT(CONFIG_old) takes as an input the structure 
% CONFIG, which contains all the robot configuration parameters. The output 
% is the same structure properly overwritten. 
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, February 2017
%

% ------------Initialization----------------
global state;

CONFIG = CONFIG_old;

%% Config parameters
% number of dofs
CONFIG.ndof = 25;

% feet size
CONFIG.footSize  = [-0.16 0.16;     % xMin, xMax
                    -0.075 0.075];  % yMin, yMax

CONFIG.p         = 100;  %transmission ratio
CONFIG.dqjInit   = zeros(CONFIG.ndof,1);
dx_bInit         = zeros(3,1);
w_omega_bInit    = zeros(3,1);

%% Rewrite configuration parameters according to the current state
if strcmp(CONFIG.demo_type,'yoga') == 1
    if state == 1

        % overwrite configuration                    
        CONFIG.feet_on_ground  = [1,1];
        
    elseif state == 3
        
        CONFIG.feet_on_ground  = [1,0];
    elseif state == 6
        
        CONFIG.feet_on_ground  = [1,1];   
    end
    SM = initStateMachine(CONFIG, state, 'init');
    CONFIG.qjInit    = transpose(SM.qjRef);
    CONFIG.gainsInit = gains(CONFIG);
    CONFIG.gainsInit = reshapeGains(SM.gainsVector,CONFIG);
    
else

    % initial joints position [deg]
    leftArmInit  = [ 0  8  0  0  0]';
    rightArmInit = [ 0 -8  0  0  0]';
    torsoInit    = [ 0   0  0]';

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

    % joints configuration [rad]
    CONFIG.qjInit = [torsoInit;leftArmInit;rightArmInit;leftLegInit;rightLegInit]*(pi/180);
    
    % the initial gains are defined before the numerical integration
    CONFIG.gainsInit = gains(CONFIG);
end

%% Contact constraints definition
if sum(CONFIG.feet_on_ground) == 2
    
    CONFIG.constraintLinkNames = {'l_sole','r_sole'};
    
elseif CONFIG.feet_on_ground(1) == 1 && CONFIG.feet_on_ground(2) == 0
    
    CONFIG.constraintLinkNames = {'l_sole'};
    
elseif CONFIG.feet_on_ground(1) == 0 && CONFIG.feet_on_ground(2) == 1
    
    CONFIG.constraintLinkNames = {'r_sole'};
end

CONFIG.numConstraints = length(CONFIG.constraintLinkNames);

%% Configure the model using initial conditions
wbm_updateState(CONFIG.qjInit,CONFIG.dqjInit,[dx_bInit;w_omega_bInit]);

% fixing the world reference frame w.r.t. the foot on ground position
[x_bInit,w_R_bInit] = wbm_getWorldFrameFromFixLnk(CONFIG.constraintLinkNames{1},CONFIG.qjInit);

wbm_setWorldFrame(w_R_bInit,x_bInit,[0 0 -9.81]')

% initial state (floating base + joints)
[basePoseInit,~,~,~]          = wbm_getState();
CONFIG.chi_robotInit          = [basePoseInit; CONFIG.qjInit; dx_bInit; w_omega_bInit; CONFIG.dqjInit];

%% Initial dynamics and forward kinematics
% initial state
CONFIG.initState              = robotState(CONFIG.chi_robotInit,CONFIG);
% initial dynamics
CONFIG.initDynamics           = robotDynamics(CONFIG.initState,CONFIG);
% initial forward kinematics
CONFIG.initForKinematics      = robotForKinematics(CONFIG.initState,CONFIG.initDynamics);

end

