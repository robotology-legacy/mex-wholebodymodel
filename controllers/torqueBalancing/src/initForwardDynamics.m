function [] = initForwardDynamics(CONFIG)
%INITFORWARDDYNAMICS setups the forward dynamics integration of the robot 
%                    iCub using MATLAB.
%
% [] = INITFORWARDDYNAMICS(CONFIG) takes as input the structure CONFIG 
% containing all the configuration parameters. It has no output. The 
% forward dynamics integration will be performed following the options 
% the user specified in the initialization file.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
%% Setup the configuration and state parameters
feet_on_ground               = CONFIG.feet_on_ground;
ndof                         = CONFIG.ndof;
qjInit                       = CONFIG.qjInit;
dqjInit                      = zeros(ndof,1);
v_bInit                      = zeros(3,1);
w_omega_bInit                = zeros(3,1);

%% Contact constraints definition
if sum(feet_on_ground) == 2
    
    CONFIG.constraintLinkNames = {'l_sole','r_sole'};
    
elseif feet_on_ground(1) == 1 && feet_on_ground(2) == 0
    
    CONFIG.constraintLinkNames = {'l_sole'};
    
elseif feet_on_ground(1) == 0 && feet_on_ground(2) == 1
    
    CONFIG.constraintLinkNames = {'r_sole'};
end

CONFIG.numConstraints = length(CONFIG.constraintLinkNames);

%% Update the initial conditions
wbm_updateState(qjInit,dqjInit,[v_bInit;w_omega_bInit]);

% fixing the world reference frame w.r.t. the foot on ground position
if  feet_on_ground(1) == 1
    
    [R_bInit,x_bInit] = wbm_getWorldFrameFromFixedLink('l_sole',qjInit);
else
    [R_bInit,x_bInit] = wbm_getWorldFrameFromFixedLink('r_sole',qjInit);
end

wbm_setWorldFrame(R_bInit,x_bInit,[0 0 -9.81]')

% initial floating base pose; initial robot state
[~,BasePoseInit,~,~]          = wbm_getState();
chiInit                       = [BasePoseInit; qjInit; v_bInit; w_omega_bInit; dqjInit];

%% Initial gains
% the initial gains are defined before the numerical integration
CONFIG.gainsInit              = gains(CONFIG);

%% Initial dynamics and forward kinematics
% initial state
CONFIG.initState              = robotState(chiInit,CONFIG);
% initial dynamics
CONFIG.initDynamics           = robotDynamics(CONFIG.initState,CONFIG);
% initial forward kinematics
CONFIG.initForKinematics      = robotForKinematics(CONFIG.initState,CONFIG.initDynamics);
% CoM initial position
CONFIG.xCoMRef                = CONFIG.initForKinematics.xCoM;

%% Forward dynamics integration
CONFIG.wait       = waitbar(0,'Forward dynamics integration...');
forwardDynFunc    = @(t,chi)forwardDynamics(t,chi,CONFIG);

% either fixed step integrator or ODE15s
if CONFIG.integrateWithFixedStep == 1
    [t,chi]           = euleroForward(forwardDynFunc,chiInit,CONFIG.tEnd,CONFIG.tStart,CONFIG.sim_step);
else
    [t,chi]           = ode15s(forwardDynFunc,CONFIG.tStart:CONFIG.sim_step:CONFIG.tEnd,chiInit,CONFIG.options);
end

delete(CONFIG.wait)

%% Visualize integration results; robot simulator
CONFIG.figureCont = initVisualizer(t,chi,CONFIG);

end
