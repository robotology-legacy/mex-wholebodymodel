function [] = initForwardDynamics(CONFIG)
%INITFORWARDDYNAMICS configures the forward dynamics integration by setting
%                    state initial conditions, control gains, etc.  
%
% [] = INITFORWARDDYNAMICS(CONFIG) takes as input the structure CONFIG 
% containing the initial user-defined parameters. It has no output. The 
% forward dynamics integration will be performed following the options 
% the user specified in the initialization script.
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
dx_bInit                     = zeros(3,1);
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

%% Configure the model using initial conditions
wbm_updateState(qjInit,dqjInit,[dx_bInit;w_omega_bInit]);

% fixing the world reference frame w.r.t. the foot on ground position
if  feet_on_ground(1) == 1
    
    [x_bInit,w_R_bInit] = wbm_getWorldFrameFromFixLnk('l_sole',qjInit);
else
    [x_bInit,w_R_bInit] = wbm_getWorldFrameFromFixLnk('r_sole',qjInit);
end

wbm_setWorldFrame(w_R_bInit,x_bInit,[0 0 -9.81]')

% initial state (floating base + joints)
[basePoseInit,~,~,~]          = wbm_getState();
chiInit                       = [basePoseInit; qjInit; dx_bInit; w_omega_bInit; dqjInit];

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

%% Forward dynamics integration
CONFIG.wait           = waitbar(0,'Forward dynamics integration...');
forwardDynFunc        = @(t,chi)forwardDynamics(t,chi,CONFIG);

% either fixed step integrator or ODE15s
if CONFIG.integrateWithFixedStep == 1
    [t,chi]           = euleroForward(forwardDynFunc,chiInit,CONFIG.tEnd,CONFIG.tStart,CONFIG.sim_step);
else
    [t,chi]           = ode15s(forwardDynFunc,CONFIG.tStart:CONFIG.sim_step:CONFIG.tEnd,chiInit,CONFIG.options);
end

delete(CONFIG.wait)

%% Visualize integration results and robot simulator
CONFIG.figureCont     = initVisualizer(t,chi,CONFIG);

end
