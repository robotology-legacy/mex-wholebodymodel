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
global force_feet state com_error;

%% Setup the configuration and state parameters
CONFIG = configRobot(CONFIG);

% initialize motor state (xi = theta/p where theta is the motor angle and p
% is the transmission ratio)
xiInit                       = CONFIG.qjInit;
dxiInit                      = CONFIG.dqjInit;
chi_motorInit                = [xiInit;dxiInit];

%% Initial state that consider also joints elasticity
chiInit                      = [CONFIG.chi_robotInit; chi_motorInit];

%% Forward dynamics integration
cont      = 0;
exit      = 0;
toll      = 0.01;
tStart    = CONFIG.tStart;
t_total   = [];
chi_total = [];

while state <= 3 && exit == 0
    
    CONFIG            = configRobot(CONFIG);
    CONFIG.wait       = waitbar(0,'Forward dynamics integration...');
    forwardDynFunc    = @(t,chi)forwardDynamics(t,chi,CONFIG);

    % either fixed step integrator or ODE15s
    if CONFIG.integrateWithFixedStep == 1
        [t,chi]  = euleroForward(forwardDynFunc,chiInit,CONFIG.tEnd,CONFIG.tStart,CONFIG.sim_step);
    else
        [t,chi]  = ode15s(forwardDynFunc,tStart:CONFIG.sim_step:CONFIG.tEnd,chiInit,CONFIG.options);
    end
    
    delete(CONFIG.wait)
        
    tStart    = t(end);
    chiInit   = chi(end,:);
    cont      = cont + 1;
    t_total   = [t_total; t];
    chi_total = [chi_total; chi];

    % initial state
    CONFIG.initState          = robotState(chiInit',CONFIG);  
    % initial dynamics
    CONFIG.initDynamics       = robotDynamics(CONFIG.initState,CONFIG);
    % initial forward kinematics
    CONFIG.initForKinematics  = robotForKinematics(CONFIG.initState,CONFIG.initDynamics);

    if (strcmp(CONFIG.demo_type,'yoga')) && (cont < 10) && abs(tStart -CONFIG.tEnd)>toll  
    else
        exit = 1;
    end
end

%% Visualize integration results and robot simulator
% reset global variables for visualization
state      = 1;
force_feet = 0;
com_error  = zeros(3,1);

pause()
CONFIG.figureCont = initVisualizer(t_total,chi_total,CONFIG);

%% Remove local paths
rmpath(CONFIG.utility_root);
rmpath(CONFIG.robot_root);
rmpath(CONFIG.plots_root);
rmpath(CONFIG.src_root);
rmpath(CONFIG.elastic_root);
rmpath(CONFIG.state_root);
rmpath(CONFIG.config_root);

end
