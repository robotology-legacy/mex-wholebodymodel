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
global force_feet state com_error t_previous dxi_ref_previous ddxi_ref_init;

%% Setup the configuration and state parameters
CONFIG_updated               = configRobot(CONFIG);

% initialize motor state (xi = theta/p where theta is the motor angle and p
% is the transmission ratio)
xiInit                       = CONFIG_updated.qjInit;
dxiInit                      = CONFIG_updated.dqjInit;
chi_motorInit                = [xiInit;dxiInit];
ddxi_ref_init                = zeros(CONFIG_updated.ndof,1);

%% Initial state that consider also joints elasticity
chiInit                      = [CONFIG_updated.chi_robotInit; chi_motorInit];

%% Forward dynamics integration
cont      = 0;
exit      = 0;
toll      = 0.01;
tStart    = CONFIG_updated.tStart;
t_total   = [];
chi_total = [];

while state <= 6 && exit == 0
    
    CONFIG_updated        = configRobot(CONFIG_updated);
    CONFIG_updated.wait   = waitbar(0,'Forward dynamics integration...');
    forwardDynFunc        = @(t,chi)forwardDynamics(t,chi,CONFIG_updated);

    % either fixed step integrator or ODE15s
    if CONFIG_updated.integrateWithFixedStep == 1
        [t,chi]  = euleroForward(forwardDynFunc,chiInit,CONFIG_updated.tEnd,CONFIG_updated.tStart,CONFIG_updated.sim_step);
    else
        [t,chi]  = ode15s(forwardDynFunc,tStart:CONFIG_updated.sim_step:CONFIG_updated.tEnd,chiInit,CONFIG_updated.options);
    end
    
    delete(CONFIG_updated.wait)
        
    tStart    = t(end);
    chiInit   = chi(end,:);
    cont      = cont + 1;
    t_total   = [t_total; t];
    chi_total = [chi_total; chi];

    if (strcmp(CONFIG_updated.demo_type,'yoga')) && (cont < 4) && abs(tStart -CONFIG_updated.tEnd)>toll  
        % initial state
        CONFIG_updated.initState          = robotState(transpose(chiInit),CONFIG_updated);  
        % initial dynamics
        CONFIG_updated.initDynamics       = robotDynamics(CONFIG_updated.initState,CONFIG_updated);
        % initial forward kinematics
        CONFIG_updated.initForKinematics  = robotForKinematics(CONFIG_updated.initState,CONFIG_updated.initDynamics);
    else
        exit = 1;
    end
end

%% Visualize integration results and robot simulator
% reset global variables for visualization
state            = 1;
force_feet       = 0;
com_error        = zeros(3,1);
t_previous       = CONFIG_updated.tStart;
dxi_ref_previous = dxiInit;
ddxi_ref_init    = zeros(CONFIG_updated.ndof,1);

disp('Press any key to continue')
pause()
CONFIG_updated.useForDynamicsForVisual = 1;
CONFIG_updated.figureCont              = initVisualizer(t_total,chi_total,CONFIG_updated);

%% Remove local paths
rmpath(CONFIG_updated.utility_root);
rmpath(CONFIG_updated.robot_root);
rmpath(CONFIG_updated.plots_root);
rmpath(CONFIG_updated.src_root);
rmpath(CONFIG_updated.elastic_root);
rmpath(CONFIG_updated.state_root);
rmpath(CONFIG_updated.config_root);

end
