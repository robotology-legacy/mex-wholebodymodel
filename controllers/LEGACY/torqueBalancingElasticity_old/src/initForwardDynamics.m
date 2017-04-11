function [] = initForwardDynamics(CONFIG)
%INITFORWARDDYNAMICS defines the robot initial condition for forward dynamics 
%                    integration. 
%
% [] = INITFORWARDDYNAMICS(CONFIG) takes as input the structure CONFIG 
% containing the configuration parameters. It has no output. Forward dynamics
% integration will be performed following the options the user specified in 
% the initialization script.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
global state;

%% Use configuration to define robot initial conditions
INIT_CONDITIONS              = configureRobot(CONFIG);
% initialize motor state (xi = theta*eta where theta is the motor angle and 
% eta is the transmission ratio)
xiInit                       = INIT_CONDITIONS.qjInit;
dxiInit                      = INIT_CONDITIONS.dqjInit;
% motor state
chi_motorInit                = [xiInit;dxiInit];

%% CoM and joints references
INIT_CONDITIONS.qjRef        = INIT_CONDITIONS.qjInit;
INIT_CONDITIONS.xCoMRef      = INIT_CONDITIONS.initForKinematics.xCoM;
% if the robot is balancing on one foot, the CoM is setted to be in the center of support polygon
if INIT_CONDITIONS.CONFIG.feet_on_ground(2) == 0
    INIT_CONDITIONS.xCoMRef([1,2]) = INIT_CONDITIONS.initForKinematics.poseLFoot_qt([1,2]);
elseif INIT_CONDITIONS.CONFIG.feet_on_ground(1) == 0
    INIT_CONDITIONS.xCoMRef([1,2]) = INIT_CONDITIONS.initForKinematics.poseRFoot_qt([1,2]);
end

%% STABILITY TEST
if CONFIG.upperBody_stepReference
    % the initial joints configuration is changed by a small delta, but the references
    % are not updated. In this way the robot will move to the reference position
    delta            = CONFIG.stepAmplitude;
    qjInit           = INIT_CONDITIONS.qjInit;
    qjInit(1:13)     = INIT_CONDITIONS.qjInit(1:13) + delta*pi/180;
    % configure the model using the new initial conditions
    dx_bInit         = zeros(3,1);
    w_omega_bInit    = zeros(3,1);     
    % update robot state
    wbm_updateState(qjInit,INIT_CONDITIONS.dqjInit,[dx_bInit;w_omega_bInit]);
    % fixing the world reference frame w.r.t. the foot on ground position
    [x_bInit,w_R_bInit]  = wbm_getWorldFrameFromFixLnk(INIT_CONDITIONS.CONFIG.constraintLinkNames{1},qjInit);
    wbm_setWorldFrame(w_R_bInit,x_bInit,[0 0 -9.81]')
    % get current base pose
    [basePoseInit,~,~,~] = wbm_getState();
    % initial robot state (floating base + joints)
    INIT_CONDITIONS.chi_robotInit = [basePoseInit; qjInit; [dx_bInit;w_omega_bInit]; INIT_CONDITIONS.dqjInit];
    
    %% Update state, dynamics and forward kinematics
    % initial state
    INIT_CONDITIONS.initState         = robotState(INIT_CONDITIONS.chi_robotInit,INIT_CONDITIONS.CONFIG);
    % initial dynamics
    INIT_CONDITIONS.initDynamics      = robotDynamics(INIT_CONDITIONS.initState,INIT_CONDITIONS.CONFIG);
    % initial forward kinematics
    INIT_CONDITIONS.initForKinematics = robotForKinematics(INIT_CONDITIONS.initState,INIT_CONDITIONS.initDynamics);   
end

%% INITIAL SYSTEM STATE (MOTORS + ROBOT + FLOATING BASE)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
chiInit  = [INIT_CONDITIONS.chi_robotInit; chi_motorInit];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Forward dynamics integration
cont      = 0;
exit      = 0;
toll      = INIT_CONDITIONS.CONFIG.sim_step;
tStart    = INIT_CONDITIONS.CONFIG.tStart;
t_total   = [];
chi_total = [];
% define the exit state 
exitState = 1;
if strcmp(CONFIG.demo_type,'yoga')
    if INIT_CONDITIONS.CONFIG.demoOnlyRightFoot || INIT_CONDITIONS.CONFIG.demoAlsoRightFoot
        exitState = 13;
    else
        exitState = 7;
    end
end

while exit == 0
    % waitbar
    INIT_CONDITIONS.wait   = waitbar(0,'Forward dynamics integration...');
    % initialize iDyntree simulator
    if INIT_CONDITIONS.CONFIG.visualize_robot_simulator_ONLINE
        INIT_CONDITIONS.VISUALIZER = configureRobotSimulator(chiInit,INIT_CONDITIONS.CONFIG);
    end 
    % function to be integrated
    forwardDynFunc = @(t,chi) forwardDynamics(t,chi,INIT_CONDITIONS);
    % either fixed step integrator or ODE15s
    if INIT_CONDITIONS.CONFIG.integrateWithFixedStep == 1
        [t,chi]  = euleroForward(forwardDynFunc,chiInit,INIT_CONDITIONS.CONFIG.tEnd,INIT_CONDITIONS.CONFIG.tStart,INIT_CONDITIONS.CONFIG.sim_step);
    else
        [t,chi]  = ode15s(forwardDynFunc,tStart:INIT_CONDITIONS.CONFIG.sim_step:INIT_CONDITIONS.CONFIG.tEnd,chiInit,INIT_CONDITIONS.CONFIG.options);
    end
    % delete waitbar
    delete(INIT_CONDITIONS.wait)
    if INIT_CONDITIONS.CONFIG.visualize_robot_simulator_ONLINE
        INIT_CONDITIONS.VISUALIZER.viz.close();
    end 
    %% Update configuration for the next loop     
    % update robot configuration, feet on ground, gains and references. THE
    % INITIAL STATE IS NOT UPDATED
    INIT_CONDITIONS = configureRobot(INIT_CONDITIONS.CONFIG);
    tStart          = t(end);
    chiInit         = transpose(chi(end,:));
    cont            = cont + 1;
    t_total         = [t_total; t];
    chi_total       = [chi_total; chi];
    % define the coditions for exiting the loop
    if (strcmp(INIT_CONDITIONS.CONFIG.demo_type,'yoga')) && (state <= exitState) && abs(tStart -INIT_CONDITIONS.CONFIG.tEnd)>toll  
        % update robot state, dynamics, forward kinematics
        INIT_CONDITIONS.initState          = robotState(chiInit,INIT_CONDITIONS.CONFIG);  
        INIT_CONDITIONS.initDynamics       = robotDynamics(INIT_CONDITIONS.initState,INIT_CONDITIONS.CONFIG);
        INIT_CONDITIONS.initForKinematics  = robotForKinematics(INIT_CONDITIONS.initState,INIT_CONDITIONS.initDynamics);
        INIT_CONDITIONS.qjRef              = INIT_CONDITIONS.qjInit;
        INIT_CONDITIONS.xCoMRef            = INIT_CONDITIONS.initForKinematics.xCoM;
        % if the robot is balancing on one foot, the CoM is setted to be in the center of support polygon
        if INIT_CONDITIONS.CONFIG.feet_on_ground(2) == 0
            INIT_CONDITIONS.xCoMRef([1,2]) = INIT_CONDITIONS.initForKinematics.poseLFoot_qt([1,2]);
        elseif INIT_CONDITIONS.CONFIG.feet_on_ground(1) == 0
            INIT_CONDITIONS.xCoMRef([1,2]) = INIT_CONDITIONS.initForKinematics.poseRFoot_qt([1,2]);
        end
    else
        exit = 1;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize integration results and robot simulator
disp('Press any key to continue')
pause()
% save('t_total','t_total')
% save('chi_total','chi_total')
% save('INIT_CONDITIONS','INIT_CONDITIONS')
INIT_CONDITIONS.CONFIG.figureCont = 1;
INIT_CONDITIONS.figureCont = initVisualizer(t_total,chi_total,INIT_CONDITIONS);

%% Remove local paths
rmpath(INIT_CONDITIONS.tools_root);
rmpath(INIT_CONDITIONS.robot_root);
rmpath(INIT_CONDITIONS.plots_root);
rmpath(INIT_CONDITIONS.src_root);
rmpath(INIT_CONDITIONS.elastic_root);
rmpath(INIT_CONDITIONS.state_root);
rmpath(INIT_CONDITIONS.config_root);

end
