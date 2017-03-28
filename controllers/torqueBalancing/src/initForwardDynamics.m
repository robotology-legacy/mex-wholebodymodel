function [] = initForwardDynamics(CONFIG)
%INITFORWARDDYNAMICS defines the robot initial condition for forward dynamics 
%                    integration. 
%
% Format: [] = INITFORWARDDYNAMICS(CONFIG)
%
% Inputs:  - CONFIG it is the structure containing all user-defined parameters. 
%
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
global state prevTimeIndex;

%% Use configuration to define robot model and initial conditions
[MODEL,INIT_CONDITIONS]      = configureRobot(CONFIG);

%% Initialize motor state 
% it is assumed the following change of variable: xi = theta*eta where 
% theta are the motor positions and eta is the transmission ratio
xiInit                       = INIT_CONDITIONS.qjInit;
dxiInit                      = INIT_CONDITIONS.dqjInit;
% motor initial state
chi_motorInit                = [xiInit;dxiInit];

%% Initialize CoM and joints references
INIT_CONDITIONS.qjRef        = INIT_CONDITIONS.qjInit;
INIT_CONDITIONS.xCoMRef      = INIT_CONDITIONS.INITFORKINEMATICS.xCoM;
% CoM correction in case of one foot balancing: the CoM is setted to be in 
% the center of support polygon 
if MODEL.CONFIG.feet_on_ground(2) == 0
    % left foot balancing 
    INIT_CONDITIONS.xCoMRef([1,2]) = INIT_CONDITIONS.INITFORKINEMATICS.poseLFoot_qt([1,2]);
elseif MODEL.CONFIG.feet_on_ground(1) == 0
    % right foot balancing
    INIT_CONDITIONS.xCoMRef([1,2]) = INIT_CONDITIONS.INITFORKINEMATICS.poseRFoot_qt([1,2]);
end

%% STABILITY TEST: RESPONSE TO STEP REFERENCE
if MODEL.CONFIG.upperBody_stepReference
    % the initial joints configuration is changed by a small delta, but the references
    % are not updated. In this way the robot will move to the reference position
    delta            = MODEL.CONFIG.stepAmplitude;
    qjInit           = INIT_CONDITIONS.qjInit;
    qjInit(1:13)     = INIT_CONDITIONS.qjInit(1:13) + delta*pi/180;
    % update robot state using the new initial conditions  
    wbm_updateState(qjInit,INIT_CONDITIONS.dqjInit,[INIT_CONDITIONS.dx_bInit;INIT_CONDITIONS.w_omega_bInit]);
    % fixing the world reference frame w.r.t. the foot on ground position
    [x_bInit,w_R_bInit]  = wbm_getWorldFrameFromFixLnk(INIT_CONDITIONS.CONFIG.constraintLinkNames{1},qjInit);
    wbm_setWorldFrame(w_R_bInit,x_bInit,[0 0 -9.81]')
    % get current base pose
    [basePoseInit,~,~,~] = wbm_getState();
    % new initial robot state (floating base + joints)
    INIT_CONDITIONS.chi_robotInit = [basePoseInit; qjInit; [INIT_CONDITIONS.dx_bInit;INIT_CONDITIONS.w_omega_bInit]; INIT_CONDITIONS.dqjInit];
    
    %% Update state demux, dynamics and forward kinematics
    % initial state
    INIT_CONDITIONS.INITSTATE         = robotState(INIT_CONDITIONS.chi_robotInit,MODEL);
    % initial dynamics
    INIT_CONDITIONS.INITDYNAMICS      = robotDynamics(INIT_CONDITIONS.initState,MODEL);
    % initial forward kinematics
    INIT_CONDITIONS.INITFORKINEMATICS = robotForKinematics(INIT_CONDITIONS.INITSTATE,INIT_CONDITIONS.INITDYNAMICS);   
end

%% %%%%%%% INITIAL SYSTEM STATE (MOTORS + ROBOT + FLOATING BASE) %%%%%%% %%
chiInit  = [INIT_CONDITIONS.chi_robotInit; chi_motorInit];

%% Visualize forward dynamics integration results
if MODEL.CONFIG.visualize_integration_results
    % initialize time index
    prevTimeIndex = 0;
    % initialize all variables (the advantage of this method is the vectors
    % are preallocated
    ndof      = MODEL.ndof;
    sizeTime  = length(MODEL.timeTot);
    dxi       = zeros(ndof,sizeTime);
    dxi_ref   = zeros(ndof,sizeTime);
    qj        = zeros(ndof,sizeTime);
    qjRef     = zeros(ndof,sizeTime);
    xCoM      = zeros(3,sizeTime);
    poseFeet  = zeros(12,sizeTime);
    H         = zeros(6,sizeTime);
    HRef      = zeros(6,sizeTime);
    fc        = zeros(12,sizeTime);
    tau_xi    = zeros(ndof,sizeTime);
    xCoMDes   = zeros(3,sizeTime);
    timeIndex = zeros(sizeTime,1);
    % create a folder and save values inside it
    outputDir = './media';
    if (~exist(outputDir, 'dir'))
        mkdir(outputDir);
    end
    % save the data in a .mat file
    save('./media/storedValues','dxi','dxi_ref','qj','qjRef','xCoM','poseFeet', ...
         'H','HRef','fc','tau_xi','xCoMDes','timeIndex','-v7.3');
    % temporarily disable online visualizer
    MODEL.disableVisForGraphics = 1;
    % call the forward dynamics funtion at tStart, for initializing the
    % structure GRAPHICS
    forwardDynamics(MODEL.timeTot(1),chiInit,MODEL,INIT_CONDITIONS);
end
MODEL.disableVisForGraphics = 0;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% %%%%%%%%%%%%%%%%% Forward dynamics integration %%%%%%%%%%%%%%%%%%%%%% %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
exit         = 0;
time_toll    = MODEL.CONFIG.sim_step;
tStart       = MODEL.CONFIG.tStart;
t_total      = [];
chi_total    = [];

% define the state condition for exiting the loop 
exitState    = 0;
if  strcmp(CONFIG.demo_type,'yoga')
    if MODEL.CONFIG.demoOnlyRightFoot || MODEL.CONFIG.demoAlsoRightFoot
        exitState = 13;
    else
        exitState = 7;
    end
end
% integration loop. To deal with discrete events like impacts, the
% integrator stops and restarts with new initial conditions
while exit == 0
    % initialize iDyntree simulator for online visualization of the robot
    if MODEL.CONFIG.visualize_robot_simulator_ONLINE
        MODEL.VISUALIZER = configureSimulator();
    end 
    %% Function to be integrated
    forwardDynFunc = @(t,chi) forwardDynamics(t,chi,MODEL,INIT_CONDITIONS);
    % either fixed step integrator or ODE15s
    if MODEL.CONFIG.integrateWithFixedStep == 1
        [t,chi]  = euleroForward(forwardDynFunc,chiInit,MODEL.CONFIG.tEnd,MODEL.CONFIG.tStart,MODEL.CONFIG.sim_step);
    else
        [t,chi]  = ode15s(forwardDynFunc,tStart:MODEL.CONFIG.sim_step:MODEL.CONFIG.tEnd,chiInit,MODEL.CONFIG.options);
    end   
    % delete waitbar and visualizer
    if MODEL.CONFIG.visualize_robot_simulator_ONLINE
        MODEL.VISUALIZER.viz.close();
    end 
    delete(MODEL.wait)
    
    %% UPDATE INITIAL CONDITIONS FOR NEXT LOOP    
    % update robot configuration, feet on ground, gains and references. The
    % initial state is now the ending state in previous integration loop.
    % The same for the initial integration time.
    tStart                  = t(end);
    chiInit                 = transpose(chi(end,:));
    [MODEL,INIT_CONDITIONS] = configureRobot(MODEL.CONFIG);
     
    %% Update robot state and time vector
    t_total         = [t_total; t];
    chi_total       = [chi_total; chi];
    
    %% Define the coditions for exiting the loop
    if  state > exitState || abs(tStart - MODEL.CONFIG.tEnd) < time_toll  
        exit = 1;
        % delete waitbar
        delete(MODEL.wait)
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% %%%%%%%% Visualize integration results and robot simulator %%%%%%%%%% %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
disp('Forward dynamics integration completed. Press any key to continue')
pause();
% initialize visualizer
MODEL.figureCont = 1;
MODEL.figureCont = initVisualizer(t_total,chi_total,MODEL);

%% Remove local paths
rmpath(MODEL.CONFIG.tools_root);
rmpath(MODEL.CONFIG.robot_root);
rmpath(MODEL.CONFIG.plots_root);
rmpath(MODEL.CONFIG.src_root);
rmpath(MODEL.CONFIG.elastic_root);
rmpath(MODEL.CONFIG.state_root);
rmpath(MODEL.CONFIG.config_root);

end
