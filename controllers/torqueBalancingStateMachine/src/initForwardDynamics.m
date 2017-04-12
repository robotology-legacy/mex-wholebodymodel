function [] = initForwardDynamics(CONFIG)
%INITFORWARDDYNAMICS initialize the forward dynamics integration. 
%
% Format:  [] = INITFORWARDDYNAMICS(CONFIG)
%
% Inputs:  - CONFIG: it is the structure containing all user-defined 
%                    configuration parameters. 
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% initialize global variables for state machine
global state prevTimeIndex; 

%% %%%%%%%%%%%%%%%%%%%%% MODEL AND INIT CONDITIONS %%%%%%%%%%%%%%%%%%%%% %%
% use the configuration to initialize robot model and initial conditions
state                    = 1; 
[MODEL,INIT_CONDITIONS]  = configureRobot(CONFIG);

% initialize motor state. It is performed the following change of variable:
%     xi = theta*eta 
% where theta are the motor positions and eta is the transmission ratio
if MODEL.CONFIG.use_SEA
    xiInit               = INIT_CONDITIONS.qjInit;
    dxiInit              = INIT_CONDITIONS.dqjInit;
    chi_motorInit        = [xiInit;dxiInit];
end

%% %%%%%%%%%%%%%%%%% UPDATE JOINT REFERENCES (IKIN) %%%%%%%%%%%%%%%%%%%% %%
if MODEL.CONFIG.use_ikinSolver  
    MODEL.REFERENCES     = initInverseKinematics(MODEL,INIT_CONDITIONS);
end

%% %%%%%%%%%%%%%%%%%%%%%% GAIN TUNING PROCEDURE %%%%%%%%%%%%%%%%%%%%%%%% %%
if MODEL.CONFIG.use_gainTuning  
    % the system is linearized around the initial position
    MODEL.linearization  = jointSpaceLinearization(MODEL,INIT_CONDITIONS.qjInit);
    MODEL.GAINTUNING     = gainsTuning(MODEL);
end

%% %%%%%%%% STABILITY TEST: RESPONSE TO STEP REFERENCE %%%%%%%%%%%%%%%%% %%
if MODEL.CONFIG.stepReference
    % the initial joints configuration is changed by a small delta, but the references
    % are not updated. In this way the robot will move to the reference position
    INIT_CONDITIONS      = addStepReference(MODEL,INIT_CONDITIONS);
    % the system is linearized around the initial position (for checking
    % stability and verifying the linearization
    MODEL.linearization  = jointSpaceLinearization(MODEL,INIT_CONDITIONS.qjInit);
end

%% %%%%%%%%%%%%%%%%%%%%% INITIAL SYSTEM STATE %%%%%%%%%%%%%%%%%%%%%%%%%% %%
if MODEL.CONFIG.use_SEA
    chiInit  = [INIT_CONDITIONS.chi_robotInit; chi_motorInit];
else
    chiInit  = INIT_CONDITIONS.chi_robotInit;
end

%% Visualize forward dynamics integration results
if MODEL.CONFIG.enable_visualTool
    % initialize time index
    prevTimeIndex        = 0;
    % initialize all variables (the advantage of this method is the
    % variables are preallocated)
    ndof                 = MODEL.ndof;
    sizeTime             = length(MODEL.timeTot);
    MODEL.DATA.dxi       = zeros(ndof,sizeTime);
    MODEL.DATA.dxi_ref   = zeros(ndof,sizeTime);
    MODEL.DATA.qj        = zeros(ndof,sizeTime);
    MODEL.DATA.qjRef     = zeros(ndof,sizeTime);
    MODEL.DATA.xCoM      = zeros(3,sizeTime);
    MODEL.DATA.poseFeet  = zeros(12,sizeTime);
    MODEL.DATA.H         = zeros(6,sizeTime);
    MODEL.DATA.HRef      = zeros(6,sizeTime);
    MODEL.DATA.fc        = zeros(12,sizeTime);
    MODEL.DATA.tau_xi    = zeros(ndof,sizeTime);
    MODEL.DATA.xCoMDes   = zeros(3,sizeTime);
    MODEL.DATA.timeIndex = zeros(sizeTime,1);
    
    % create a folder and save stored values inside it
    outputDir = './media';
    if (~exist(outputDir, 'dir'))
        mkdir(outputDir);
    end
    % save the data in a .mat file
    save('./media/storedValues','MODEL.DATA.dxi','MODEL.DATA.dxi_ref','MODEL.DATA.qj', ...
         'MODEL.DATA.qjRef','MODEL.DATA.xCoM','MODEL.DATA.poseFeet','MODEL.DATA.H', ...
         'MODEL.DATA.HRef','MODEL.DATA.fc','MODEL.DATA.tau_xi','MODEL.DATA.xCoMDes', ...
         'MODEL.DATA.timeIndex','-v7.3');
    % temporarily disable online visualizer
    MODEL.disableVisForGraphics = true;
    % call the forward dynamics funtion at tStart, for initializing the
    % stored values
    forwardDynamics(MODEL.timeTot(1),chiInit,MODEL,INIT_CONDITIONS);
end
% reset online visualization to default
MODEL.disableVisForGraphics     = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% %%%%%%%%%%%%%%%%% FORWARD DYNAMICS INTEGRATION %%%%%%%%%%%%%%%%%%%%%% %%
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
