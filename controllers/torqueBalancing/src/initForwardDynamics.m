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

%% %%%%%%%%%%%%%%%%% UPDATE JOINT REFERENCES (IKIN) %%%%%%%%%%%%%%%%%%%% %%
if MODEL.CONFIG.use_ikinSolver  
    MODEL.REFERENCES     = initInverseKinematics(MODEL,INIT_CONDITIONS);
end

%% %%%%%%%%%% STABILITY TEST: RESPONSE TO STEP REFERENCE %%%%%%%%%%%%%%% %%
if MODEL.CONFIG.stepReference
    % the initial joints configuration is changed by a small delta, but the references
    % are not updated. In this way the robot will move to the reference position
    INIT_CONDITIONS      = addStepReference(MODEL,INIT_CONDITIONS);
end
% linearization tool is shared between the step reference and the gain
% tuning procedure
if MODEL.CONFIG.use_gainTuning  || MODEL.CONFIG.stepReference 
    % the system is linearized around the initial position
    MODEL.LINEARIZATION  = jointSpaceLinearization(MODEL,INIT_CONDITIONS);
end

%% %%%%%%%%%%%%%%%%%%%%%% GAIN TUNING PROCEDURE %%%%%%%%%%%%%%%%%%%%%%%% %%
if MODEL.CONFIG.use_gainTuning  
    [MODEL.GAINS,MODEL.LINEARIZATION.KS,MODEL.LINEARIZATION.KD] = gainsTuning(MODEL.LINEARIZATION,MODEL);
end

%% %%%%%%%%%%%%%%%%%%%%% INITIAL SYSTEM STATE %%%%%%%%%%%%%%%%%%%%%%%%%% %%
if MODEL.CONFIG.use_SEA
    % in case motor dynamics is considered, the state is extended for
    % adding the motor state. It is performed the following change of variable:
    %     xi = theta*eta 
    % where theta are the motor positions and eta is the transmission ratio
    xiInit          = INIT_CONDITIONS.INITSTATE.qj;
    dxiInit         = INIT_CONDITIONS.INITSTATE.dqj;
    chi_motorInit   = [xiInit;dxiInit];
    chiInit         = [INIT_CONDITIONS.chi_robotInit; chi_motorInit];
else
    chiInit         = INIT_CONDITIONS.chi_robotInit;
end

%% %%%%%%%%%%%%%%%%%%%%%% VISUALIZATION TOOL %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
if MODEL.CONFIG.enable_visualTool
    % initialize time index
    prevTimeIndex = 0;
    % initialize all variables to plot (the advantage of this method is the
    % variables are preallocated)
    ndof      = MODEL.ndof;   
    sizeTime  = length(MODEL.timeTot);
    dxi       = zeros(ndof,sizeTime);  %#ok<NASGU>
    dxi_ref   = zeros(ndof,sizeTime);  %#ok<NASGU>
    qj        = zeros(ndof,sizeTime);  %#ok<NASGU>
    qjRef     = zeros(ndof,sizeTime);  %#ok<NASGU>
    xCoM      = zeros(3,sizeTime);     %#ok<NASGU>
    xCoMDes   = zeros(3,sizeTime);     %#ok<NASGU>    
    poseFeet  = zeros(12,sizeTime);    %#ok<NASGU>
    H         = zeros(6,sizeTime);     %#ok<NASGU>
    HRef      = zeros(6,sizeTime);     %#ok<NASGU>
    fc        = zeros(12,sizeTime);    %#ok<NASGU>
    tau_xi    = zeros(ndof,sizeTime);  %#ok<NASGU>
    timeIndex = zeros(sizeTime,1);     %#ok<NASGU>
    
    % save the data in a .mat file
    save('./media/storedValuesFwdDyn','dxi','dxi_ref','qj','qjRef','xCoM',...
         'poseFeet','H','HRef','fc','tau_xi','xCoMDes','timeIndex','-v7.3');
    % temporarily disable online visualizer
    MODEL.disableVisForGraphics = true;
    % call the forward dynamics funtion at tStart, for initializing all the
    % stored values
    forwardDynamics(MODEL.timeTot(1),chiInit,MODEL,INIT_CONDITIONS);
    % reset online visualization to default
    MODEL.disableVisForGraphics = false;
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% %%%%%%%%%%%%%%%%% FORWARD DYNAMICS INTEGRATION %%%%%%%%%%%%%%%%%%%%%% %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% configuration parameters
exitLoop     = 0;
time_toll    = MODEL.CONFIG.sim_step;
tStart       = MODEL.CONFIG.tStart; %#ok<NASGU>
t_total      = [];
chi_total    = [];

% define the conditions for exiting the loop 
exitState    = 0;
% the condition is updated in case of the yoga and standup demos
if  strcmp(CONFIG.demo_type,'yoga')
    if MODEL.CONFIG.demoOnlyRightFoot || MODEL.CONFIG.demoAlsoRightFoot
        exitState = 13;
    else
        exitState = 7;
    end
end

% integration loop. To deal with discrete events (like impacts), the
% integrator needs to stop and restart with new initial conditions
while exitLoop == 0
    %% Initialization
    % initialize the waitbar
    MODEL.wait = waitbar(0,'1','Name','Forward dynamics integration in progress...',...
                         'CreateCancelBtn','setappdata(gcbf,''canceling'',1); delete(gcbf);');
    set(MODEL.wait, 'Units', 'Pixels', 'Position', [800 500 365 100])
    % initialize iDyntree simulator for online visualization of the robot
    if MODEL.CONFIG.visualize_robot_simulator_ONLINE
        MODEL.VISUALIZER = configureSimulator();
    end 
    
    %% Function to be integrated
    forwardDynFunc = @(t,chi) forwardDynamics(t,chi,MODEL,INIT_CONDITIONS); %#ok<NASGU>
    % forward dynamics integrator. Suggested integrators: ODE15s and
    % ODE23t. The user can select also other integrators, but they are not recommended.
    selectedOdeSolver  = [CONFIG.odeSolver,'(forwardDynFunc,tStart:MODEL.CONFIG.sim_step:MODEL.CONFIG.tEnd,chiInit,MODEL.CONFIG.options)'];
    disp('[Forward dynamics]: integration in progress...')
    [t,chi]            = eval(selectedOdeSolver);  
    
    % delete waitbar and visualizer (they will be updated again in the loop)
    if MODEL.CONFIG.visualize_robot_simulator_ONLINE
        MODEL.VISUALIZER.viz.close();
    end 
    delete(MODEL.wait)
    
    %% Update initial conditions for the next loop  
    % update robot configuration, feet on ground, gains and references. The
    % initial state is now the ending state in previous integration loop.
    % The same for the initial integration time.
    tStart                  = t(end);
    chiInit                 = transpose(chi(end,:)); %#ok<NASGU>
    chi_robotInit           = transpose(chi(end,1:13+2*MODEL.ndof));
    [MODEL,INIT_CONDITIONS] = configureRobot(CONFIG,chi_robotInit,MODEL,INIT_CONDITIONS);
    % update robot state and time total vector
    t_total                 = [t_total; t];      %#ok<AGROW>
    chi_total               = [chi_total; chi];  %#ok<AGROW>
    
    %% Define the coditions for exiting the loop
    % second condition exploits the fact that it is a nonsense to go into a
    % new integration loop if the starting time is really close to the
    % ending time (< integration step)
    if  state > exitState || abs(tStart - MODEL.CONFIG.tEnd) < time_toll  
        exitLoop            = 1;
    end
end
disp('[Forward dynamics]: integration completed')

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%% VISUALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%% %%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

% % % % % % % % initialize visualizer
% % % % % % % MODEL.figureCont = 1;
% % % % % % % MODEL.figureCont = initVisualizer(t_total,chi_total,MODEL);
% % % % % % % 
% % % % % % % %% Remove local paths
% % % % % % % rmpath(MODEL.CONFIG.tools_root);
% % % % % % % rmpath(MODEL.CONFIG.robot_root);
% % % % % % % rmpath(MODEL.CONFIG.plots_root);
% % % % % % % rmpath(MODEL.CONFIG.src_root);
% % % % % % % rmpath(MODEL.CONFIG.elastic_root);
% % % % % % % rmpath(MODEL.CONFIG.state_root);
% % % % % % % rmpath(MODEL.CONFIG.config_root);

end
