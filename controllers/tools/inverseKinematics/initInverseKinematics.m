function REFERENCES = initInverseKinematics(MODEL,INIT_CONDITIONS)
%INITINVERSEKINEMATICS generates the initial condition for inverse kinematics
%                      integration of robot iCub in MATLAB.
%
% INITINVERSEKINEMATICS evaluates the desired Momentum trajectory for the
% robot and generates proper joint references. The joints accelerations are
% calculated using a task-based structure: the first task is to guarantee
% the contact constraints, the second task is to follow a desired Momentum
% trajectory while the third task is to keep the initial posture of the
% robot. Positions and velocities are obtained by means of a double fixed
% step integrator.
%
% [IKIN, chiInit, figureCont] = INITINVERSEKINEMATICS(CONFIG) takes
% as input the structure CONFIG containing all the utility parameters.
% The output are the initial conditions for the forward dynamics integration,
% chiInit [13+2ndof x 1], the structure IKIN which contains the joint
% references and the figures number counter, figureCont.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% config parameters
pinv_tol               = MODEL.CONFIG.pinv_tol;
ndof                   = MODEL.ndof;
m                      = INIT_CONDITIONS.INITDYNAMICS.M(1,1);
xCoM                   = INIT_CONDITIONS.INITFORKINEMATICS.xCoM;

%% Define initial conditions for ikin integration
% this actually encompasses the fact that for all demos, the starting 
% reference velocity and acceleration are always zero, but the reference 
% joint/CoM position may be slightly different from the initial one, and
% this may cause the initial reference velocity to be different from zero.
% To avoid an undesired step reference at initial time, the following three
% task stack of task procedure is setted up.

% task #1: enforce contact constraints. This is done by computing Jc*nu = 0
% where Jc is the contact jacobian and nu is the robot + floating base
% velocity. 
constraintsDynamics    = zeros(lenght(MODEL.constraintLinkNames),1);
Jc                     = INIT_CONDITIONS.INITDYNAMICS.Jc;
pinvJc                 = pinv(Jc,pinv_tol);
NullJc                 = eye(ndof+6) - pinvJc*Jc;
% task #2: obtain a desired momentum dynamics. This is achieved comuting
% JH*nu = H_desired. the velocity obtained from this equation is then projected
% in the null space of the first task, NullJc.
JH                     = INIT_CONDITIONS.INITDYNAMICS.JH;
pinvJH                 = pinv(JH*NullJc,pinv_tol);
momentumDynamics       = [-m*(xCoM-MODEL.REFERENCES.xCoMRef);zeros(3,1)] -JH*pinvJc*constraintsDynamics;
NullJH                 = eye(ndof+6) - pinvJH*(JH*NullJc);
% task #3: postural task. The joint position is computed from the formula
% JP*nu = qj-qjRef and as the previous task, it is projected in the null
% space NullJh.
JP                     = [zeros(ndof,6) eye(ndof)];
jointDynamics          = -(qj-MODEL.REFERENCES.qjRef)-JP*(pinvJc*constraintsDynamics+NullJc*pinvJH*momentumDynamics);
pinvJP                 = pinv(JP*NullJc*NullJH,pinv_tol);

% task-base state velocity
nuThirdTask            = pinvJP*jointDynamics;
nuSecondTask           = pinvJH*momentumDynamics + NullJH*nuThirdTask;
nuFirstTask            = pinvJc*constraintsDynamics + NullJc*nuSecondTask;

% initial condition for ikin integration
chi_robotInit          = [INIT_CONDITIONS.chi_robotInit(1:(7+ndof)); nuFirstTask];


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
        [t,chi]  = euleroForward(forwardDynFunc,chi_robotInit,MODEL.CONFIG.tEnd,MODEL.CONFIG.tStart,MODEL.CONFIG.sim_step);
    else
        [t,chi]  = ode15s(forwardDynFunc,tStart:MODEL.CONFIG.sim_step:MODEL.CONFIG.tEnd,chi_robotInit,MODEL.CONFIG.options);
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
    chi_robotInit                 = transpose(chi(end,:));
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
%% Inverse kinematics integrator
CONFIG.wait            = waitbar(0,'Inverse kinematics integration...');
IKIN                   = integrateInverseKinematics(CONFIG,chi_robotInit);
delete(CONFIG.wait)

%% Visualize the results of the inverse kinematics
if CONFIG.visualize_ikin_results == 1
    
    figureCont         = visualizeInverseKin(CONFIG,IKIN);
end

