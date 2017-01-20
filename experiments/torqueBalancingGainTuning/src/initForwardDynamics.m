function [] = initForwardDynamics(CONFIG)
%INITFORWARDDYNAMICS setup the forward dynamics integration of robot iCub
%                    in MATLAB.
%
%            [] = INITFORWARDDYNAMICS(config) takes as input the structure
%            CONFIG containing all the configuration parameters. It has no
%            output. The forward dynamics integration will be performed
%            following the options the user specifies in the initialization
%            file.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
%% Config parameters
feet_on_ground               = CONFIG.feet_on_ground;
ndof                         = CONFIG.ndof;
qjInit                       = CONFIG.qjInit;

%% Contact constraints definition
if       feet_on_ground(1) == 1 && feet_on_ground(2) == 1
    
    CONFIG.constraintLinkNames   = {'l_sole','r_sole'};
    
elseif   feet_on_ground(1) == 1 && feet_on_ground(2) == 0
    
    CONFIG.constraintLinkNames   = {'l_sole'};
    
elseif   feet_on_ground(1) == 0 && feet_on_ground(2) == 1
    
    CONFIG.constraintLinkNames   = {'r_sole'};
end

CONFIG.numConstraints        = length(CONFIG.constraintLinkNames);

%% Initial conditions
dqjInit                      = zeros(ndof,1);
VelBaseInit                  = zeros(3,1);
omegaBaseWorldInit           = zeros(3,1);

% Update the initial position
wbm_updateState(qjInit,dqjInit,[VelBaseInit;omegaBaseWorldInit]);

% Fixing the world reference frame w.r.t. the foot on ground position
if  feet_on_ground(1) == 1
    
    [RotBaseInit,PosBaseInit] = wbm_getWorldFrameFromFixedLink('l_sole',qjInit);
else
    [RotBaseInit,PosBaseInit] = wbm_getWorldFrameFromFixedLink('r_sole',qjInit);
end

wbm_setWorldFrame(RotBaseInit,PosBaseInit,[0 0 -9.81]')

% Initial base pose; initial robot state
[~,BasePoseInit,~,~]          = wbm_getState();
chiInit                       = [BasePoseInit; qjInit; VelBaseInit; omegaBaseWorldInit; dqjInit];
CONFIG.initState              = robotState(chiInit,CONFIG);

%% Initial gains
CONFIG.gainsInit              = gains(CONFIG);

%% Joint references with inverse kinematics
if CONFIG.jointRef_with_ikin == 1
    
    [CONFIG.ikin,chiInit,CONFIG.figureCont]  = initInverseKinematics(CONFIG);
    CONFIG.initState                         = robotState(chiInit,CONFIG);
end

%% Initial dynamics and forward kinematics
% Initial dynamics
CONFIG.initDynamics              = robotDynamics(CONFIG.initState,CONFIG);
% Initial forward kinematics
CONFIG.initForKinematics         = robotForKinematics(CONFIG.initState,CONFIG.initDynamics);
CONFIG.xCoMRef                   = CONFIG.initForKinematics.xCoM;

%% %%%%%%%%%%%%% LINEARIZATION DEBUG AND STABILITY ANALYSIS %%%%%%%%%%%% %%
if CONFIG.linearizationDebug == 1
    
    % the initial configuration is changed by a small delta, but the references
    % are not updated. In this way the robot will move to the reference position
    qjInit(1:13)        = qjInit(1:13)-1*pi/180;
    
    wbm_updateState(qjInit,dqjInit,[VelBaseInit;omegaBaseWorldInit]);
    
    if  feet_on_ground(1) == 1
        
        [RotBaseInit,PosBaseInit] = wbm_getWorldFrameFromFixedLink('l_sole',qjInit);
    else
        [RotBaseInit,PosBaseInit] = wbm_getWorldFrameFromFixedLink('r_sole',qjInit);
    end
    
    wbm_setWorldFrame(RotBaseInit,PosBaseInit,[0 0 -9.81]')
    [~,BasePoseInit,~,~]         = wbm_getState();
    chiInit                      = [BasePoseInit; qjInit; VelBaseInit; omegaBaseWorldInit; dqjInit];
    CONFIG.initState             = robotState(chiInit,CONFIG);
    CONFIG.initDynamics          = robotDynamics(CONFIG.initState,CONFIG);
    CONFIG.initForKinematics     = robotForKinematics(CONFIG.initState,CONFIG.initDynamics);
    
end

% the system is linearized around the initial position, to verify the stability
CONFIG.linearization         = jointSpaceLinearization(CONFIG,qjInit);

%% Gains tuning procedure
if CONFIG.gains_tuning == 1
    
    [gainsKron,CONFIG.linearization.KS,CONFIG.linearization.KD]    = gainsTuning(CONFIG.linearization,CONFIG);
    CONFIG.gain                                                    = gainsConstraint(gainsKron,CONFIG.linearization,CONFIG);
else
    CONFIG.gain  = CONFIG.gainsInit;
end

%% FORWARD DYNAMICS INTEGRATION
CONFIG.wait       = waitbar(0,'Forward dynamics integration...');
forwardDynFunc    = @(t,chi)forwardDynamics(t,chi,CONFIG);

% Either fixed step integrator or ODE15s
if CONFIG.integrateWithFixedStep == 1
    
    [t,chi]           = euleroForward(forwardDynFunc,chiInit,CONFIG.tEnd,CONFIG.tStart,CONFIG.sim_step);
else
    [t,chi]           = ode15s(forwardDynFunc,CONFIG.tStart:CONFIG.sim_step:CONFIG.tEnd,chiInit,CONFIG.options);
end

delete(CONFIG.wait)

%% VISUALIZATION
CONFIG.figureCont = initVisualizer(t,chi,CONFIG);

end
