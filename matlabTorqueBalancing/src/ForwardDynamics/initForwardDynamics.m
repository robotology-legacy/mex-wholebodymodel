function [] = initForwardDynamics(config)
%INITFORWARDDYNAMICS setup the integration of the forward dynamics of
%                    the robot iCub in MATLAB.
%   [] = INITFORWARDDYNAMICS(config) takes as input the structure
%   CONFIG containing all the utility parameters. It has no output. The 
%   forward dynamics integration will be performed following the options
%   the user specified in the initialization file.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------

% this call the script which contains the figure setup
plot_set
config.ContFig            = 1;
config.MassCorr           = 0.1;
config.postCorrection     = 1;

%% Correction of user-defined paths
config.SoTController      = strcmp(config.BalancingController,'StackOfTask');
config.JSController       = strcmp(config.BalancingController,'JointSpace');
config.linearize          = 0;

if config.JSController == 1
    
config.linearize_for_stability_analysis     = 0;                       
config.linearize_for_gains_tuning           = 0;                           
config.use_QPsolver                         = 0;
config.visualize_stability_analysis_results = 0;
config.jointRef_with_ikin                   = 1;
end

if config.linearize_for_gains_tuning == 1 || config.linearize_for_stability_analysis == 1
    
config.linearize           = 1;    
end

if  config.linearize == 1 && config.visualize_stability_analysis_results  == 1
    
config.jointRef_with_ikin  = 1;
end

%% Contact constraints parameters        
if       config.feet_on_ground(1) == 1 && config.feet_on_ground(2) == 1
 
config.constraintLinkNames      = {'l_sole','r_sole'}; 

elseif   config.feet_on_ground(1) == 1 && config.feet_on_ground(2) == 0
   
config.constraintLinkNames      = {'l_sole'}; 
 
elseif   config.feet_on_ground(1) == 0 && config.feet_on_ground(2) == 1
       
config.constraintLinkNames      = {'r_sole'};
end

config.numConstraints           = length(config.constraintLinkNames);

%% Initial state of the robot 
config.qjInit             = [config.torsoInit;config.leftArmInit;config.rightArmInit;config.leftLegInit;config.rightLegInit]*(pi/180);
config.dqjInit            = zeros(config.ndof,1);
config.VelBaseInit        = zeros(3,1);
config.omegaWorldBaseInit = zeros(3,1);

% Update the initial position
wbm_updateState(config.qjInit,config.dqjInit,[config.VelBaseInit;config.omegaWorldBaseInit]);

% fixing the world reference frame w.r.t. the foot on ground position
if config.feet_on_ground(1) == 1

    [config.RotBaseInit,config.PosBaseInit] = wbm_getWorldFrameFromFixedLink('l_sole',config.qjInit);
else
    [config.RotBaseInit,config.PosBaseInit] = wbm_getWorldFrameFromFixedLink('r_sole',config.qjInit);    
end

wbm_setWorldFrame(config.RotBaseInit,config.PosBaseInit,[0 0 -9.81]')

% initial base pose of the base; initial state
[~,config.BasePoseInit,~,~]         = wbm_getState();
config.chiInit                      = [config.BasePoseInit; config.qjInit; config.VelBaseInit; config.omegaWorldBaseInit; config.dqjInit];
config.InitState                    = robotState(config.chiInit,config);

%% Joint references with ikin
if config.jointRef_with_ikin == 1
    
[config.ikin,config.chiInit,config.ContFig] = initInverseKinematics(config,config.InitState);
config.InitState                            = robotState(config.chiInit,config);
end

%% Initial dynamics
config.InitDynamics              = robotDynamics(config.InitState,config);

% the centroidal momentum jacobian is reduced to eliminate the base velocity. This
% is then used to compute che approximation of the angular momentum integral
JH                               = config.InitDynamics.JH;
Jc                               = config.InitDynamics.Jc;
config.InitDynamics.JhReduced    = JH(:,7:end) -JH(:,1:6)*(eye(6)/Jc(1:6,1:6))*Jc(1:6,7:end);

%% Initial forward kinematics
config.InitForKinematics         = robotForKinematics(config.InitState,config.InitDynamics);

%% Initial gains (no gains tuning) 
gainsInit                        = gains(config,config.InitDynamics);

%% Joint space linearization around the initial joint position
if config.linearize == 1
    
config.linearization             = jointSpaceLinearization(config,gainsInit);

if config.linearize_for_gains_tuning == 1
    
[config.gains,config.visualizeTuning] = gainsTuning(config.linearization,config,gainsInit);
else
config.gains.impedances     = gainsInit.impedances; 
config.gains.dampings       = gainsInit.dampings;
config.gains.posturalCorr   = gainsInit.posturalCorr ;
config.gains.VelGainsMom    = [gainsInit.gainsDCoM zeros(3); zeros(3) gainsInit.gainsDAngMom];
config.gains.PosGainsMom    = [gainsInit.gainsPCoM zeros(3); zeros(3) gainsInit.gainsPAngMom];
end

%% FORWARD DYNAMICS INTEGRATION
config.wait      = waitbar(0,'State integration in progress...');
forwardDynFunc   = @(t,chi)forwardDynamics(t,chi,config);

if config.integrateWithFixedStep == 1
   
[t,chi]         = euleroForward(forwardDynFunc,config.chiInit,config.tEnd,config.tStart,config.sim_step);   
else    
[t,chi]         = ode15s(forwardDynFunc,config.tStart:config.sim_step:config.tEnd,config.chiInit,config.options);
end

delete(config.wait)       
 
%% Visualize the results
initVisualizer(t,chi,config);

end
