function [] = initForwardDynamics(CONFIG)
%INITFORWARDDYNAMICS setup the forward dynamics integration of robot iCub 
%                    in MATLAB.
%
%             [] = INITFORWARDDYNAMICS(config) takes as input the structure
%             CONFIG containing all the configuration parameters. It has no 
%             output. The forward dynamics integration will be performed 
%             following the options the user specifies in the initialization 
%             file.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
%% PARAMETERS SETUP
plot_set
feet_on_ground            = CONFIG.feet_on_ground;
ndof                      = CONFIG.ndof;
CONFIG.figureCont         = 1;
CONFIG.linearizeJointSp   = 0;

% Paths correction: linearization
if CONFIG.linearize_for_gains_tuning == 1 || CONFIG.linearize_for_stability_analysis == 1
    
CONFIG.linearizeJointSp   = 1;    
end

% Joint space controller
if strcmp(CONFIG.controller,'JointSpace') == 1
    
CONFIG.linearizeJointSp                     = 0;                          
CONFIG.use_QPsolver                         = 0;
CONFIG.visualize_stability_analysis_results = 0;
CONFIG.jointRef_with_ikin                   = 1;
end

% Stability analysis visualization
if  CONFIG.linearizeJointSp == 1 && CONFIG.visualize_stability_analysis_results  == 1
    
CONFIG.jointRef_with_ikin  = 1;
end

%% Contact constraints definition       
if       feet_on_ground(1) == 1 && feet_on_ground(2) == 1
 
CONFIG.constraintLinkNames      = {'l_sole','r_sole'}; 

elseif   feet_on_ground(1) == 1 && feet_on_ground(2) == 0
   
CONFIG.constraintLinkNames      = {'l_sole'}; 
 
elseif   feet_on_ground(1) == 0 && feet_on_ground(2) == 1
       
CONFIG.constraintLinkNames      = {'r_sole'};
end

CONFIG.numConstraints           = length(CONFIG.constraintLinkNames);

%% Initial state of the robot 
qjInit             = [CONFIG.torsoInit;CONFIG.leftArmInit;CONFIG.rightArmInit;CONFIG.leftLegInit;CONFIG.rightLegInit]*(pi/180);
dqjInit            = zeros(ndof,1);
VelBaseInit        = zeros(3,1);
omegaBaseWorldInit = zeros(3,1);

% Update the initial position
wbm_updateState(qjInit,dqjInit,[VelBaseInit;omegaBaseWorldInit]);

% Fixing the world reference frame w.r.t. the foot on ground position
if  feet_on_ground(1) == 1

    [RotBaseInit,PosBaseInit] = wbm_getWorldFrameFromFixedLink('l_sole',qjInit);
else
    [RotBaseInit,PosBaseInit] = wbm_getWorldFrameFromFixedLink('r_sole',qjInit);    
end

wbm_setWorldFrame(RotBaseInit,PosBaseInit,[0 0 -9.81]')

% Initial base pose; initial state
[~,BasePoseInit,~,~]         = wbm_getState();
chiInit                      = [BasePoseInit; qjInit; VelBaseInit; omegaBaseWorldInit; dqjInit];
CONFIG.initState             = robotState(chiInit,CONFIG);

%% INVERSE KINEMATICS
if CONFIG.jointRef_with_ikin == 1
    
[CONFIG.ikin,chiInit,CONFIG.figureCont]  = initInverseKinematics(CONFIG);
CONFIG.initState                         = robotState(chiInit,CONFIG);
end

%% INITIAL CONDITIONS
% Initial dynamics
CONFIG.initDynamics              = robotDynamics(CONFIG.initState,CONFIG);

% The centroidal momentum jacobian is reduced to eliminate the base velocity. This
% is then used to compute the approximation of the angular momentum integral
JH                               = CONFIG.initDynamics.JH;
Jc                               = CONFIG.initDynamics.Jc;
% CONFIG.initDynamics.JhReduced    = JH(:,7:end) -JH(:,1:6)*(eye(6)/Jc(1:6,1:6))*Jc(1:6,7:end);
CONFIG.JHRed   = JH(:,7:end) -JH(:,1:6)*(eye(6)/Jc(1:6,1:6))*Jc(1:6,7:end);
CONFIG.initForKinematics         = robotForKinematics(CONFIG.initState,CONFIG.initDynamics);
CONFIG.xCoMRef = CONFIG.initForKinematics.xCoM;

% PoseLFootQuat                    =  CONFIG.initForKinematics.LFootPoseQuat;
% PoseRFootQuat                    =  CONFIG.initForKinematics.RFootPoseQuat;
% PoseLFootEul                     = CONFIG.initForKinematics.LFootPoseEul;
% PoseRFootEul                     = CONFIG.initForKinematics.RFootPoseEul

%%%%%%%%%%%%%%%%%%%%% DELTA POSITION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
qjInit(1:13)             = qjInit(1:13)+5*pi/180;
dqjInit            = zeros(ndof,1);
VelBaseInit        = zeros(3,1);
omegaBaseWorldInit = zeros(3,1);

% Update the initial position
wbm_updateState(qjInit,dqjInit,[VelBaseInit;omegaBaseWorldInit]);

% Fixing the world reference frame w.r.t. the foot on ground position
if  feet_on_ground(1) == 1

    [RotBaseInit,PosBaseInit] = wbm_getWorldFrameFromFixedLink('l_sole',qjInit);
else
    [RotBaseInit,PosBaseInit] = wbm_getWorldFrameFromFixedLink('r_sole',qjInit);    
end

wbm_setWorldFrame(RotBaseInit,PosBaseInit,[0 0 -9.81]')

% Initial base pose; initial state
[~,BasePoseInit,~,~]         = wbm_getState();
chiInit                      = [BasePoseInit; qjInit; VelBaseInit; omegaBaseWorldInit; dqjInit];
CONFIG.initState             = robotState(chiInit,CONFIG);
CONFIG.initDynamics              = robotDynamics(CONFIG.initState,CONFIG);
CONFIG.initDynamics.JhReduced    = CONFIG.JHRed ;
CONFIG.initForKinematics         = robotForKinematics(CONFIG.initState,CONFIG.initDynamics);

% CONFIG.initForKinematics.RFootPoseEul 
% CONFIG.initForKinematics.LFootPoseEul = PoseLFootEul;
% CONFIG.initForKinematics.RFootPoseEul = PoseRFootEul;
% CONFIG.initForKinematics.LFootPoseQuat = PoseLFootQuat;
% CONFIG.initForKinematics.RFootPoseQuat = PoseRFootQuat;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIAL GAINS
CONFIG.gains                 = gains(CONFIG);

%% FORWARD DYNAMICS INTEGRATION
CONFIG.wait     = waitbar(0,'State integration in progress...');
forwardDynFunc  = @(t,chi)forwardDynamics(t,chi,CONFIG);

% Either fixed step integrator or ODE15s
if CONFIG.integrateWithFixedStep == 1
  
[t,chi]         = euleroForward(forwardDynFunc,chiInit,CONFIG.tEnd,CONFIG.tStart,CONFIG.sim_step);   
else    
[t,chi]         = ode15s(forwardDynFunc,CONFIG.tStart:CONFIG.sim_step:CONFIG.tEnd,chiInit,CONFIG.options);
end

delete(CONFIG.wait)       
 
%% VISUALIZER
CONFIG.figureCont = initVisualizer(t,chi,CONFIG);

end
