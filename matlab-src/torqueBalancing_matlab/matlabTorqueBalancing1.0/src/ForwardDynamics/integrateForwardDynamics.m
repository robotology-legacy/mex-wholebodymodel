function [] = integrateForwardDynamics(params)
%INTEGRATEFORWARDDYNAMICS setup the integration of the forward dynamics of
%                         the robot iCub in MATLAB.
%   [] = INTEGRATEFORWARDDYNAMICS(params) takes as input the structure
%   params containing all the utility parameres. It has no output. The 
%   forward dynamics integration will be performed following the options the 
%   user specified in the initialization file.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% initial parameters setup
% call the script which contains all the options for the visualization
plot_set
params.ContFig            = 1;
% tolerances for pseudoinverse and Qp solver
params.pinv_tol           = 1e-8;
params.pinv_damp          = 5e-6;
params.reg_HessianQP      = 1e-3;
% options correction
params.SoTController      = strcmp(params.BalancingController,'StackOfTask');
params.JSController       = strcmp(params.BalancingController,'JointSpace');

if params.JSController == 1
    
params.linearize_for_stability_analysis     = 0;                       
params.linearize_for_gains_tuning           = 0;                           
params.use_QPsolver                         = 0;                           
params.jointRef_with_ikin                   = 1;
end

params.linearization = 0;

if params.linearize_for_gains_tuning == 1 || params.linearize_for_stability_analysis == 1
    
    params.linearization = 1;    
end

if  params.linearization == 1 && params.visualize_stability_analysis_results  == 1
    
params.jointRef_with_ikin                   = 1;
end

%% Initial joints position and velocities; initial base velocity
params.qjInit             = [params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit]*(pi/180);
params.dqjInit            = zeros(params.ndof,1);
params.VelBaseInit        = zeros(3,1);
params.omegaWorldBaseInit = zeros(3,1);

%% Update the initial position
wbm_updateState(params.qjInit,params.dqjInit,[params.VelBaseInit;params.omegaWorldBaseInit]);

% fixing the world reference frame w.r.t. the foot on ground position
if params.feet_on_ground(1) == 1

    [params.RotBaseInit,params.PosBaseInit] = wbm_getWorldFrameFromFixedLink('l_sole',params.qjInit);
else
    [params.RotBaseInit,params.PosBaseInit] = wbm_getWorldFrameFromFixedLink('r_sole',params.qjInit);    
end

wbm_setWorldFrame(params.RotBaseInit,params.PosBaseInit,[0 0 -9.81]')

% initial base pose of the robot
[~,params.BasePoseInit,~,~]  =  wbm_getState();
 
%% Contact constraints         
if       params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1
 
% contact constraints for two feet on ground
 params.constraintLinkNames      = {'l_sole','r_sole'}; 

elseif   params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 0

% contact constraints for the left foot on ground     
 params.constraintLinkNames      = {'l_sole'}; 
 
elseif   params.feet_on_ground(1) == 0 && params.feet_on_ground(2) == 1
    
% contact constraints for the right foot on ground    
 params.constraintLinkNames      = {'r_sole'};
end

params.numConstraints            = length(params.constraintLinkNames);
   
%% Initial parameters definition
% mass matrix, constraints Jacobian and CoM Jacobian
params.MInit         = wbm_massMatrix(params.RotBaseInit,params.PosBaseInit,params.qjInit);
params.JCoMInit      = wbm_jacobian(params.RotBaseInit,params.PosBaseInit,params.qjInit,'com');
params.JcInit        = zeros(6*params.numConstraints,params.ndof+6);

for i=1:params.numConstraints
    
params.JcInit(6*(i-1)+1:6*i,:) = wbm_jacobian(params.RotBaseInit,params.PosBaseInit,params.qjInit,params.constraintLinkNames{i});   
end

% centroidal momentum jacobian
JhBaseInit            = zeros(6,6);
JhJointInit           = zeros(6,params.ndof);

for ii = 1:6
    
NuBaseJh              = zeros(6,1);
NuBaseJh(ii)          = 1;
JhBaseInit(:,ii)      = wbm_centroidalMomentum(params.RotBaseInit, params.PosBaseInit, params.qjInit, zeros(params.ndof,1), NuBaseJh);
end

for ii = 1:params.ndof

dqjJh                = zeros(params.ndof,1);
dqjJh(ii)            = 1;
JhJointInit(:,ii)    = wbm_centroidalMomentum(params.RotBaseInit, params.PosBaseInit, params.qjInit, dqjJh, zeros(6,1));
end

params.JhInit        =  [JhBaseInit JhJointInit];     

% the centroidal momentum jacobian is reduced to eliminate the base velocity. This
% is then used to compute che approximation of the angular momentum integral
params.JhReduced     =  JhJointInit -JhBaseInit*(eye(6)/params.JcInit(1:6,1:6))*params.JcInit(1:6,7:end);

% Forward kinematics and joints limits
params.CoMInit                = wbm_forwardKinematics('com');
params.PoseLFootQuatInit      = wbm_forwardKinematics('l_sole');
params.PoseRFootQuatInit      = wbm_forwardKinematics('r_sole');
[jl1,jl2]                     = wbm_jointLimits();
params.limits                 = [jl1 jl2];

%% Initial gains (no tuning), constraints for QP and CoM trajectory definition
[gainsInit, params.constraints, params.trajectory] = gainsAndConstraints(params);

%% Joints references definition using inverse kinematics
if params.jointRef_with_ikin == 1
    
[params.ikin,params.chiInit,params.ContFig] = initInverseKin(params);
else
params.chiInit  =  [params.BasePoseInit; params.qjInit; params.VelBaseInit; params.omegaWorldBaseInit; params.dqjInit];    
end

%% State linearization around the initial joint position
if params.linearize_for_gains_tuning == 1 || params.linearize_for_stability_analysis == 1
    
    if  sum(params.feet_on_ground) == 1
        
        [params.linearization, params.gains] = OneFootLinearization(params,gainsInit);
    else
        [params.linearization, params.gains] = TwoFeetLinearization(params,gainsInit);
    end   
else
    
 params.gains.impedances    = gainsInit.impedances; 
 params.gains.dampings      = gainsInit.dampings;
 params.gains.posturalCorr  = gainsInit.posturalCorr ;
 params.gains.VelGainsMom   = [gainsInit.gainsDCoM zeros(3); zeros(3) gainsInit.gainsDAngMom];
 params.gains.PosGainsMom   = [gainsInit.gainsPCoM zeros(3); zeros(3) gainsInit.gainsPAngMom];
end

%% Integrate the forward dynamics of the robot
params.wait      = waitbar(0,'State integration in progress...');
forwardDynFunc   = @(t,chi)forwardDynamics(t,chi,params);

if params.integrateWithFixedStep == 1
   
[t,chi]         = integrateForwardDynEulero(forwardDynFunc,params.chiInit,params.tEnd,params.tStart,params.sim_step);   
else    
[t,chi]         = ode15s(forwardDynFunc,params.tStart:params.sim_step:params.tEnd,params.chiInit,params.options);
end

delete(params.wait)       
 
%% Visualize forward dynamics integration results

if params.visualize_robot_simulator == 1
    
params.ContFig = visDemo(t,chi,params);
end

if params.visualize_integration_results == 1  || params.visualize_joints_dynamics == 1 &&  params.linearization == 1 ||...
   params.visualize_gains_tuning_results == 1 || params.visualize_stability_analysis_results == 1

params.wait = waitbar(0,'Generating the plots...');
visMain(t,chi,params);
delete(params.wait)
end

end
