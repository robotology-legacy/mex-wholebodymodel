%% integrateForwardDynamics_js
%  This is the main program for integrating the forward dynamics of the robot iCub in matlab.
%  It integrates the robot state defined in forwardDynamics_js.m and the user can set 
%  how many feet are on the ground, decide if activate the robot's movements, 
%  plot forces, torques and joints variables, and activate a demo of the
%  robot's movements.
clear all
close all
clc

%% Setup path
addpath('./../../../mex-wholebodymodel/matlab/utilities');
addpath('./../../../mex-wholebodymodel/matlab/wrappers');
addpath('./../../../../../build/');
addpath('./ikin_functions');
addpath('./centroidalTransf_functions');

%% Initialise the mexWholeBodyModel
wbm_modelInitialise('icubGazeboSim');

%% Setup robot parameters
% CoM follows a desired trajectory
 params.demo_movements           =  1;                                     %either 0 or 1
 
% balancing on two feet or one foot
 params.feet_on_ground           = [1 1];                                  %either 1 or 0

% visualize inverse kinematics graphics
 params.visualizerIkin           =  1;                                      %either 0 or 1
 
% visualize state integration graphics
 params.visualizerGraphics       =  1;                                      %either 0 or 1
 params.visualizerJoints         =  1;                                      %either 0 or 1

% visualize demo of robot's movements
 params.visualizerDemo           =  1;                                      %either 0 or 1

%% Com trajectory reference
% if demo_movements = 1, these parameters define the desired CoM trajectory
% reference = [amplitude of oscillation (meters); frequency (Hz)]
if     sum(params.feet_on_ground) == 2

params.noOscillationTime   =  0;    
params.direction           = [0;1;0];
params.reference           = [0.035 0.35]; 

elseif  sum(params.feet_on_ground) == 1
    
params.noOscillationTime   =  0;
params.direction           = [0;1;0];
params.reference           = [0.015 0.15];  

end

%% Setup integration initial conditions
% this is assuming a 25DoF iCub
 params.ndof         = 25;

% initial joints position (deg)                 
 params.leftArmInit  = [ -20   30  0.0  45   0.0]';          
 params.rightArmInit = [ -20   30  0.0  45   0.0]';
 params.torsoInit    = [ -10.0   0.0    0.0]';
 
 if     sum(params.feet_on_ground) == 2
     
% initial conditions for balancing on two feet  
 params.leftLegInit  = [  25.5   0.0   0.0  -18.5  -5.5  -0.0]';
 params.rightLegInit = [  25.5   0.0   0.0  -18.5  -5.5  -0.0]';

 elseif params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 0
      
% initial conditions for the robot standing on the left foot
 params.leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
 params.rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1]'; 
 
 elseif params.feet_on_ground(1) == 0 && params.feet_on_ground(2) == 1
      
% initial conditions for the robot standing on the left foot
 params.rightLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
 params.leftLegInit   = [  25.5   5.0    0.0  -40    -5.5  -0.1]'; 

 end
 
 params.qjInit  = [params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit]*(pi/180);
 params.dqjInit = zeros(params.ndof,1);
   
% initial floating base velocity
 params.dx_bInit    = zeros(3,1);
 params.omega_bInit = zeros(3,1);
  
%% Update the robot state with the initial conditions
wbm_updateState(params.qjInit,params.dqjInit,[params.dx_bInit;params.omega_bInit]);

% fixing the world reference frame w.r.t. the foot on ground
if params.feet_on_ground(2) == 0
    
[rot,pos]   = wbm_getWorldFrameFromFixedLink('r_sole',params.qjInit);

else

[rot,pos]   = wbm_getWorldFrameFromFixedLink('l_sole',params.qjInit);

end

wbm_setWorldFrame(rot,pos,[0 0 -9.81]')
[~,T_b,~,~] = wbm_getState();
 
%% Contact constraints at feet        
if     sum(params.feet_on_ground) == 2
     
 params.constraintLinkNames      = {'l_sole','r_sole'}; 

 elseif params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 0
     
 params.constraintLinkNames      = {'l_sole'}; 
 
  elseif params.feet_on_ground(1) == 0 && params.feet_on_ground(2) == 1

 params.constraintLinkNames      = {'l_sole'}; 
       
end

 params.numConstraints           = length(params.constraintLinkNames);
   
%% Feet initial position and orientation and CoM initial position                               
 params.lfoot_ini = wbm_forwardKinematics(rot, pos, params.qjInit,'l_sole');
 params.rfoot_ini = wbm_forwardKinematics(rot, pos, params.qjInit,'r_sole');
 
 params.CoM_ini   = wbm_forwardKinematics(rot, pos, params.qjInit,'com');
 
%% Joints limits
 jointLimits
 limits           = [jl1 jl2];
 params.limits    = limits;

%% Desired CoM trajectory at time t = 0 
directionOfOscillation            = [0; 0; 0];
referenceParams                   = [0  0]; 
 
if  params.demo_movements == 1
        
        directionOfOscillation = params.direction;
        referenceParams        = params.reference;   
    
end
 
desired_x_dx_ddx_CoM   = generTraj_js (params.CoM_ini(1:3), 0, referenceParams,...
                                       directionOfOscillation, params.noOscillationTime); 

%% Velocity initial condition for inverse kinematics integration
% setup parameters
PINV_TOL    = 1e-8;

% Feet and CoM jacobians 
for ii=1:params.numConstraints
    
    Jc_ikin(6*(ii-1)+1:6*ii,:) = wbm_jacobian(rot, pos, params.qjInit, params.constraintLinkNames{ii});

end

JCoM_ikin_tot  =  wbm_jacobian(rot, pos, params.qjInit,'com');
JCoM_ikin      =  JCoM_ikin_tot(1:3,:);

%% Desired state velocity
% higher priority: feet position and orientation
pinvJc_ikin        = pinv(Jc_ikin,PINV_TOL);
feetPosAndOrient   = zeros(6*params.numConstraints,1); 
Nullfeet           = eye(params.ndof+6) - pinvJc_ikin*Jc_ikin;

% lower priority: CoM position
JCoM_low           = JCoM_ikin*Nullfeet;
CoMPos             = desired_x_dx_ddx_CoM(:,2) - JCoM_ikin*pinvJc_ikin*feetPosAndOrient;

% desired state velocity
Nu_ikin0           = pinv(JCoM_low, PINV_TOL)*CoMPos;    
Nu_ikin            = pinvJc_ikin*feetPosAndOrient + Nullfeet*Nu_ikin0;

% initial condition for inverse kinematics integrator
params.ikin_init   = [T_b; params.qjInit; Nu_ikin];

% initial condition for state integration 
params.chiInit     = [T_b; params.qjInit; Nu_ikin]; 

%% Final and initial integration time
params.tStart        = 0;
params.tEnd          = 10;
 
%% Inverse kinematics integrator
% integration step for inverse kinematics integrator
params.euler_step    = 0.01; 

% waitbar 
params.wait          = waitbar(0,'Inverse kinematics integration in process...');
 
[params.t_kin, params.joints_traj] = ikin_integrator(params);

delete(params.wait)       
 
% inverse kinematics visualization
plot_set
ikin_graphics(params);

%% Setup state integration
% ode tolerances
 params.sim_step = 0.01;
 options         = odeset('RelTol',1e-5,'AbsTol',1e-5);
 params.wait     = waitbar(0,'State integration in process...');

% function that will be integrated
 forwardDynFunc  = @(t,chi)forwardDynamics_js(t,chi,params);
 [t,chi]         = ode15s(forwardDynFunc,params.tStart:params.sim_step:params.tEnd,params.chiInit,options);

 delete(params.wait) 

%% Visualizer
params.wait     = waitbar(0,'Preparing the graphics...');   
 
visualizer_js(t,chi,params);
 
delete(params.wait)
 
   