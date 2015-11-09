% integrateForwardDynamics sets the parameters used to integrate the
% forward dynamics of the robot iCub in matlab
clear all
close all
clc

%% Setup path
% set the path properly depending on where are the
% required folders in your computer
addpath('./../../../mex-wholebodymodel/matlab/utilities');
addpath('./../../../mex-wholebodymodel/matlab/wrappers');
addpath('./../../../../../build/');
addpath('./../../');

%% Initialise the mexWholeBodyModel
wbm_modelInitialise('icubGazeboSim');

%% Setup balancing controller and visualizer
% the user can set this parameters depending on what he wants to do with
% the robot
 params.demo_left_and_right      =  1;                                      %either 0 or 1
 
% balancing on two feet or one foot
 params.feet_on_ground           =  2;                                      %either 1 or 2

% visualize inverse kinematic graphics
 vis_ikin_graphics               =  1;                                      %either 0 or 1
 
% visualize state integration graphics
 vis_graphics                    =  1;                                      %either 0 or 1
 params.vis_joints_variables     =  1;                                      %either 0 or 1

% visualize demo of robot's movements and add a pause between each
% integration step
 vis_demo                        =  1;                                      %either 0 or 1
 use_pause                       =  0;                                      %either 0 or 1
 
%% Com trajectory reference
% if demo_left_and_right = 1, these parameters define the desired CoM trajectory
% reference = amplitude of oscillation (meters); frequency (Hz)
if     params.feet_on_ground == 2

params.direction  = [0;1;0];
params.reference  = [0.035 0.35]; 

elseif  params.feet_on_ground == 1
    
params.direction  = [0;1;0];
params.reference  = [0.015 0.15];  

end

%% Setup initial conditions
% this is assuming a 25DoF iCub
 params.ndof         = 25;

% initial joints position (deg)                 
 params.leftArmInit  = [ -19.7   29.7  0.0  44.9   0.0]';          
 params.rightArmInit = [ -19.7   29.7  0.0  44.9   0.0]';
 params.torsoInit    = [ -10.0   0.0    0.0]';
 
 if     params.feet_on_ground == 2
     
% initial conditions for balancing on two feet  
 params.leftLegInit  = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';
 params.rightLegInit = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';

 elseif params.feet_on_ground == 1
     
% initial conditions for the robot standing on one foot
 params.leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
 params.rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1]'; 

 end
 
 params.qjInit  = [params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit]*(pi/180);
 params.dqjInit = zeros(params.ndof,1);
   
% initial floating base velocity
 params.dx_bInit    = zeros(3,1);
 params.omega_bInit = zeros(3,1);
  
%% Fixing the world reference frame to the ground, not to the left foot 
wbm_updateState(params.qjInit,params.dqjInit,[params.dx_bInit;params.omega_bInit]);

% fixing the world reference frame w.r.t. the left foot position
[rot,pos]   = wbm_getWorldFrameFromFixedLink('l_sole',params.qjInit);

wbm_setWorldFrame(rot,pos,[0 0 -9.81]')

[~,T_b,~,~] = wbm_getState();

% initial condition for state integration 
 params.chiInit    = [T_b; params.qjInit; params.dx_bInit; params.omega_bInit; params.dqjInit];   

% initial condition for inverse kinematics integration
 params.ikin_init  = [T_b; params.qjInit];

%% Contact constraints         
if     params.feet_on_ground == 2
     
 params.constraintLinkNames      = {'l_sole','r_sole'}; 

elseif params.feet_on_ground == 1
     
% the one foot balancing is setted with left foot on the ground by default
 params.constraintLinkNames      = {'l_sole'}; 

end

 params.numConstraints           = length(params.constraintLinkNames);
   
%% Others parameters for balancing controller
 load('jointLimits.mat')
 
 limits           = [jl1 jl2];
 params.limits    = limits;
 params.com_ini   = wbm_forwardKinematics(rot.', pos, params.qjInit,'com');
 
 params.lfoot_ini = wbm_forwardKinematics(rot.', pos, params.qjInit,'l_sole');
 params.rfoot_ini = wbm_forwardKinematics(rot.', pos, params.qjInit,'r_sole');
 
%% Final and initial integration time
 params.tStart          = 0;
 params.tEnd            = 10;
 
%% Inverse kinematics integrator
% step for inverse kinematics integrator
 params.euler_step      = 0.01; 

% waitbar 
 params.wait            = waitbar(0,'Inverse kinematics integration in process...');
 
 [params.t_kin, params.joints_traj] = ikin_integrator(params);

 delete(params.wait)       
 
 % for visualization of inverse kinematics results
 if  vis_ikin_graphics == 1
 ikin_graphics(params);
 end
 
 if  use_pause == 1
 pause
 end
 
%% Setup state integration
% ode tolerances
 params.sim_step = 0.01;
 
 options         = odeset('RelTol',1e-5,'AbsTol',1e-5);
 params.wait     = waitbar(0,'State integration in process...');

% function that will be integrated
 forwardDynFunc  = @(t,chi)forwardDynamics(t,chi,params);
 [t,chi]         = ode15s(forwardDynFunc,params.tStart:params.sim_step:params.tEnd,params.chiInit,options);
%[t,chi]         = euler_f(forwardDynFunc,params.chiInit,params.tEnd,params.tStart,params.sim_step);

 delete(params.wait) 

% uncomment the following line if you want to save the state integration results 
%save('storedTestTrajectory.mat','t','chi','params');

%% Visualizer
% demo of the robot dynamics
if vis_demo   == 1
 visualizer_demo(t,chi,params);
end

if  use_pause == 1
 pause
end
 
% desired graphics
if vis_graphics == 1
 
 params.wait     = waitbar(0,'Preparing the graphics...');   
 visualizer_graphics(t,chi,params);
 delete(params.wait)
 
end

    