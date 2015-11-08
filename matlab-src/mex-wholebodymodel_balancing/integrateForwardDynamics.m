clear all
close all
clc

% This is the main program for integrating the forward dynamics of the
% robot in matlab

%% setup path
% Don't forget to set the path properly depending on where are the folders in
% your computer
addpath('./../../mex-wholebodymodel/matlab/utilities');
addpath('./../../mex-wholebodymodel/matlab/wrappers');
addpath('./../../../../build/');
addpath('./../');

%% initialise mexWholeBodyModel
 wbm_modelInitialise('icubGazeboSim');
 
%% user defined parameters 
% the user can set this parameters depending on what he wants to do with
% the robot
 params.demo_left_and_right      =  1;                                      %either 0 or 1; only for two feet on the ground
 params.QP_solver                =  0;                                      %either 0 or 1
 
% balancing on two feet or one foot
 params.feet_on_ground           =  2;                                      %either 1 or 2

% for the visualization of user-defined graphics 
 vis_graphics                    =  1;                                      %either 0 or 1
 
%% setup params
% this is assuming a 25DoF iCub
 params.ndof         = 25;

% initial conditions        
 params.torsoInit    = [ -10.0   0.0   0.0]';
 params.leftArmInit  = [ -19.7   29.7  0.0  44.9   0.0]';          
 params.rightArmInit = [ -19.7   29.7  0.0  44.9   0.0]';         
 
 if     params.feet_on_ground == 2
     
% initial conditions for balancing on two feet 
 params.leftLegInit  = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';
 params.rightLegInit = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';

 elseif params.feet_on_ground == 1
     
% initial conditions for the robot standing on one foot
 params.leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
 params.rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1]'; 

 end
 
 params.qjInit  = [params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit] * (pi/180);
 params.dqjInit = zeros(params.ndof,1);
   
 params.dx_bInit    = zeros(3,1);
 params.omega_bInit = zeros(3,1);
  
%% obtaining the world reference frame from left foot position and fixing it on the ground
% update the state with the initial conditions
wbm_updateState(params.qjInit,params.dqjInit,[params.dx_bInit;params.omega_bInit]);

% fixing the world reference frame w.r.t. the left foot position
[rot,pos]   = wbm_getWorldFrameFromFixedLink('l_sole',params.qjInit);

wbm_setWorldFrame(rot,pos,[0 0 -9.81]')

[~,T_b,~,~] = wbm_getState();

%% contact constraints         
 if     params.feet_on_ground == 2
     
 params.constraintLinkNames      = {'l_sole','r_sole'}; 

 elseif params.feet_on_ground == 1
     
% only left foot on the ground available for now
 params.constraintLinkNames      = {'l_sole'}; 

 end

 params.numConstraints           = length(params.constraintLinkNames); 
 
%% initial conditions for state integration
 params.chiInit = [T_b;params.qjInit;...
                   params.dx_bInit;params.omega_bInit;params.dqjInit];

%% others parameters for balancing controller
 load('jointLimits.mat')
 
 limits           = [jl1 jl2];
 params.limits    = limits;
 params.com_ini   = wbm_forwardKinematics(rot',pos,params.qjInit,'com');
 
 params.lfoot_ini = wbm_forwardKinematics(rot',pos,params.qjInit,'l_sole');
 params.rfoot_ini = wbm_forwardKinematics(rot',pos,params.qjInit,'r_sole');
 
%% setup integration
 params.tStart   = 0;
 params.tEnd     = 10;   
 params.sim_step = 0.01;
 
 % waitbar 
 params.wait            = waitbar(0,'State integration in process...');
 
 forwardDynFunc  = @(t,chi)forwardDynamics(t,chi,params);
    
%% integrate forward dynamics
 options = odeset('RelTol',1e-4,'AbsTol',1e-4);
   
 [t,chi] = ode15s(forwardDynFunc,params.tStart:params.sim_step:params.tEnd,params.chiInit,options);

 delete(params.wait) 
 
% uncomment the following line if you want to save the integration results
%save('storedTestTrajectory.mat','t','chi','params');

%% visualize forward dynamics integration results
% demo of the robot dynamics
 visualizer_demo(t,chi,params);

% desired graphics 
if vis_graphics == 1
 
 params.wait            = waitbar(0,'Preparing the graphics...');
 visualizer_graphics(t,chi,params);
 delete(params.wait) 

end

