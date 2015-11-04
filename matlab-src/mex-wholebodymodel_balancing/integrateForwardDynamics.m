clear all
close all
clc

% This is the main program for integrating the forward dynamics of the
% robot in matlab

%% setup path
% Don't forget to set the path properly depending on where are the folders in
% your computer
addpath('./../whole_body_model_functions/');
addpath('./../../../build/');
addpath('./../worker_functions');
addpath('./../');

%% initialise mexWholeBodyModel
 wbm_modelInitialise('icubGazeboSim');
 
%% params for balancing controller
% the user can set this parameters depending on what he wants to do with
% the robot
 params.demo_left_and_right      =  1;                                      %either 0 or 1; only for two feet on the ground
 params.QP_solver                =  0;                                      %either 0 or 1
 
% balancing on two feet or one foot
 params.feet_on_ground           =  2;                                      %either 1 or 2

% for the visualization of torques, forces and other user-defined graphics 
 vis_graphics                    =  1;                                      %either 0 or 1
 
%% setup params
% this is assuming a 25DoF iCub
 params.ndof         = 25;

% initial conditions                  
 params.leftArmInit  = [ -19.7   29.7  0.0  44.9   0.0]';          
 params.rightArmInit = [ -19.7   29.7  0.0  44.9   0.0]';         
 
 if     params.feet_on_ground == 2
     
% initial conditions for balancing on two feet 
 params.torsoInit    = [ -10.0   0.0   0.0]'; 
 params.leftLegInit  = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';
 params.rightLegInit = [  25.5   0.1   0.0  -18.5  -5.5  -0.1]';

 elseif params.feet_on_ground == 1
     
% initial conditions for the robot standing on one foot
 params.torsoInit    = [ -10.0   0.0    0.0]';
 params.leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
 params.rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1]'; 

 end
 
 params.qjInit  = [params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit] * (pi/180);
 params.dqjInit = zeros(params.ndof,1);
   
 params.dx_bInit    = zeros(3,1);
 params.omega_bInit = zeros(3,1);
  
%% fixing the world reference frame to the ground, (first obtaining from left foot )
 
[rot,pos] = wbm_getWorldFrameFromFixedLink('l_sole',params.qjInit);
wbm_setWorldFrame(rot,pos,[ 0,0,-9.81]');

 %wbm_updateState(params.qjInit,zeros(params.ndof,1),zeros(6,1));
 
 [qj,T_b,dqj,vb] = wbm_getState();
 %[pos,rot]       = frame2posrot(T_b);
 %wbm_setWorldFrame(rot,pos,[0,0,-9.81]')
 
% the vector of variables is redefined to integrate also the position of the feet to
% correct numerical errors
%  params.chiInit = [T_b;params.qjInit;...
%                    params.dx_bInit;params.omega_bInit;params.dqjInit];

if     params.feet_on_ground == 2
    
   params.chiInit = [T_b;params.qjInit;...
                    params.dx_bInit;params.omega_bInit;params.dqjInit; zeros(12,1)];   
    
elseif params.feet_on_ground == 1
  
   params.chiInit = [T_b;params.qjInit;...
                    params.dx_bInit;params.omega_bInit;params.dqjInit; zeros(6,1)];
    
end

%% contact constraints         
 if     params.feet_on_ground == 2
     
 params.constraintLinkNames      = {'l_sole','r_sole'}; 

 elseif params.feet_on_ground == 1
     
% this is for only for the left foot on the ground
 params.constraintLinkNames      = {'l_sole'}; 

 end

 params.numConstraints           = length(params.constraintLinkNames);
   
%% others parameters for balancing controller
 load('jointLimits.mat')
 
 limits           = [jl1 jl2];
 params.limits    = limits;
 params.com_ini   = wbm_forwardKinematics('com');
 
 params.lfoot_ini = wbm_forwardKinematics('l_sole');
 params.rfoot_ini = wbm_forwardKinematics('r_sole');
 
%% setup integration
 forwardDynFunc  = @(t,chi)forwardDynamics(t,chi,params);
    
 params.tStart   = 0;
 params.tEnd     = 0.05;   
 params.sim_step = 0.01;

%% integrate forward dynamics
 disp('starting numerical integration');
 
 options = odeset('RelTol',1e-4,'AbsTol',1e-4);
   
 [t,chi] = ode15s(forwardDynFunc,params.tStart:params.sim_step:params.tEnd,params.chiInit,options);

 save('storedTestTrajectory.mat','t','chi','params');
 disp('numerical integration complete..rendering.');

%% visualize forward dynamics
% demo of the dynamics
 visualizer_demo(t,chi,params);

% desired graphics 
if vis_graphics == 1
 
 disp('preparing the graphics...')
 visualizer_graphics(t,chi,params);

end



    