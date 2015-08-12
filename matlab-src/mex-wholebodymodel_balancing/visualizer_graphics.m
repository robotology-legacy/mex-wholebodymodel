% clear all
% close all
% clc
% 
% % This is the main program for integrating the forward dynamics of the
% % robot in matlab
% 
% %% setup path
% addpath('./../whole_body_model_functions/');
% addpath('./../../../../build/');
% addpath('./../worker_functions');
% addpath('./../');
% 
% %% initialise mexWholeBodyModel
%  wbm_modelInitialise('icubGazeboSim');
%  wbm_setWorldLink('l_sole',eye(3),[0 0 0]',[0,0,-9.81]');
% 
% %% params for controller
% % the user can set this parameters depending on what he wants to do with
% % the robot
%  params.demo_left_and_right      =  1;                                      %either 0 or 1; only for two feet on the ground
%  params.QP_solver                =  0;                                      %either 0 or 1
% % this is  for the new angular momentum controller
%  params.HDes_solver              =  1;                                      %either 0 or 1 
% 
% %% setup params
% % this is assuming a 25DoF iCub
%  params.ndof         = 25;
% 
% % initial conditions
%  params.torsoInit    = [-10.0  0.0   0.0]';                   
%  params.leftArmInit  = [ -19.7  29.7  0.0  44.9 0.0]';          
%  params.rightArmInit = [ -19.7  29.7  0.0  44.9 0.0]';         
%  params.leftLegInit  = [ 25.5   0.1  0.0 -18.5 -5.5 -0.1]';     
%  params.rightLegInit = [ 25.5   0.1  0.0 -18.5 -5.5 -0.1]';
% 
% % right leg initial conditions for the robot standing on one foot
% %params.rightLegInit = [ 25.5   0.1  0.0 -70 -5.5 -0.1]'; 
% 
%  params.qjInit  = [params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit] * (pi/180);
%  params.dqjInit = zeros(params.ndof,1);
%    
%  params.dx_bInit    = zeros(3,1);
%  params.omega_bInit = zeros(3,1);
%   
% % fixing the world reference frame to the ground, not to the left foot 
%  wbm_updateState(params.qjInit,zeros(params.ndof,1),zeros(6,1));
%  [qj,T_b,dqj,vb] = wbm_getState();
%  [pos,rot] = frame2posrot(T_b);
%  wbm_setWorldFrame(rot,pos,[0,0,-9.81]')
% 
% % the vector of variables is redefined to add the position of the feet to
% % correct numerical errors
% %params.chiInit = [T_bInit_mod;params.qjInit;...
% %                  params.dx_bInit;params.omega_bInit;params.dqjInit];
% 
%  params.chiInit = [T_b;params.qjInit;...
%                    params.dx_bInit;params.omega_bInit;params.dqjInit; zeros(12,1)];
% 
% %% contact constraints                
%  params.constraintLinkNames      = {'l_sole','r_sole'}; 
%  
% % constraint for the robot on one foot
% %params.constraintLinkNames      = {'l_sole'};   
% 
%  params.numConstraints           = length(params.constraintLinkNames);
%    
% %% others parameters for controller
%  load('jointLimits.mat')
%  limits        = [jl1 jl2];
%  params.limits = limits;
%  params.com    = wbm_forwardKinematics('com');
%  params.qjDes  = params.qjInit;
 
 %% this is a program for visualizing old results saved in the "stored test trajectory" file.
load('storedTestTrajectory.mat')

%% plot other quantities    
fc   = [];
f0   = [];
tau  = [];
ecom = [];
pos  = [];

for tt=1:length(t)
    
[a,c]=forwardDynamics(t(tt),chi(tt,:).',params);

pos_t  =c.pos_feet;
fc_t   =c.fc;
f0_t   =c.f0;
tau_t  =c.tau;
ecom_t =c.error_com;

norm_tau(tt) = norm(tau_t);

pos  = [pos pos_t];
ecom = [ecom ecom_t];
tau  = [tau tau_t];
fc   = [fc fc_t];
f0   = [f0 f0_t];

end

for k=1:12

figure(4)
hold all
grid on
plot(t,pos(k,:))
title('feet position error')

figure(5)
hold all
grid on
plot(t,fc(k,:))
title('contact forces')


figure(6)
hold all
grid on
plot(t,f0(k,:))
title('f0')

end

for k=1:25

figure(7)
hold all
grid on
plot(t,tau(k,:))
title('torques at joints')

end

for k=1:3

figure(8)
hold all
grid on
plot(t,ecom(k,:))
title('CoM error')

figure(9)
hold all
grid on
plot(t,f0(12+k,:))
title('H_w_Desired')

end

figure(10)
hold on
grid on
plot(t,norm_tau)
title('norm of joints torques')

