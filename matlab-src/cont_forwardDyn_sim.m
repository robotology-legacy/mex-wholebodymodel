close all;
clear all;
clc;

% Script to integrate and test forward dynamics
addpath('./../build');
tic;
wholeBodyModel('model-initialise','icubGazeboSim');

fprintf('Initialisation time: %f secs\n',toc());
%[jl1, jl2] = wholeBodyModel('joint-limits');
load('./jointLimits.mat');

simtime = 0.6;
simstep = 0.01;


param.ndof = length(jl1);
%param.tau = @(t)zeros(param.ndof,1);

jointToActuate = 8;

jointTau = @(t)1.*sin(5*2*pi*t*(1/1));
%param.tau = @(t)0.000000001.*ones(param.ndof,1);
 param.tau = @(t)zeros(param.ndof,1);
%param.tau = @(t)0.000000001.*sin(2*pi*t*(1/10)).*ones(param.ndof,1);
% param.tau = @(t)jointTau(t).*ones(param.ndof,1);
% param.tau = @(t)[zeros(jointToActuate-1,1);jointTau(t);zeros(param.ndof-jointToActuate,1)];
%initial conditions

% floating_base = 'l_sole';

%qjInit = 0.5*(jl1+jl2);
torsoInit = [-50.0 0.0 0.0]';
leftArmInit = [49.7 29.7 0.0 44.9 0.0]';
rightArmInit = [49.7 29.7 0.0 44.9 0.0]';
leftLegInit =  [10.5 0.1 0.0 -5.5 -5.5 -0.1]';
rightLegInit = [10.5 0.1 0.0 -5.5 -5.5 -0.1]';


qjInit = pi*([torsoInit;leftArmInit;rightArmInit;leftLegInit;rightLegInit])/180;




%(0.55*jl2 + 0.45*jl1);% + rand(32,1)./100;
%zeros(32,1);

%disp('Initial Joint Configuration');
%disp(qjInit);
wholeBodyModel('update-state',qjInit,zeros(param.ndof,1),zeros(6,1));
             %wholeBodyModel('forward-kinematics',qj,refLink1)
%wholeBodyModel('visualize-trajectory',t,qjInit);
             
%T_baseInit = wholeBodyModel('forward-kinematics', qjInit,floating_base);
%T_baseInit(4:end) = convertAxisAngle2Quaternion(T_baseInit(4:end));
[qj,T_baseInit,qjDot,vb] = wholeBodyModel('get-state');

qjDotInit = zeros(length(qjInit),1);
v_baseInit = zeros(6,1);
% v_baseInit = [0.1;zeros(5,1)];

% T_baseInit(3) = T_baseInit(3)+0.75;

xInit = [T_baseInit;qjInit;qjDotInit;v_baseInit];

func = @(t,qv)cont_forwardDynamics_fun(t,qv,qjInit,param);
%pause;

disp('Starting Integration'); tic;
options = odeset('RelTol',1e-3,'AbsTol',1e-6);%,'Refine',2);
%options = odeset('InitialStep', 1e-16, 'MaxStep',1e-3);
[t,x] = ode15s(func,linspace(0,simtime,simtime/simstep),xInit,options);
%[t,x] = ode15s(func,linspace(2.0e-4,2.5e-4,1000),xInit,options);
%[t,x] = ode15s(func,linspace(0,2.5e-3,1000),xInit,options);
%[t,x] = ode23s(func,linspace(0,2.5e-3,1000),xInit,options);
fprintf('Integration took %f sec\n',toc());


%Fc check
%%
Fc_dat = [];
tau = param.tau;
for ii=1:size(x,1)
    
qj_test = x(ii,8:32)';  
qjDot_test = x(ii,39:end)';  
pDot_base_test = x(ii,33:35)';
omega_base_test = x(ii,36:38)';
t_test = t(ii);

wholeBodyModel('update-state',qj_test,qjDot_test,[pDot_base_test;omega_base_test]);

M_test = wholeBodyModel('mass-matrix');    
%MTilde = wholeBodyModel('mass-matrix',qj);

H_test = wholeBodyModel('generalised-forces');      
    
    
[Fc_test,J_test] = constraintBothFeetOnGround(qj_test,qjDot_test,tau(t_test), M_test, H_test);

Fc_dat = [Fc_dat;Fc_test'];

end

Fc_left_force_ts = timeseries(Fc_dat(:,1:3),t);
Fc_left_torque_ts = timeseries(Fc_dat(:,4:6),t);
Fc_right_force_ts = timeseries(Fc_dat(:,7:9),t);
Fc_right_torque_ts = timeseries(Fc_dat(:,10:12),t);

figure(1);
subplot(2,2,1);
plot(Fc_left_force_ts);
legend('Fx','Fy','Fz');
title('Fc left forces');
subplot(2,2,2);
plot(Fc_left_torque_ts);
legend('Tx','Ty','Tz');
title('Fc left torques');
subplot(2,2,3);
plot(Fc_right_force_ts);
legend('Fx','Fy','Fz');
title('Fc right forces');
subplot(2,2,4);
plot(Fc_right_torque_ts);
legend('Tx','Ty','Tz');
title('Fc right torques');

figure;
subplot(2,1,1);
plot(t,x(:,1:3));
legend('x','y','z');
subplot(2,1,2);
plot(t,x(:,1:3));
legend('x','y','z');


tXbOut = x(:,1:7);
qjOut = x(:,8:7+param.ndof);
%%
% plotQuat(tXbOut);

cont_visualize(x,simtime,simstep);
% 
% disp('I sent');
% disp(qjOut(:,1:3));
% drawnow;
fprintf('Visualization took %f sec \n',toc());
simtime
%%
% tic;

% tXbOut(:,3) = tXbOut(:,3) + 0.5*ones(size(x,1),1);
% wholeBodyModel('visualize-trajectory',t*10,180*(qjOut)/pi,tXbOut);
% fprintf('Visualization took %f sec \n',toc());


% figure(1);
% plot(t,x(:,1:3)); axis tight;
% title('Floating Base position vs Time');
% xlabel('Time t(sec)');
% ylabel('Position (m)');
% legend('x','y','z');
% 
% figure(2);
% plot(t,x(:,4:8)); axis tight;
% title('Floating Base Orientation vs Time');de
% xlabel('Time t(sec)');
% ylabel('Orientation (quaternion components)');
% legend('q0','q1','q2','q3');
% 
% figure(3);
% plot(t,rad2deg(qjOut(:,1:3))); axis tight;
% title('Torso Joint Angles vs Time');
% xlabel('Time t(sec)');
% ylabel('Angles in Deg');
% 
% figure(4);
% subplot(2,1,1);
% plot(t,rad2deg(qjOut(:,4:8)));axis tight;
% title('LeftArm Angles vs Time');
% xlabel('Time t(sec)');
% ylabel('Angles in Deg');
% 
% 
% %figure(4);
% subplot(2,1,2);
% plot(t,rad2deg(qjOut(:,9:13)));axis tight;
% title('RightArm Angles vs Time');
% xlabel('Time t(sec)');
% ylabel('Angles in Deg');
