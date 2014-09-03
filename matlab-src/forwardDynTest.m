% Script to integrate and test forward dynamics
addpath('./../build');
tic;
wholeBodyModel('model-initialise','icubGazeboSim');

fprintf('Initialisation time: %f secs\n',toc());
%[jl1, jl2] = wholeBodyModel('joint-limits');
load('./jointLimits.mat');


param.ndof = length(jl1);
%param.tau = @(t)zeros(param.ndof,1);


jointToActuate = 8;

jointTau = @(t)1.*sin(5*2*pi*t*(1/1));
%param.tau = @(t)0.000000001.*ones(param.ndof,1);
%param.tau = @(t)zeros(param.ndof,1);
%param.tau = @(t)0.000000001.*sin(2*pi*t*(1/10)).*ones(param.ndof,1);
param.tau = @(t)jointTau(t).*ones(param.ndof,1);
%param.tau = @(t)[zeros(jointToActuate-1,1);jointTau(t);zeros(param.ndof-jointToActuate,1)];
%initial conditions

floating_base = 'l_sole';

%qjInit = 0.5*(jl1+jl2);
torsoInit = [0.0 0.0 4]';
leftArmInit = [-29.7 29.7 0.0 44.9 0.0]';
rightArmInit = [-29.7 29.7 0.0 44.9 0.0]';
leftLegInit = [0.5 0.1 0.0 0.5 -0.5 -0.1]';
rightLegInit = [0.5 0.1 0.0 0.5 -0.5 -0.1]';


qjInit = deg2rad([torsoInit;leftArmInit;rightArmInit;leftLegInit;rightLegInit]);




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
%v_baseInit = [0.01;zeros(5,1)];

T_baseInit(3) = T_baseInit(3)+0.75;

xInit = [T_baseInit;qjInit;qjDotInit;v_baseInit];

func = @(t,qv)forwardDynamics(t,qv,param);
%pause;

disp('Starting Integration'); tic;
options = odeset('RelTol',1e-3,'AbsTol',1e-6);%,'Refine',2);
%options = odeset('InitialStep', 1e-16, 'MaxStep',1e-3);
[t,x] = ode15s(func,linspace(0,2.5,1000),xInit,options);
%[t,x] = ode15s(func,linspace(2.0e-4,2.5e-4,1000),xInit,options);
%[t,x] = ode15s(func,linspace(0,2.5e-3,1000),xInit,options);
%[t,x] = ode23s(func,linspace(0,2.5e-3,1000),xInit,options);
fprintf('Integration took %f sec\n',toc());

tXbOut = x(:,1:7);
qjOut = x(:,8:7+param.ndof);

% 
% disp('I sent');
% disp(qjOut(:,1:3));
tic;
wholeBodyModel('visualize-trajectory',t*10,rad2deg(qjOut),tXbOut);
fprintf('Visualization took %f sec \n',toc());


figure(1);
plot(t,x(:,1:3)); axis tight;
title('Floating Base position vs Time');
xlabel('Time t(sec)');
ylabel('Position (m)');
legend('x','y','z');

figure(2);
plot(t,x(:,4:8)); axis tight;
title('Floating Base Orientation vs Time');
xlabel('Time t(sec)');
ylabel('Orientation (quaternion components)');
legend('q0','q1','q2','q3');

figure(3);
plot(t,rad2deg(qjOut(:,1:3))); axis tight;
title('Torso Joint Angles vs Time');
xlabel('Time t(sec)');
ylabel('Angles in Deg');

figure(4);
subplot(2,1,1);
plot(t,rad2deg(qjOut(:,4:8)));axis tight;
title('LeftArm Angles vs Time');
xlabel('Time t(sec)');
ylabel('Angles in Deg');


%figure(4);
subplot(2,1,2);
plot(t,rad2deg(qjOut(:,9:13)));axis tight;
title('RightArm Angles vs Time');
xlabel('Time t(sec)');
ylabel('Angles in Deg');
