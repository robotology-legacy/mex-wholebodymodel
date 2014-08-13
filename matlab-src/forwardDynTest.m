% Script to integrate and test forward dynamics
addpath('./../build');
wholeBodyModel('model-initialise');

%[jl1, jl2] = wholeBodyModel('joint-limits');

param.ndof = 32;%length(jl1);
param.tau = @(t)zeros(param.ndof,1);

%initial conditions

floating_base = 'l_sole';

qjInit = rand(32,1)./100;
%zeros(32,1);

             %wholeBodyModel('forward-kinematics',qj,refLink1)
T_baseInit = wholeBodyModel('forward-kinematics', qjInit,floating_base);
T_baseInit(4:end) = convertAxisAngle2Quaternion(T_baseInit(4:end));

qjDotInit = zeros(length(qjInit),1);
v_baseInit = zeros(6,1);

qvInit = [T_baseInit;qjInit;qjDotInit;v_baseInit];

func = @(t,qv)forwardDynamics(t,qv,param);

options = odeset('RelTol',1e-3,'AbsTol',1e-5);
%options = odeset('I
[t,qv] = ode45(func,[0 1],qvInit,options);



