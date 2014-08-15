% Script to integrate and test forward dynamics
addpath('./../build');
wholeBodyModel('model-initialise','icubGazeboSim');

[jl1, jl2] = wholeBodyModel('joint-limits');

param.ndof = length(jl1);
param.tau = @(t)zeros(param.ndof,1);

%initial conditions

floating_base = 'l_sole';

qjInit = 0.5 * (jl2 + jl1) + rand(32,1)./100;
%zeros(32,1);

             %wholeBodyModel('forward-kinematics',qj,refLink1)
T_baseInit = wholeBodyModel('forward-kinematics', qjInit,floating_base);
T_baseInit(4:end) = convertAxisAngle2Quaternion(T_baseInit(4:end));

qjDotInit = zeros(length(qjInit),1);
v_baseInit = zeros(6,1);

qvInit = [T_baseInit;qjInit;qjDotInit;v_baseInit];

func = @(t,qv)forwardDynamics(t,qv,param);

options = odeset('RelTol',1e-5,'AbsTol',1e-10);
%options = odeset('I
[t,qv] = ode45(func,[0 0.001],qvInit,options);



