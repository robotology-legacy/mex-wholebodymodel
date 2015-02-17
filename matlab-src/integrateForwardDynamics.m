%% setup path
addpath('./whole_body_model_functions/');
addpath('./../build/');
addpath('./worker_functions');

%% initialise mexWholeBodyModel
wbm_modelInitialise('icubGazeboSim');


%% setup params
params.ndof = 25;
param.dampingCoeff = 0.25;

%% initial conditions
% this is assuming a 25DoF iCub
params.torsoInit    = [-10.0  0.0   0.0]';
params.leftArmInit  = [ -19.7  29.7  0.0  44.9  0.0]';
params.rightArmInit = [ -19.7  29.7  0.0  44.9  0.0]';
params.leftLegInit  = [ 25.5   0.1  0.0 -18.5 -5.5 -0.1]';
params.rightLegInit = [ 25.5   0.1  0.0 -18.5 -5.5 -0.1]';

params.qjInit = [params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit];
params.dqjInit = zeros(params.ndof,1);
%params.x_bInit =  zeros(3,1);
%params.qt_bInit = zeros(4,1);
params.dx_bInit = zeros(3,1);
params.omega_bInit = zeros(3,1);
params.dampingCoeff = 0.75;

wbm_updateState(params.qjInit,zeros(params.ndof,1),zeros(6,1));
[qj,T_bInit,dqj,vb] = wholeBodyModel('get-state');
% flipping quaternion organisation (to real followed by imaginary
% convention)
T_bInit_mod = [T_bInit(1:3);T_bInit(7);T_bInit(4:6)];
params.chiInit = [T_bInit_mod;params.qjInit;...
                    params.dx_bInit;params.omega_bInit;params.dqjInit];
params.dampingCoeff = 0.1;

%% contact constraints                
params.constraintLinkNames = {'l_sole','r_sole'};                

%% control torques
gInit = wbm_generalisedBiasForces(params.qjInit,zeros(25,1),zeros(6,1));
%params.tau = @(t)zeros(size(gInit(1:params.ndof)));%gInit(1:params.ndof);
params.tau = @(t)gInit(1:params.ndof);

%% setup integration
forwardDynFunc = @(t,chi)forwardDynamics(t,chi,params);
tStart = 0;
tEnd = 1;

%% integrate forward dynamics
disp('starting integration');
options = odeset('RelTol',1e-3,'AbsTol',1e-6);
[t,chi] = ode15s(forwardDynFunc,[tStart tEnd],params.chiInit,options);

%% plot results
% CoM trajectory
ndof = params.ndof;
x_b = chi(:,1:3,:);
qt_b = chi(:,4:7);
qj = chi(:,8:ndof+7);

plot3(x_b(:,1),x_b(:,2),x_b(:,3));hold on;
plot3(x_b(1,1),x_b(1,2),x_b(1,3),'ro');
grid on;
axis square;
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');