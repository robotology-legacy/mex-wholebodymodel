%% setup path
addpath('./whole_body_model_functions/');
addpath('./../build/');
addpath('./worker_functions');

%% initialise mexWholeBodyModel
wbm_modelInitialise('icubGazeboSim');


%% setup params
params.ndof = 25;% 25;
%param.dampingCoeff = 0.25;

%% random initial conditions

load('./jointLimits.mat');

%if(exist('./randomIni.mat','file')==0)
    deltaJl = jl2 - jl1;
    %params.qjInit = 0;
    qjInit=jl1 + rand(25,1) .* deltaJl ;%[params.torsoInit;params.leftArmInit;params.rightArmInit;params.leftLegInit;params.rightLegInit];
    maxVel = 10;
    %params.dqjInit = 0;%0 * rand(25,1);%zeros(params.ndof,1);
    %params.dqjInit = zeros(params.ndof,1);
    dqjInit =maxVel*(0.5*ones(25,1)-1*rand(25,1));
%    save('./randomIni.mat','qjInit','dqjInit');
%else
%    load('./randomIni.mat');
%end

params.qjInit = qjInit;
params.dqjInit = dqjInit;

%params.dqjInit =zeros(size(params.dqjInit));
%params.x_bInit =  zeros(3,1);
%params.qt_bInit = zeros(4,1);
params.dx_bInit = 2.5*rand(3,1) - 5*ones(3,1); %zeros(3,1);
params.omega_bInit = 2.5*rand(3,1) - 5*ones(3,1);%zeros(3,1);
params.dampingCoeff = 0;%0.75;


fprintf('Random Initial configuration\n');
disp(params.qjInit');
fprintf('Random Initial velocity\n');
disp(params.dqjInit');

wbm_setWorldFrame(eye(3),[0 0 0]',[0 0 0]');

wbm_updateState(params.qjInit,params.dqjInit,[params.dx_bInit;params.omega_bInit]);
%[qj,T_bInit,dqj,vb] = wholeBodyModel('get-state');

[qj,T_bInit,dqj,vb] = wbm_getState();
[Ptemp,Rtemp] = frame2posrot(T_bInit);
params.chiInit = [T_bInit;params.qjInit;...
                    params.dx_bInit;params.omega_bInit;params.dqjInit];
%% contact constraints                
params.constraintLinkNames = {};%{'l_sole','r_sole'};                

%% control torques
gInit = wbm_generalisedBiasForces(Rtemp,Ptemp,params.qjInit,zeros(25,1),zeros(6,1));
params.tau = @(t)zeros(params.ndof,1);%gInit(1:params.ndof);
%params.tau = @(t)gInit(1:params.ndof);

%% setup integration
forwardDynFunc = @(t,chi)forwardDynamics_zeroExternalForces(t,chi,params);
tStart = 0;
tEnd = 10.0;%5.0;

%% integrate forward dynamics
disp('starting integration');
options = odeset('RelTol',1e-4,'AbsTol',1e-8);
[t,chi] = ode15s(forwardDynFunc,[tStart tEnd],params.chiInit,options);

%% plot results
% CoM trajectory
ndof = params.ndof;
x_b = chi(:,1:3,:);
qt_b = chi(:,4:7);
qj = chi(:,8:ndof+7);
x = [x_b qt_b qj];
dx_b = chi(:,ndof+8:ndof+10);
omega_W = chi(:,ndof+11:ndof+13);
dqj = chi(:,ndof+14:2*ndof+13);

v = [dx_b omega_W dqj ];
%v = dx;
kinEnergy = zeros(length(t),1);
%kinEnergy2 = zeros(length(t),1);
chiDot = zeros(length(t),size(chi,2));
hOut = zeros(length(t),ndof+6);
gOut = zeros(length(t),ndof+6);

fc = zeros(length(t),ndof+6);
wbm_setWorldFrame(eye(3),[0 0 0]', [0 0 0]');

for tCnt = 1:length(t)
    [chiDot(tCnt,:),hOut(tCnt,:),gOut(tCnt,:),~,kinEnergy(tCnt) ] = forwardDynamics_zeroExternalForces(t(tCnt,:),chi(tCnt,:)',params);    
   % [chiDot(tCnt,:),hOut(tCnt,:),gOut(tCnt,:),~,~ ] = forwardDynamics_zeroExternalForces(t(tCnt,:),chi(tCnt,:)',params);    
    %kinEnergy2(tCnt) = 0.5 * v(tCnt,:) * wbm_massMatrix(qj(tCnt,:)') * v(tCnt,:)';
end

plot(t,kinEnergy,'b');
hold on;
%plot(t,kinEnergy2,'r');
xlabel('Time t(sec)');
ylabel(' (J)');
title('Kinetic Energy');

figure;
plot(t,dqj);
xlabel('Time t(sec)');
ylabel('dqj (rad/sec)');
title('Joint velocities ');

figure;
plot(t,dx_b);
xlabel('Time t(sec)');
ylabel('(m/sec)');
title('Floating Base translation velocities');


figure;
plot(t,omega_W);
xlabel('Time t(sec)');
ylabel(' (m/sec)');
title('Floating Base rotational velocities');

ddx_b = chiDot(:,1:3);
domega_b = chiDot(:,4:6);
ddqj = chiDot(:,7:7+ndof);

figure;
plot(t,ddqj);
xlabel('Time t(sec)');
ylabel('(rad/sec^2)');
title('Joint accelerations ');

figure;
plot(t,ddx_b);
xlabel('Time t(sec)');
ylabel('(m/sec^2)');
title(' Floating Base translation acceleration ');

figure;
plot(t,domega_b);
xlabel('Time t(sec)');
ylabel('(m/sec^2)');
title('Floating Base angular acceleration');

figure;
plot(t,gOut);
xlabel('Time t(sec)');
ylabel('mixed units');
title('g');

figure;
plot(t,hOut);
xlabel('Time t(sec)');
ylabel('mixed units');
title('h');

figure(1);

% figure;
% plot(t,fc);
% xlabel('Time t(sec)');
% ylabel('mixed units');
% title('fc');

% plot3(x_b(:,1),x_b(:,2),x_b(:,3));hold on;
% plot3(x_b(1,1),x_b(1,2),x_b(1,3),'ro');
% grid on;
% axis square;
% xlabel('X(m)');
% ylabel('Y(m)');
% zlabel('Z(m)');