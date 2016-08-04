function [] = kinEnergyConservationTest( params )
rng(0)

clear wholeBodyModel;

%% initialise mexWholeBodyModel
if( params.isURDF )
    wbm_modelInitialiseFromURDF(params.urdfFilePath);
    robotDisplayName = params.urdfFilePath;
else
    wbm_modelInitialise(params.yarpRobotName);
    robotDisplayName = params.yarpRobotName;
end

%% get limits
[jlMin,jlMax] = wbm_jointLimits();

%% setup params
params.ndof = size(jlMin,1);

%% random initial conditions inside

deltaJl = jlMax - jlMin;
qjInit=jlMin + rand(params.ndof,1) .* deltaJl ;
maxVel = 10;

dqjInit =maxVel*(0.5*ones(params.ndof,1)-1*rand(params.ndof,1));

params.qjInit = qjInit;
params.dqjInit = dqjInit;

params.dx_bInit = 2.5*rand(3,1) - 5*ones(3,1); %zeros(3,1);
params.omega_bInit = 2.5*rand(3,1) - 5*ones(3,1);%zeros(3,1);
params.dampingCoeff = 0;


fprintf('fwdDynKinEnergyTest: Random Initial configuration\n');
disp(params.qjInit');
fprintf('fwdDynKinEnergyTest: Random Initial velocity\n');
disp(params.dqjInit');

wbm_setWorldFrame(eye(3),[0 0 0]',[0 0 0]');
wbm_updateState(params.qjInit,params.dqjInit,[params.dx_bInit;params.omega_bInit]);

[qj,T_bInit,dqj,vb] = wbm_getState();
[Ptemp,Rtemp] = frame2posrot(T_bInit);
params.chiInit = [T_bInit;params.qjInit;...
    params.dx_bInit;params.omega_bInit;params.dqjInit];

%% contact constraints (no constraint, free floating system)
params.constraintLinkNames = {};

%% no control torques: zero input torques
params.tau = @(t)zeros(params.ndof,1);

%% setup integration
forwardDynFunc = @(t,chi)forwardDynamics_kinEnergyTest(t,chi,params);
tStart = 0;
tEnd = params.simulationLengthInSecs;

%% integrate forward dynamics
disp('starting integration');
options = odeset('RelTol',1e-5,'AbsTol',1e-7);
[t,chi] = ode15s(forwardDynFunc,[tStart tEnd],params.chiInit,options);

%% plot results
ndof = params.ndof;

params.demux.baseOrientationType = 1;  % sets the base orientation in stateDemux.m as positions + quaternions (1) or transformation matrix (0)
[basePose,qj,baseVelocity,dqj]   = stateDemux(chi,params);

% position and orientation
x_b     = basePose(1:3,:);

% normalize quaternions to avoid numerical errors
% qt_b = qt_b/norm(qt_b);

qt_b    = basePose(4:7,:);

% linear and angular velocity
dx_b    = baseVelocity(1:3,:);
omega_W = baseVelocity(4:6,:);

x = [x_b' qt_b' qj'];
v = [dx_b' omega_W' dqj' ];
kinEnergy = zeros(length(t),1);
chiDot = zeros(length(t),size(chi,2));
hOut = zeros(length(t),ndof+6);
gOut = zeros(length(t),ndof+6);

fc = zeros(length(t),ndof+6);
wbm_setWorldFrame(eye(3),[0 0 0]', [0 0 0]');

for tCnt = 1:length(t)
    [chiDot(tCnt,:),hOut(tCnt,:),gOut(tCnt,:),~,kinEnergy(tCnt) ] = forwardDynamics_kinEnergyTest(t(tCnt,:),chi(tCnt,:)',params);
end

if( params.plot )
    figure
    plot(t,kinEnergy,'b');
    hold on;
    %plot(t,kinEnergy2,'r');
    xlabel('Time t(sec)');
    ylabel(' (J)');
    title(['Kinetic Energy for ',robotDisplayName]);
end

kinEnergyMaxErrRel = max(abs(kinEnergy-kinEnergy(1)))/kinEnergy(1);

fprintf('Relative error for energy for model %s: %f\n',robotDisplayName,kinEnergyMaxErrRel);

if( params.raiseErrorOnFail )
    WBMAssertEqual(kinEnergyMaxErrRel,0,'Kinetic energy is not constant',params.relTol);
end

%     figure;
%     plot(t,dqj);
%     xlabel('Time t(sec)');
%     ylabel('dqj (rad/sec)');
%     title('Joint velocities ');
%
%     figure;
%     plot(t,dx_b);
%     xlabel('Time t(sec)');
%     ylabel('(m/sec)');
%     title('Floating Base translation velocities');
%
%
%     figure;
%     plot(t,omega_W);
%     xlabel('Time t(sec)');
%     ylabel(' (m/sec)');
%     title('Floating Base rotational velocities');
%
%     ddx_b = chiDot(:,1:3);
%     domega_b = chiDot(:,4:6);
%     ddqj = chiDot(:,7:7+ndof);
%
%     figure;
%     plot(t,ddqj);
%     xlabel('Time t(sec)');
%     ylabel('(rad/sec^2)');
%     title('Joint accelerations ');
%
%     figure;
%     plot(t,ddx_b);
%     xlabel('Time t(sec)');
%     ylabel('(m/sec^2)');
%     title(' Floating Base translation acceleration ');
%
%     figure;
%     plot(t,domega_b);
%     xlabel('Time t(sec)');
%     ylabel('(m/sec^2)');
%     title('Floating Base angular acceleration');
%
%     figure;
%     plot(t,gOut);
%     xlabel('Time t(sec)');
%     ylabel('mixed units');
%     title('g');
%
%     figure;
%     plot(t,hOut);
%     xlabel('Time t(sec)');
%     ylabel('mixed units');
%     title('h');
%
%     figure(1);
%
%     figure;
%     plot(t,fc);
%     xlabel('Time t(sec)');
%     ylabel('mixed units');
%     title('fc');
%
%     plot3(x_b(:,1),x_b(:,2),x_b(:,3));hold on;
%     plot3(x_b(1,1),x_b(1,2),x_b(1,3),'ro');
%     grid on;
%     axis square;
%     xlabel('X(m)');
%     ylabel('Y(m)');
%     zlabel('Z(m)');

end