function [] = kinEnergyConservationTest( params )

    %% initialise mexWholeBodyModel 
    if( params.isURDF )
        wbm_modelInitialiseFromURDF(params.urdfFilePath);
    else
        wbm_modelInitialise(params.yarpRobotName);
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
    gInit = wbm_generalisedBiasForces(Rtemp,Ptemp,params.qjInit,zeros(params.ndof,1),zeros(6,1));
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
    x_b = chi(:,1:3,:);
    qt_b = chi(:,4:7);
    qj = chi(:,8:ndof+7);
    x = [x_b qt_b qj];
    dx_b = chi(:,ndof+8:ndof+10);
    omega_W = chi(:,ndof+11:ndof+13);
    dqj = chi(:,ndof+14:2*ndof+13);
    
    v = [dx_b omega_W dqj ];
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
        plot(t,kinEnergy,'b');
        hold on;
        %plot(t,kinEnergy2,'r');
        xlabel('Time t(sec)');
        ylabel(' (J)');
        title('Kinetic Energy');
    end
    
    kinEnergy
    
    % figure;
    % plot(t,dqj);
    % xlabel('Time t(sec)');
    % ylabel('dqj (rad/sec)');
    % title('Joint velocities ');
    %
    % figure;
    % plot(t,dx_b);
    % xlabel('Time t(sec)');
    % ylabel('(m/sec)');
    % title('Floating Base translation velocities');
    %
    %
    % figure;
    % plot(t,omega_W);
    % xlabel('Time t(sec)');
    % ylabel(' (m/sec)');
    % title('Floating Base rotational velocities');
    %
    % ddx_b = chiDot(:,1:3);
    % domega_b = chiDot(:,4:6);
    % ddqj = chiDot(:,7:7+ndof);
    %
    % figure;
    % plot(t,ddqj);
    % xlabel('Time t(sec)');
    % ylabel('(rad/sec^2)');
    % title('Joint accelerations ');
    %
    % figure;
    % plot(t,ddx_b);
    % xlabel('Time t(sec)');
    % ylabel('(m/sec^2)');
    % title(' Floating Base translation acceleration ');
    %
    % figure;
    % plot(t,domega_b);
    % xlabel('Time t(sec)');
    % ylabel('(m/sec^2)');
    % title('Floating Base angular acceleration');
    %
    % figure;
    % plot(t,gOut);
    % xlabel('Time t(sec)');
    % ylabel('mixed units');
    % title('g');
    %
    % figure;
    % plot(t,hOut);
    % xlabel('Time t(sec)');
    % ylabel('mixed units');
    % title('h');
    %
    % figure(1);
    
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
    
end