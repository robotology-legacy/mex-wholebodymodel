function INIT_CONDITIONS  = addStepReference(MODEL,INIT_CONDITIONS) 

    delta            = MODEL.CONFIG.stepAmplitude;
    qjInit           = INIT_CONDITIONS.qjInit;
    qjInit(1:13)     = INIT_CONDITIONS.qjInit(1:13) + delta*pi/180;
    % update robot state using the new initial conditions  
    wbm_updateState(qjInit,INIT_CONDITIONS.dqjInit,[INIT_CONDITIONS.dx_bInit;INIT_CONDITIONS.w_omega_bInit]);
    % fixing the world reference frame w.r.t. the foot on ground position
    [x_bInit,w_R_bInit]  = wbm_getWorldFrameFromFixLnk(INIT_CONDITIONS.CONFIG.constraintLinkNames{1},qjInit);
    wbm_setWorldFrame(w_R_bInit,x_bInit,[0 0 -9.81]')
    % get current base pose
    [basePoseInit,~,~,~] = wbm_getState();
    % new initial robot state (floating base + joints)
    INIT_CONDITIONS.chi_robotInit = [basePoseInit; qjInit; [INIT_CONDITIONS.dx_bInit;INIT_CONDITIONS.w_omega_bInit]; INIT_CONDITIONS.dqjInit];
    
    %% Update state demux, dynamics and forward kinematics
    % initial state
    INIT_CONDITIONS.INITSTATE         = robotState(INIT_CONDITIONS.chi_robotInit,MODEL);
    % initial dynamics
    INIT_CONDITIONS.INITDYNAMICS      = robotDynamics(INIT_CONDITIONS.initState,MODEL);
    % initial forward kinematics
    INIT_CONDITIONS.INITFORKINEMATICS = robotForKinematics(INIT_CONDITIONS.INITSTATE,INIT_CONDITIONS.INITDYNAMICS);   
end