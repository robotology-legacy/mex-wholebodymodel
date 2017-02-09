function diff_dxi_i = numericalDerivative(STATE,CONFIG,xi,ELASTICITY,mode,i)

import WBM.utilities.rotm2quat;
delta          = 0.0001;
feet_on_ground = CONFIG.feet_on_ground;
ndof           = CONFIG.ndof;
qjInit         = CONFIG.qjInit;

% trajectory generator
trajectory.jointReferences.ddqjRef = zeros(ndof,1);
trajectory.jointReferences.dqjRef  = zeros(ndof,1);
trajectory.jointReferences.qjRef   = qjInit;
trajectory.desired_x_dx_ddx_CoM    = [CONFIG.xCoMRef,zeros(3,1),zeros(3,1)];

if strcmp(mode,'position') == 1
    
    % add a delta to the i-th joint position, and recompute the base pose
    % w.r.t. the fixed link
    qj_plus            = STATE.qj;
    qj_plus(i)         = qj_plus(i) + delta;

    if  feet_on_ground(1) == 1
    
        [x_b,w_R_b]    = wbm_getWorldFrameFromFixLnk('l_sole',qj_plus);
    else
        [x_b,w_R_b]    = wbm_getWorldFrameFromFixLnk('r_sole',qj_plus);
    end

    qt_b               = rotm2quat(w_R_b);
    w_omega_b          = STATE.w_omega_b;
    dx_b               = STATE.dx_b;
    dqj                = STATE.dqj;
    chi_plus           = [x_b;qt_b;qj_plus;dx_b;w_omega_b;dqj];
    
    % recompute the robot dynamics, for kinematics and state
    STATE_plus         = robotState(chi_plus,CONFIG);
    DYNAMICS_plus      = robotDynamics(STATE_plus,CONFIG);
    FORKINEMATICS_plus = robotForKinematics(STATE_plus,DYNAMICS_plus);

    % evaluate dxi_ref_plus
    controlParam = stackOfTaskController(CONFIG,CONFIG.gainsInit,trajectory,DYNAMICS_plus,FORKINEMATICS_plus,STATE_plus,xi,ELASTICITY);

    if  CONFIG.use_QPsolver == 1                 
        % quadratic programming solver for the nullspace of contact forces
        controlParam.fcDes = QPSolver(controlParam,CONFIG,FORKINEMATICS_plus);
    end
    
    dxi_ref_plus = ELASTICITY.KD\(controlParam.tauModel+controlParam.Sigma*controlParam.fcDes);
    
    % subtract a delta to the i-th joint position, and recompute the base pose
    % w.r.t. the fixed link
    qj_minus            = STATE.qj;
    qj_minus(i)         = qj_minus(i) - delta;

    if  feet_on_ground(1) == 1
    
        [x_b,w_R_b]    = wbm_getWorldFrameFromFixLnk('l_sole',qj_minus);
    else
        [x_b,w_R_b]    = wbm_getWorldFrameFromFixLnk('r_sole',qj_minus);
    end

    qt_b               = rotm2quat(w_R_b);
    w_omega_b          = STATE.w_omega_b;
    dx_b               = STATE.dx_b;
    dqj                = STATE.dqj;
    chi_minus          = [x_b;qt_b;qj_minus;dx_b;w_omega_b;dqj];
    
    % recompute the robot dynamics, for kinematics and state
    STATE_minus         = robotState(chi_minus,CONFIG);
    DYNAMICS_minus      = robotDynamics(STATE_minus,CONFIG);
    FORKINEMATICS_minus = robotForKinematics(STATE_minus,DYNAMICS_minus);
    
    % evaluate dxi_ref_plus
    controlParam = stackOfTaskController(CONFIG,CONFIG.gainsInit,trajectory,DYNAMICS_minus,FORKINEMATICS_minus,STATE_minus,xi,ELASTICITY);

    if  CONFIG.use_QPsolver == 1                 
        % quadratic programming solver for the nullspace of contact forces
        controlParam.fcDes = QPSolver(controlParam,CONFIG,FORKINEMATICS_minus);
    end
    
    dxi_ref_minus = ELASTICITY.KD\(controlParam.tauModel+controlParam.Sigma*controlParam.fcDes);
    
    % differential
    diff_dxi_i = (dxi_ref_plus-dxi_ref_minus)/(2*delta);
        
elseif strcmp(mode,'velocity') == 1
    
    % add a delta to the i-th joint velocity, and recompute the base
    % velocity w.r.t. the fixed link
    dqj_plus           = STATE.dqj;
    dqj_plus(i)        = dqj_plus(i) + delta; 
    
    w_R_b              = STATE.w_R_b;
    x_b                = STATE.x_b;
    qj                 = STATE.qj;
    qt_b               = STATE.qt_b;

    if  feet_on_ground(1) == 1
    
        Jf  = wbm_jacobian(w_R_b,x_b,qj,'l_sole');
    else
        Jf  = wbm_jacobian(w_R_b,x_b,qj,'r_sole');
    end
    
    v_b                = -Jf(1:6,1:6)\Jf(1:6,7:end)*dqj_plus;
    chi_plus           = [x_b;qt_b;qj;v_b;dqj_plus];
    
    % recompute the robot dynamics, for kinematics and state
    STATE_plus         = robotState(chi_plus,CONFIG);
    DYNAMICS_plus      = robotDynamics(STATE_plus,CONFIG);
    FORKINEMATICS_plus = robotForKinematics(STATE_plus,DYNAMICS_plus);
    
    % evaluate dxi_ref_plus
    controlParam = stackOfTaskController(CONFIG,CONFIG.gainsInit,trajectory,DYNAMICS_plus,FORKINEMATICS_plus,STATE_plus,xi,ELASTICITY);

    if  CONFIG.use_QPsolver == 1                 
        % quadratic programming solver for the nullspace of contact forces
        controlParam.fcDes = QPSolver(controlParam,CONFIG,FORKINEMATICS_plus);
    end
    
    dxi_ref_plus = ELASTICITY.KD\(controlParam.tauModel+controlParam.Sigma*controlParam.fcDes);
    
    % subtract a delta to the i-th joint velocity, and recompute the base
    % velocity w.r.t. the fixed link
    dqj_minus          = STATE.dqj;
    dqj_minus(i)       = dqj_minus(i) - delta; 
    
    w_R_b              = STATE.w_R_b;
    x_b                = STATE.x_b;
    qj                 = STATE.qj;
    qt_b               = STATE.qt_b;

    if  feet_on_ground(1) == 1
    
        Jf  = wbm_jacobian(w_R_b,x_b,qj,'l_sole');
    else
        Jf  = wbm_jacobian(w_R_b,x_b,qj,'r_sole');
    end
    
    v_b                = -Jf(1:6,1:6)\Jf(1:6,7:end)*dqj_minus;
    chi_minus          = [x_b;qt_b;qj;v_b;dqj_minus];
    
    % recompute the robot dynamics, for kinematics and state
    STATE_minus         = robotState(chi_minus,CONFIG);
    DYNAMICS_minus      = robotDynamics(STATE_minus,CONFIG);
    FORKINEMATICS_minus = robotForKinematics(STATE_minus,DYNAMICS_minus);
    
    % evaluate dxi_ref_plus
    controlParam = stackOfTaskController(CONFIG,CONFIG.gainsInit,trajectory,DYNAMICS_minus,FORKINEMATICS_minus,STATE_minus,xi,ELASTICITY);

    if  CONFIG.use_QPsolver == 1                 
        % quadratic programming solver for the nullspace of contact forces
        controlParam.fcDes = QPSolver(controlParam,CONFIG,FORKINEMATICS_minus);
    end
    
    dxi_ref_minus = ELASTICITY.KD\(controlParam.tauModel+controlParam.Sigma*controlParam.fcDes);
    
    % differential
    diff_dxi_i = (dxi_ref_plus-dxi_ref_minus)/(2*delta);
    
elseif strcmp(mode,'motorPos') == 1
    
    % add a delta to the i-th motor position
    xi_plus            = xi;
    xi_plus(i)         = xi_plus(i) + delta;
    
    % recompute the robot dynamics, for kinematics and state
    STATE_plus         = STATE;
    DYNAMICS_plus      = robotDynamics(STATE_plus,CONFIG);
    FORKINEMATICS_plus = robotForKinematics(STATE_plus,DYNAMICS_plus);
    
    % evaluate dxi_ref_plus
    controlParam = stackOfTaskController(CONFIG,CONFIG.gainsInit,trajectory,DYNAMICS_plus,FORKINEMATICS_plus,STATE_plus,xi_plus,ELASTICITY);

    if  CONFIG.use_QPsolver == 1                 
        % quadratic programming solver for the nullspace of contact forces
        controlParam.fcDes = QPSolver(controlParam,CONFIG,FORKINEMATICS_plus);
    end
    
    dxi_ref_plus = ELASTICITY.KD\(controlParam.tauModel+controlParam.Sigma*controlParam.fcDes);
    
    % substract a delta to the i-th motor position
    xi_minus            = xi;
    xi_minus(i)         = xi_minus(i) - delta;
    
    % recompute the robot dynamics, for kinematics and state
    STATE_minus         = STATE;
    DYNAMICS_minus      = robotDynamics(STATE_minus,CONFIG);
    FORKINEMATICS_minus = robotForKinematics(STATE_minus,DYNAMICS_minus);
    
    % evaluate dxi_ref_plus
    controlParam = stackOfTaskController(CONFIG,CONFIG.gainsInit,trajectory,DYNAMICS_minus,FORKINEMATICS_minus,STATE_minus,xi_minus,ELASTICITY);

    if  CONFIG.use_QPsolver == 1                 
        % quadratic programming solver for the nullspace of contact forces
        controlParam.fcDes = QPSolver(controlParam,CONFIG,FORKINEMATICS_minus);
    end
    
    dxi_ref_minus = ELASTICITY.KD\(controlParam.tauModel+controlParam.Sigma*controlParam.fcDes);
    
    % differential
    diff_dxi_i = (dxi_ref_plus-dxi_ref_minus)/(2*delta);     
end

end
    