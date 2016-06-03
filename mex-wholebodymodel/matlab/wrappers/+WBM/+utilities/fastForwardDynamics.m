function [dstvChi, C_qv] = fastForwardDynamics(t, stvChi, ctrlTrqs, robot_model, robot_config)
    ndof         = robot_model.ndof;
    nCstrs       = robot_config.nCstrs;
    vfrict_coeff = robot_model.vfrict_coeff;
    cfrict_coeff = robot_model.cfrict_coeff;

    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.fastGetStateParams(stvChi, robot_config.stvLen, ndof);

    omega_w = stp.omega_b;
    v_b = vertcat(stp.dx_b, omega_w);
    %v = [stp.dx_b; omega_w; stp.dq_j];

    % update the state for the optimized mode (precautionary) ...
    wbm_updateState(stp.q_j, stp.dq_j, v_b);

    % reconstruct the rotation of the 'root link' to the 'world'
    % from the quaternion part of the transformation vector vqT_b:
    [~,vqT_b,~,~] = wbm_getState();
    [~,R_b] = WBM.utilities.frame2posRotm(vqT_b);

    M    = wbm_massMatrix();
    C_qv = wbm_generalisedBiasForces();

    % compute for each contact constraint the Jacobian and the corresponding
    % derivative Jacobian:
    m = 6*nCstrs;
    n = 6 + ndof;
    Jc = zeros(m,n);
    dJcdq = zeros(m,1);
    for i = 1:nCstrs % parallelizable?
        Jc(6*i-5:6*i,1:n)  = wbm_jacobian(robot_config.cstr_link_names{i});
        dJcdq(6*i-5:6*i,1) = wbm_djdq(robot_config.cstr_link_names{i});
    end

    % get the current control torque vector ...
    tau = ctrlTrqs.tau(t);

    % compute the contact force vector:
    Jc_t      =  Jc.';
    JcMinv    =  Jc / M;
    JcMinvJct =  JcMinv * Jc_t;
    tau_fr    =  calcFrictionForces(stp.dq_j, vfrict_coeff, cfrict_coeff); % damped torques
    tau_gen   =  vertcat(zeros(6,1), tau + tau_fr); % generalized force tau_gen = tau - tau_fr
    % calculate the contact (constraint) forces ...
    f_c = JcMinvJct \ (JcMinv*(C_qv - tau_gen) - dJcdq);

    % need to apply root-to-world rotation to the spatial angular velocity omega_w to
    % obtain angular velocity in body frame omega_b. This is then used in the
    % quaternion derivative computation:
    omega_b = R_b * omega_w;
    dqt_b   = WBM.utilities.dQuat(stp.qt_b, omega_b);

    dx = vertcat(stp.dx_b, dqt_b, stp.dq_j);
    dv = M \ (Jc_t*f_c + tau_gen - C_qv);
    %dv = M \ (Jc.'*f_c + tau_gen - C_qv); % cause Jc.'*f_c round-off errors?

    dstvChi = vertcat(dx, dv);
    %kinEnergy = 0.5*v.'*M*v;
end
%% END of fastForwardDynamics.


function tau_fr = calcFrictionForces(dq_j, vfrict_coeff, cfrict_coeff)
    epsilon = 1e-12; % min. value to treat a number as zero ...

    if (sum(dq_j ~= 0) <= epsilon) % if dq_j = 0:
        tau_fr = zeros(size(dq_j,1),1);
        return
    end
    tau_vf = -vfrict_coeff .* dq_j;       % viscous friction torques
    tau_cf = -cfrict_coeff .* sign(dq_j); % Coulomb friction torques
    tau_fr =  tau_vf + tau_cf;            % friction torques
end
