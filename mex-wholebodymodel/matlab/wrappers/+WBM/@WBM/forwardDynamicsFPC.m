function dstvChi = forwardDynamicsFPC(obj, t, stvChi, fhTrqControl, foot_conf, ac_f)
    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);

    wf_omega_b = stp.omega_b;
    v_b  = vertcat(stp.dx_b, wf_omega_b); % generalized base velocity
    nu_s = vertcat(v_b, stp.dq_j);        % mixed generalized velocity of the current state

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, foot_conf); % optimized mode

    % get the current control torques from the controller ...
    tau = fhTrqControl(t, M, c_qv, stp, nu_s, Jc_f, djcdq_f, foot_conf);

    % new mixed generalized velocity vector ...
    nu  = fdynNewMixedVelocities(obj, stp.qt_b, stp.dx_b, wf_omega_b, stp.dq_j);
    % joint acceleration dnu = ddq_j (optimized mode):
    dnu = jointAccelerationsFPC(obj, foot_conf, tau, ac_f, Jc_f, djcdq_f, ...
                                M, c_qv, stp.dq_j, nu_s);
    dstvChi = vertcat(nu, dnu);
end
