function dstvChi = forwardDynamicsPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, f_cp, ac_f)
    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);

    wf_omega_b = stp.omega_b;
    v_b = vertcat(stp.dx_b, wf_omega_b); % generalized base velocity

    % update the state for the optimized mode
    % (else the procedure computes nonsense) ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    % get the current control torques ...
    tau = fhTrqControl(t);

    % new mixed generalized velocity vector ...
    nu  = fdynNewMixedVelocities(obj, stp.qt_b, stp.dx_b, wf_omega_b, stp.dq_j);
    % joint acceleration dnu = ddq_j:
    dnu = jointAccelerationsPL(obj, tau, stp.dq_j, fhTotCWrench, feet_conf, hand_conf, f_cp, ac_f, stp.dq_j); % optimized mode

    dstvChi = vertcat(nu, dnu);
end
