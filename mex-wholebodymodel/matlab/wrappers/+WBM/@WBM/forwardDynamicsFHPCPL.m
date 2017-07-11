function dstvChi = forwardDynamicsFHPCPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, feet_conf, hand_conf, f_cp, ac_f)
    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);

    omega_w = stp.omega_b;
    v_b = vertcat(stp.dx_b, omega_w); % generalized base velocity
    nu  = vertcat(v_b, stp.dq_j);     % current mixed generalized velocity

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    % get the current control torques from the controller ...
    tau = fhTrqControl(t, M, c_qv, stp, nu, Jc, djcdq, feet_conf);

    % get the rotation matrix from the VQ-transformation (from 'base' to 'world frame') ...
    vqT_b = obj.stvqT;
    [~,wf_R_b] = WBM.utilities.frame2posRotm(vqT_b);

    % We need to apply the base-to-world rotation to the spatial angular velocity
    % omega_w to obtain the angular velocity in body frame omega_b. This is then
    % used in the quaternion derivative computation:
    omega_b = wf_R_b * omega_w;
    dqt_b   = WBM.utilities.dquat(stp.qt_b, omega_b);

    % new mixed generalized velocity ...
    v = vertcat(stp.dx_b, dqt_b, stp.dq_j);

    % joint acceleration dv (optimized mode):
    dv = jointAccelerationsFHPCPL(obj, feet_conf, hand_conf, fhTotCWrench, f_cp, tau, ac_f, dq_j, v_b, nu);

    dstvChi = vertcat(v, dv);
end
