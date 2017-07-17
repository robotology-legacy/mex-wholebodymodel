function dstvChi = forwardDynamicsFHPC(obj, t, stvChi, fhTrqControl, feet_conf, hand_conf, fe_h, ac_f)
    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);

    omega_w = stp.omega_b;
    v_b = vertcat(stp.dx_b, omega_w); % generalized base velocity
    nu  = vertcat(v_b, stp.dq_j);     % current mixed generalized velocity

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, feet_conf); % optimized mode

    % get the current control torques from the controller ...
    tau = fhTrqControl(t, M, c_qv, stp, nu, Jc_f, djcdq_f, feet_conf);

    % get the rotation matrix from the VQ-transformation (from 'base' to 'world frame') ...
    vqT_b = obj.vqT_base;
    [~,wf_R_b] = WBM.utilities.tfms.frame2posRotm(vqT_b);

    % We need to apply the base-to-world rotation to the spatial angular velocity
    % omega_w to obtain the angular velocity in body frame omega_b. This is then
    % used in the quaternion derivative computation:
    omega_b = wf_R_b * omega_w;
    dqt_b   = WBM.utilities.tfms.dquat(stp.qt_b, omega_b);

    % new mixed generalized velocity ...
    v = vertcat(stp.dx_b, dqt_b, stp.dq_j);
    % joint acceleration dv (optimized mode):
    dv = jointAccelerationsFHPC(obj, feet_conf, hand_conf, tau, fe_h, ac_f, ...
                                Jc_f, djcdq_f, M, c_qv, stp.dq_j, nu);
    dstvChi = vertcat(v, dv);
end
