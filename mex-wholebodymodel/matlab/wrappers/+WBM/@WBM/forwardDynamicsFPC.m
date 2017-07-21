function dstvChi = forwardDynamicsFPC(obj, t, stvChi, fhTrqControl, feet_conf, ac_f)
    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);

    wf_omega_b = stp.omega_b;
    v_b  = vertcat(stp.dx_b, wf_omega_b); % generalized base velocity
    nu_s = vertcat(v_b, stp.dq_j);        % mixed generalized velocity of the current state

    % update the state for the optimized mode ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, feet_conf); % optimized mode

    % get the current control torques from the controller ...
    tau = fhTrqControl(t, M, c_qv, stp, nu_s, Jc_f, djcdq_f, feet_conf);

    % get the rotation matrix from the VQ-transformation (from 'base' to 'world frame') ...
    vqT_b = obj.vqT_base;
    [~,wf_R_b] = WBM.utilities.tfms.frame2posRotm(vqT_b);

    % We need to apply the world-to-base rotation b_R_wf to the spatial angular
    % velocity wf_omega_b to obtain the angular velocity b_omega_wf in the base
    % body frame. This is then used in the quaternion derivative computation:
    b_R_wf = wf_R_b.';
    b_omega_wf = b_R_wf * wf_omega_b;
    dqt_b      = WBM.utilities.tfms.dquat(stp.qt_b, b_omega_wf);

    % new mixed generalized velocity ...
    nu = vertcat(stp.dx_b, dqt_b, stp.dq_j);
    % joint acceleration dnu = ddq_j (optimized mode):
    dnu = jointAccelerationsFPC(obj, feet_conf, tau, ac_f, Jc_f, ...
                                djcdq_f, M, c_qv, stp.dq_j, nu_s);
    dstvChi = vertcat(nu, dnu);
end
