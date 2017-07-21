function dstvChi = forwardDynamics(obj, t, stvChi, fhTrqControl)
    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);

    wf_omega_b = stp.omega_b;
    v_b = vertcat(stp.dx_b, wf_omega_b); % generalized base velocity

    % update the state for the optimized mode
    % (else the procedure computes nonsense) ...
    setState(obj, stp.q_j, stp.dq_j, v_b);

    % get the current control torques ...
    tau = fhTrqControl(t);

    % reconstruct the rotation from the 'base' to the 'world' of
    % the quaternion part of the transformation vector vqT_b:
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
    % joint acceleration dnu = ddq_j:
    dnu = jointAccelerations(obj, tau, stp.dq_j); % optimized mode

    dstvChi = vertcat(nu, dnu);
end
