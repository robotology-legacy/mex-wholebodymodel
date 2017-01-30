function dstvChi = forwardDynamics(obj, t, stvChi, fhTrqControl)
    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);

    omega_w = stp.omega_b;
    v_b = vertcat(stp.dx_b, omega_w); % generalized base velocity
    %nu  = vertcat(v_b, stp.dq_j);

    % update the state for the optimized mode (precautionary) ...
    obj.setState(stp.q_j, stp.dq_j, v_b);

    % get the current control torques ...
    [tau,~] = fhTrqControl(t);

    % reconstruct the rotation from the 'base' to the 'world'
    % of the quaternion part of the transformation vector vqT_b:
    vqT_b   = obj.stvqT;
    [~,R_b] = WBM.utilities.frame2posRotm(vqT_b);

    % We need to apply the base-to-world rotation to the spatial angular velocity
    % omega_w to obtain the angular velocity in body frame omega_b. This is then
    % used in the quaternion derivative computation:
    omega_b = R_b * omega_w;
    dqt_b   = WBM.utilities.dQuat(stp.qt_b, omega_b);

    % velocities:
    dx = vertcat(stp.dx_b, dqt_b, stp.dq_j);
    % joint acceleration dv:
    [dv,~] = jointAccelerations(obj, stp.dq_j, tau); % optimized mode

    dstvChi = vertcat(dx, dv);
    %kinEnergy = 0.5*nu.'*M*nu;
end
