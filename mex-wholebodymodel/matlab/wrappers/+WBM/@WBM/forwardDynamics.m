function dstvChi = forwardDynamics(obj, t, stvChi, fhCtrlTrqs)
    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, obj.mwbm_model.ndof);

    omega_w = stp.omega_b;
    v_b = vertcat(stp.dx_b, omega_w);
    %v = [stp.dx_b; omega_w; stp.dq_j];

    % update the state for the optimized mode (precautionary) ...
    obj.setState(stp.q_j, stp.dq_j, v_b);

    % reconstruct the rotation of the 'root link' to the 'world'
    % from the quaternion part of the transformation vector vqT_b:
    vqT_b = obj.stvqT;
    [~,R_b] = WBM.utilities.frame2posRotm(vqT_b);

    % get the current control torque vector ...
    tau = fhCtrlTrqs(t);

    % need to apply root-to-world rotation to the spatial angular velocity omega_w to
    % obtain angular velocity in body frame omega_b. This is then used in the
    % quaternion derivative computation:
    omega_b = R_b * omega_w;
    dqt_b   = WBM.utilities.dQuat(stp.qt_b, omega_b);

    % velocities:
    dx = vertcat(stp.dx_b, dqt_b, stp.dq_j);
    % joint acceleration dv:
    dv = jointAccelerations(obj, stp.dq_j, tau);

    dstvChi = vertcat(dx, dv);
    %kinEnergy = 0.5*v.'*M*v;
end
