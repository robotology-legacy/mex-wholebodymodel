function [dstvChi, h] = forwardDynamics(obj, t, stvChi, ctrlTrqs)
    ndof = obj.mwbm_config.ndof;
    nCstrs = obj.mwbm_config.nCstrs;
    dampCoeff = obj.mwbm_config.dampCoeff;

    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.fastGetStateParams(stvChi, obj.mwbm_config.stvLen, ndof);

    omega_w = stp.omega_b;
    %v_bw = [stp.dx_b; omega_w]; % slow ...
    v_bw = vertcat(stp.dx_b, omega_w); % faster ...
    %v = [stp.dx_b; omega_w; stp.dq_j];

    % mex-WBM calls:
    %
    obj.setState(stp.q_j, stp.dq_j, v_bw);

    % reconstruct the rotation of the 'root link' to the 'world'
    % from the quaternion part of the transformation vector vqT_b:
    vqT_b = obj.stvqT;
    [~,R_b] = WBM.utilities.frame2posRotm(vqT_b);

    M = obj.massMatrix();
    h = obj.generalizedBiasForces();

    % compute for each contact constraint the Jacobian and the corresponding
    % derivative Jacobian:
    m = 6*nCstrs;
    n = 6 + ndof;
    Jc = zeros(m,n);
    dJcDq = zeros(m,1);
    for i = 1:nCstrs
        Jc(6*i-5:6*i,1:n)  = obj.jacobian(obj.mwbm_config.cstrLinkNames{i}); % 6*(i-1)+1 == 6*i-5
        dJcDq(6*i-5:6*i,1) = obj.dJdq(obj.mwbm_config.cstrLinkNames{i});
    end

    % get the current control torque vector ...
    tau = ctrlTrqs.tau(t);

    % contact force computations:
    JcMinv = Jc/M;
    JcMinvJct = JcMinv * Jc';
    tauDamp = -dampCoeff * stp.dq_j;
    % calculate the contact (constraint) force ...
    f_c = JcMinvJct \ (JcMinv * (h - vertcat(zeros(6,1), tau+tauDamp)) - dJcDq);

    % need to apply root-to-world rotation to the spatial angular velocity omega_w to
    % obtain angular velocity in body frame omega_b. This is then used in the
    % quaternion derivative computation:
    omega_b = R_b * omega_w;
    dqt_b = WBM.utilities.quatDerivative(stp.qt_b, omega_b);

    %dx = [stp.dx_b; dqt_b; stp.dq_j]; % slow ...
    dx = vertcat(stp.dx_b, dqt_b, stp.dq_j); % faster ...
    dv = M \ (Jc'*f_c + vertcat(zeros(6,1), tau+tauDamp) - h);
    dstvChi = vertcat(dx, dv);
    %kinEnergy = 0.5*v'*M*v;
end
