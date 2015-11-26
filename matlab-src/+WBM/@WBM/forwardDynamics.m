function [dchi , h] = forwardDynamics(obj, t, chi, ctrlTrqs)
    ndof = obj.wbm_config.ndof;
    nCstrs = obj.wbm_config.nCstrs;
    dampCoeff = obj.wbm_config.dampCoeff;

    % get the current state parameters from the run-time variable "chi" ...
    st = obj.getStateParams(chi);
    omega_w = st.omega_b;
    v_bw = [st.dx_b; omega_w];
    %v = [st.dx_b; omega_w; st.dq_j];

    % mex-WBM calls:
    %
    obj.setState(st.q_j, st.dq_j, v_bw);

    % reconstruct the rotation of the 'root link' to the 'world'
    % from the quaternion part of the transformation vector T_b:
    [T_b,~,~,~] = obj.getState();
    [~,R_b] = frame2posRotm(T_b);

    M = obj.massMatrix();
    h = obj.generalBiasForces();

    % compute the Jacobian and the corresponding derivative Jacobian for
    % each contact constraint:
    Jc = zeros(6*nCstrs,6+ndof);
    dJcDq = zeros(6*nCstrs,1);
    for i = 1:nCstrs
        Jc(6*i-5:6*i,:) = obj.jacobian(obj.wbm_config.cstrLinkNames{i}); % 6*(i-1)+1 == 6*i-5
        dJcDq(6*i-5:6*i,:) = obj.dJdq(obj.wbm_config.cstrLinkNames{i});
    end

    % get the current control torque ...
    tau = ctrlTrqs.tau(t);

    % contact force computations:
    %
    JcMinv = Jc/M;
    JcMinvJct = JcMinv * Jc';
    tauDamp = -dampCoeff * st.dq_j;
    
    % calculate the contact (constraint) force ...
    f_c = JcMinvJct \ (JcMinv * (h - [zeros(6,1); (tau + tauDamp)]) - dJcDq);

    % need to apply root-to-world rotation to the spatial angular velocity omega_w to
    % obtain angular velocity in body frame omega_b. This is then used in the
    % quaternion derivative computation:
    omega_b = R_b * omega_w;
    dqt_b = quatDerivative(st.qt_b, omega_b);

    dx = [st.dx_b; dqt_b; st.dq_j];
    dv = M \ (Jc'*f_c + [zeros(6,1); (tau + tauDamp)] - h);
    dchi = [dx; dv];
    %kinEnergy = 0.5*v'*M*v;
end
