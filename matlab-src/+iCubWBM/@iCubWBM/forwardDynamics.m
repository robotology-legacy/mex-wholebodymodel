function [dchi , h] = forwardDynamics(t, chi)
    ndof = obj.wb_config.ndof;
    nCstrs = obj.wb_config.nCstrs;

    x_b  = chi(1:3, :);
    qt_b = chi(4:7, :);
    q_j  = chi(8:ndof+7, :);

    dx_b    = chi(ndof+8:ndof+10, :);
    omega_W = chi(ndof+11:ndof+13, :);
    dq_j    = chi(ndof+14:2*ndof+13, :);

    v = [dx_b; omega_W; dq_j];

    %% MexWholeBodyModel calls
    setState(q_j, dq_j, [dx_b; omega_W]);

    %reconstructing rotation of root to world from the quaternion
    [~,T_b,~,~] = getState();

    qt_b_mod_s = T_b(4);
    qt_b_mod_r = T_b(5:end);
    R_b = eye(3) - 2*qt_b_mod_s*skew(qt_b_mod_r) + 2 * skew(qt_b_mod_r)^2;
    p_b = T_b(1:3);

    M = massMatrix();
    h = genBiasForces();

    hDash = genBiasForces(R_b, p_b, q_j, dq_j, [dx_b; omega_W]);
    g = genBiasForces(R_b, p_b, q_j, zeros(size(q_j)), zeros(6,1));

    %% Building up contraints jacobian and djdq
    Jc = zeros(6*nCstrs, 6 + ndof);
    dJcDq = zeros(6*nCstrs, 1);
    for i = 1:nCstrs
        Jc(6*(i-1)+1:6*i, :) = jacobian(obj.wb_config.cstrLinkNames{i});
        dJcDq(6*(i-1)+1:6*i, :) = dJdq(obj.wb_config.cstrLinkNames{i});
    end

    %% control torque
    tau = obj.wb_config.tau(t);

    %% Contact forces computation
    JcMinv = Jc/M;
    JcMinvJct = JcMinv * Jc';   

    tauDamp = -obj.wb_config.dampCoeff*dq_j;

    temp = JcMinv*h;
    temp2 = JcMinvJct\(JcMinv*h);

    fc = (JcMinvJct)\(JcMinv*(h - [zeros(6,1); tau + tauDamp]) - dJcDq);

    % need to apply root-to-world rotation to the spatial angular velocity omega_W to
    % obtain angular velocity in body frame omega_b. This is then used in the
    % quaternion derivative computation.
    omega_b = R_b*omega_W;
    dqt_b = quaternionDeriv(qt_b, omega_b);

    dx = [dx_b; dqt_b; dq_j];
    dv = M\(Jc'*fc + [zeros(6,1); tau + tauDamp] - h);
    dchi = [dx; dv];  
end
