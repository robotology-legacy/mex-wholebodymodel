function [dchi , h] = forwardDynamics(obj, t, ctrlTrqs, chi)
    %% extraction of state
    %ndof = param.ndof;

    % x_b = chi(1:3,:);
    % qt_b = chi(4:7,:);
    % qj = chi(8:ndof+7,:);
    % %x = [x_b;qt_b;qj];
    % 
    % dx_b = chi(ndof+8:ndof+10,:);
    % omega_W = chi(ndof+11:ndof+13,:);
    % dqj = chi(ndof+14:2*ndof+13,:);
    % 
    % v = [dx_b;omega_W;dqj];

    omega_w = chi.omega_b;
    v_bw = [chi.dx_b; omega_w];
    %v = [chi.dx_b; omega_w; chi.dq_j];

    %% MexWholeBodyModel calls
    %wbm_updateState(qj,dqj,[dx_b;omega_W]);
    obj.setState(chi.q_j, chi.dq_j, v_bw);

    %reconstructing rotation of root to world from the quaternion
    %[~,T_b,~,~] = wbm_getState();
    currState = obj.getState();

    % qt_b_mod_s = T_b(4);          % is exactly frame2posRot()
    % qt_b_mod_r = T_b(5:end);
    % R_b = eye(3) - 2*qt_b_mod_s*skew(qt_b_mod_r) + 2 * skew(qt_b_mod_r)^2;
    % p_b = T_b(1:3);

    %pos_b = currState.x_b;
    R_b = obj.quat2rot(currState.qt_b);

    % M = wbm_massMatrix();
    % h = wbm_generalisedBiasForces();

    M = obj.massMatrix();
    h = obj.genBiasForces();

    % hDash = wbm_generalisedBiasForces(R_b,p_b,qj,dqj,[dx_b;omega_W]);
    % g = wbm_generalisedBiasForces(R_b,p_b,qj,zeros(size(qj)),zeros(6,1));

    %hDash = obj.genBiasForces(R_b, pos_b, chi.q_j, chi.dq_j, v_bw);
    %g = genBiasedForces(R_b, pos_b, chi.q_j, zeros(size(chi.q_j)), zeros(6, 1));

    %% Building up contraints jacobian and djdq
    % Jc = zeros(6*param.numConstraints,6+ndof);
    % dJcDq = zeros(6*param.numConstraints,1);
    % for i=1:param.numConstraints
    %     Jc(6*(i-1)+1:6*i,:) = wbm_jacobian(param.constraintLinkNames{i});
    %     dJcDq(6*(i-1)+1:6*i,:) = wbm_djdq(param.constraintLinkNames{i});
    % end

    Jc = zeros(6 * obj.wb_config.nCstrs, 6 + obj.wb_config.ndof);
    dJcDq = zeros(6 * obj.wb_config.nCstrs, 1);
    for i=1:obj.wb_config.nCstrs
        Jc(6*(i-1) + 1:6*i, :) = obj.jacobian(obj.wb_config.cstrLinkNames{i});
        dJcDq(6*(i-1) + 1:6*i, :) = obj.dJdq(obj.wb_config.cstrLinkNames{i});
    end

    %% control torque
    tau = param.tau(t);

    %% Contact forces computation
    JcMinv = Jc/M;
    JcMinvJct = JcMinv * Jc'; 

    %tauDamp = -param.dampingCoeff*dqj;
    tauDamp = -obj.wb_config.dampCoeff * chi.dq_j;

    %temp = JcMinv*h;
    %temp2 = JcMinvJct\(JcMinv*h);

    % calculate the contact forces ...
    f_c = JcMinvJct\(JcMinv*(h - [zeros(6, 1); tau + tauDamp]) - dJcDq);

    % need to apply root-to-world rotation to the spatial angular velocity omega_W to
    % obtain angular velocity in body frame omega_b. This is then used in the
    % quaternion derivative computation.

    % omega_b = R_b*omega_W;% R_b*omega_W;
    % dqt_b = quaternionDerivative(omega_b, qt_b);%,param.QuaternionDerivativeParam);

    omega_b = R_b * omega_w;
    dqt_b = obj.quatDerivative(omega_b, chi.qt_b);

    %dx = [dx_b;dqt_b;dqj];
    dx = [chi.dx_b; dqt_b; chi.dq_j];
    dv = M\(Jc'*f_c + [zeros(6, 1); tau + tauDamp] - h);
    dchi = [dx; dv];
    
    %kinEnergy = 0.5*v'*M*v;
    %dchi = zeros(size(dchi));
end
