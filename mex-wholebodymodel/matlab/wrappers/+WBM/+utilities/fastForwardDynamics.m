function dstvChi = fastForwardDynamics(t, stvChi, fhTrqControl, robot_model, robot_config)
    ndof         = robot_model.ndof;
    nCstrs       = robot_config.nCstrs;
    vfrict_coeff = robot_model.frict_coeff.v;
    cfrict_coeff = robot_model.frict_coeff.c;

    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.fastGetStateParams(stvChi, robot_config.stvLen, ndof);

    omega_w = stp.omega_b;
    v_b = vertcat(stp.dx_b, omega_w); % generalized base velocity
    %v = vertcat(v_b, stp.dq_j);

    % update the state for the optimized mode (precautionary) ...
    wbm_updateState(stp.q_j, stp.dq_j, v_b);

    % reconstruct the rotation of the 'root link' to the 'world'
    % from the quaternion part of the transformation vector vqT_b:
    [~,vqT_b,~,~] = wbm_getState();
    [~,R_b] = WBM.utilities.frame2posRotm(vqT_b);

    M    = wbm_massMatrix();
    C_qv = wbm_generalisedBiasForces();

    % compute for each contact constraint the Jacobian and the derivative Jacobian:
    m = 6*nCstrs;
    n = 6 + ndof;
    Jc = zeros(m,n);
    dJcdq = zeros(m,1);
    for i = 1:nCstrs
        cstr_link = robot_config.cstr_link_names{1,i};
        Jc(6*i-5:6*i,1:n)  = wbm_jacobian(cstr_link);
        dJcdq(6*i-5:6*i,1) = wbm_djdq(cstr_link);
    end

    % get the current control torques ...
    [tau,~] = fhTrqControl(t);

    % Computation of the contact (constraint) force vector:
    % Further details about the formula see,
    %   [1] Control Strategies for Robots in Contact, J. Park, PhD-Thesis, Artificial Intelligence Laboratory,
    %       Department of Computer Science, Stanford University, 2006, chapter 5, pp. 106-110, eq. (5.5)-(5.14),
    %       <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>.
    %   [2] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 269-270, eq. (6.5) & (6.6).
    Jc_t      =  Jc.';
    JcMinv    =  Jc / M;
    JcMinvJct =  JcMinv * Jc_t; % inverse mass matrix in contact space
    tau_fr    =  calcFrictionForces(stp.dq_j, vfrict_coeff, cfrict_coeff); % friction torques (negative values)
    tau_gen   =  vertcat(zeros(6,1), tau + tau_fr); % generalized force tau_gen = tau + (-tau_fr)
    % calculate the contact forces ...
    f_c = JcMinvJct \ (JcMinv*(C_qv - tau_gen) - dJcdq);

    % We need to apply the base-to-world rotation to the spatial angular velocity
    % omega_w to obtain the angular velocity in body frame omega_b. This is then
    % used in the quaternion derivative computation:
    omega_b = R_b * omega_w;
    dqt_b   = WBM.utilities.dQuat(stp.qt_b, omega_b);

    % velocities:
    dx = vertcat(stp.dx_b, dqt_b, stp.dq_j);
    % Joint Acceleration q_ddot (derived from the state-space equation):
    % For further details see:
    %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
    dv = M \ (Jc_t*f_c + tau_gen - C_qv);
    %dv = M \ (Jc.'*f_c + tau_gen - C_qv); % cause Jc.'*f_c round-off errors?

    dstvChi = vertcat(dx, dv);
    %kinEnergy = 0.5*v.'*M*v;
end
%% END of fastForwardDynamics.


function tau_fr = calcFrictionForces(dq_j, vfrict_coeff, cfrict_coeff)
    epsilon = 1e-12; % min. value to treat a number as zero ...

    if (sum(dq_j ~= 0) <= epsilon) % if dq_j = 0:
        tau_fr = zeros(size(dq_j,1),1);
        return
    end
    tau_vf = -vfrict_coeff .* dq_j;       % viscous friction torques
    tau_cf = -cfrict_coeff .* sign(dq_j); % Coulomb friction torques
    tau_fr =  tau_vf + tau_cf;            % friction torques
end
