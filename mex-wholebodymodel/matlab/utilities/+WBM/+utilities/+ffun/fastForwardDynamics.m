function dstvChi = fastForwardDynamics(t, stvChi, fhTrqControl, robot_model, robot_config)
    ndof    = robot_model.ndof;
    nCstrs  = robot_config.nCstrs;
    frict_c = robot_model.frict_coeff.c;
    frict_v = robot_model.frict_coeff.v;

    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.ffun.fastGetStateParams(stvChi, robot_config.stvLen, ndof);

    wf_omega_b = stp.omega_b;
    v_b  = vertcat(stp.dx_b, wf_omega_b); % generalized base velocity
    %nu_s = vertcat(v_b, stp.dq_j);        % mixed generalized velocity of the current state

    % update the state for the optimized mode (else the procedure computes nonsense) ...
    wbm_updateState(stp.q_j, stp.dq_j, v_b);

    % reconstruct the rotation from the 'base' to the 'world' of
    % the quaternion part of the transformation vector vqT_b:
    [vqT_b,~,~,~] = wbm_getState();
    [~,wf_R_b] = WBM.utilities.tfms.frame2posRotm(vqT_b);

    M    = wbm_massMatrix();
    c_qv = wbm_generalizedBiasForces();

    % compute for each contact constraint the Jacobian and the derivative Jacobian:
    m = 6*nCstrs;
    n = 6 + ndof;
    Jc = zeros(m,n);
    djcdq = zeros(m,1);
    for i = 1:nCstrs
        ccstr_link = robot_config.ccstr_link_names{1,i};
        Jc(6*i-5:6*i,1:n)  = wbm_jacobian(ccstr_link);
        djcdq(6*i-5:6*i,1) = wbm_dJdq(ccstr_link);
    end

    % get the current control torques ...
    tau = fhTrqControl(t);

    tau_fr  = wbm_frictionForces(stp.dq_j, frict_c, frict_v); % friction torques (negated values)
    tau_gen = vertcat(zeros(6,1), tau + tau_fr); % generalized forces tau_gen = S_j*(tau + (-tau_fr)),
                                                 % S_j = [0_(6xn); I_(nxn)] ... joint selection matrix.
    % Computation of the contact (constraint) force vector:
    % For further details about the formula see,
    %   [1] Control Strategies for Robots in Contact, J. Park, PhD-Thesis, Artificial Intelligence Laboratory, Stanford University, 2006,
    %       <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>, Chapter 5, pp. 106-110, eq. (5.5)-(5.14).
    %   [2] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 269-270, eq. (6.5) & (6.6).
    Jc_t      = Jc.';
    JcMinv    = Jc / M; % x*M = Jc --> x = Jc * M^(-1)
    Upsilon_c = JcMinv * Jc_t; % inverse mass matrix Upsilon_c = Lambda^(-1) = Jc * M^(-1) * Jc^T
                               % in contact space {c} (Lambda^(-1) ... inverse pseudo-kinetic energy matrix).
    % calculate the contact forces ...
    f_c = Upsilon_c \ (JcMinv*(c_qv - tau_gen) - djcdq);
    % (this calculation method is numerically more accurate and robust than the calculation variant with the cartmass-function.)

    % We need to apply the world-to-base rotation b_R_wf to the spatial angular
    % velocity wf_omega_b to obtain the angular velocity b_omega_wf in the base
    % body frame. This is then used in the quaternion derivative computation:
    b_R_wf = wf_R_b.';
    b_omega_wf = b_R_wf * wf_omega_b;
    dqt_b      = WBM.utilities.tfms.dquat(stp.qt_b, b_omega_wf);

    % new mixed generalized velocity ...
    nu = vertcat(stp.dx_b, dqt_b, stp.dq_j);
    % Joint Acceleration nu_dot = q_ddot (derived from the state-space equation):
    % For further details see:
    %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
    dnu = M \ (tau_gen + Jc_t*f_c - c_qv);
    %dnu = M \ (tau_gen + Jc.'*f_c - c_qv); % cause Jc.'*f_c round-off errors?

    dstvChi = vertcat(nu, dnu);
    %kinEnergy = 0.5*nu_s.'*M*nu_s;
end
