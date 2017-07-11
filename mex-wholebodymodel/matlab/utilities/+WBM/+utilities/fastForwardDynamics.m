function dstvChi = fastForwardDynamics(t, stvChi, fhTrqControl, robot_model, robot_config)
    ndof    = robot_model.ndof;
    nCstrs  = robot_config.nCstrs;
    frict_c = robot_model.frict_coeff.c;
    frict_v = robot_model.frict_coeff.v;

    % get the state parameters from the current state vector "stvChi" ...
    stp = WBM.utilities.fastGetStateParams(stvChi, robot_config.stvLen, ndof);

    omega_w = stp.omega_b;
    v_b = vertcat(stp.dx_b, omega_w); % generalized base velocity
    %nu = vertcat(v_b, stp.dq_j);

    % update the state for the optimized mode (precautionary) ...
    wbm_updateState(stp.q_j, stp.dq_j, v_b);

    % reconstruct the rotation from the 'base' to the 'world' of
    % the quaternion part of the transformation vector vqT_b:
    [vqT_b,~,~,~] = wbm_getState();
    [~,wf_R_b] = WBM.utilities.frame2posRotm(vqT_b);

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

    % Computation of the contact (constraint) force vector:
    % For further details about the formula see,
    %   [1] Control Strategies for Robots in Contact, J. Park, PhD-Thesis, Artificial Intelligence Laboratory,
    %       Department of Computer Science, Stanford University, 2006, chapter 5, pp. 106-110, eq. (5.5)-(5.14),
    %       <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>.
    %   [2] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 269-270, eq. (6.5) & (6.6).
    Jc_t      =  Jc.';
    JcMinv    =  Jc / M;
    Upsilon_c =  JcMinv * Jc_t; % inverse mass matrix in contact space Upsilon_c = (Jc * M^(-1) * Jc^T) ... (= "inverse pseudo-kinetic energy matrix"?)
    tau_fr    =  wbm_frictionForces(stp.dq_j, frict_c, frict_v); % friction torques (negated values)
    tau_gen   =  vertcat(zeros(6,1), tau + tau_fr); % generalized forces tau_gen = S_j*(tau + (-tau_fr)),
                                                    % S_j = [0_(6xn); I_(nxn)] ... joint selection matrix
    % calculate the contact forces ...
    f_c = Upsilon_c \ (JcMinv*(c_qv - tau_gen) - djcdq);

    % We need to apply the base-to-world rotation to the spatial angular velocity
    % omega_w to obtain the angular velocity in body frame omega_b. This is then
    % used in the quaternion derivative computation:
    omega_b = wf_R_b * omega_w;
    dqt_b   = WBM.utilities.dquat(stp.qt_b, omega_b);

    % new mixed generalized velocity ...
    v = vertcat(stp.dx_b, dqt_b, stp.dq_j);
    % Joint Acceleration q_ddot (derived from the state-space equation):
    % For further details see:
    %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
    dv = M \ (tau_gen + Jc_t*f_c - c_qv);
    %dv = M \ (tau_gen + Jc.'*f_c - c_qv); % cause Jc.'*f_c round-off errors?

    dstvChi = vertcat(v, dv);
    %kinEnergy = 0.5*nu.'*M*nu;
end
