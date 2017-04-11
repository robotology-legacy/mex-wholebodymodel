function [ddq_j, fd_prms] = jointAccelerationsCLPC(obj, clink_conf, tau, f_e, a_c, varargin)
    switch nargin
        case 11 % optimized mode:
            Jc    = varargin{1,1};
            djcdq = varargin{1,2};
            M     = varargin{1,3};
            c_qv  = varargin{1,4};
            dq_j  = varargin{1,5};
            nu    = varargin{1,6};
            % a_c  ... contact accelerations
            % f_e  ... external forces affecting on the contact links

            % get the desired poses (VE-Transformations*) for the contact links as reference:
            % *) veT - position vector with Euler angles (in this case it represents a
            %    joint motion m(t) = (p(t), e(t))^T, where p(t) in R^3 and e(t) in S^3).
            fk_ref_pose.veT_llnk = clink_conf.des_pose.veT_llnk; % reference motions
            fk_ref_pose.veT_rlnk = clink_conf.des_pose.veT_rlnk;

            % calculate the new positions and orientations (VQ-Transformations) for the contact links:
            fk_new_pose.vqT_llnk = mexWholeBodyModel('forward-kinematics', clink_conf.lnk_l);
            fk_new_pose.vqT_rlnk = mexWholeBodyModel('forward-kinematics', clink_conf.lnk_r);
        case 10 % normal mode:
            % wf_R_b  = varargin{1}
            wf_p_b = varargin{1,2};
            q_j    = varargin{1,3};
            dq_j   = varargin{1,4};
            v_b    = varargin{1,5};
            nu     = vertcat(v_b, dq_j); % mixed generalized velocity

            % reference transformations for the contact links:
            fk_ref_pose.veT_llnk = clink_conf.des_pose.veT_llnk;
            fk_ref_pose.veT_rlnk = clink_conf.des_pose.veT_rlnk;

            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            M    = mexWholeBodyModel('mass-matrix', wf_R_b_arr, wf_p_b, q_j);
            c_qv = mexWholeBodyModel('generalized-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);

            % compute for each contact constraint the Jacobian and the derivative Jacobian:
            [Jc, djcdq] = contactJacobians(obj, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);

            % get the new VQ-Transformations for the contact links:
            fk_new_pose.vqT_llnk = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, wf_p_b, q_j, clink_conf.lnk_l);
            fk_new_pose.vqT_rlnk = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, wf_p_b, q_j, clink_conf.lnk_r);
        case 7 % optimized mode:
            dq_j = varargin{1,1};
            v_b  = varargin{1,2};
            nu   = vertcat(v_b, dq_j);

            fk_ref_pose.veT_llnk = clink_conf.des_pose.veT_llnk;
            fk_ref_pose.veT_rlnk = clink_conf.des_pose.veT_rlnk;

            M    = mexWholeBodyModel('mass-matrix');
            c_qv = mexWholeBodyModel('generalized-forces');

            [Jc, djcdq] = contactJacobians(obj);

            fk_new_pose.vqT_llnk = mexWholeBodyModel('forward-kinematics', clink_conf.lnk_l);
            fk_new_pose.vqT_rlnk = mexWholeBodyModel('forward-kinematics', clink_conf.lnk_r);
        otherwise
            error('WBM::jointAccelerationsExt: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
    k_p = clink_conf.ctrl_gains.k_p; % control gain for correcting the link positions (position feedback).
    k_v = clink_conf.ctrl_gains.k_v; % control gain for correcting the velocities (rate feedback).

    % get the vector Euler-angle transformations (veT) from the link frames:
    [p_ll, eul_ll] = WBM.utilities.frame2posEul(fk_new_pose.vqT_llnk);
    [p_rl, eul_rl] = WBM.utilities.frame2posEul(fk_new_pose.vqT_rlnk);
    fk_new_pose.veT_llnk = vertcat(p_ll, eul_ll); % current motions
    fk_new_pose.veT_rlnk = vertcat(p_rl, eul_rl);

    % check which link is in contact with the ground/object and calculate the corresponding
    % error between the reference (desired) and the new link transformations:
    if (clink_conf.contact.left && clink_conf.contact.right)
        % both links are in contact with the ground/object:
        % get the Euler angle velocity transformation of each contact link:
        Er_ll = WBM.utilities.eul2angVelTF(eul_ll);
        Er_rl = WBM.utilities.eul2angVelTF(eul_rl);
        % create for each link the mixed velocity transformation matrix:
        vX_ll = WBM.utilities.mixveltfm(Er_ll);
        vX_rl = WBM.utilities.mixveltfm(Er_rl);

        % delta  =   vX * (current transf. T   -   desired transf. T*)
        %                   (curr. motion)           (ref. motion)
        delta_ll = vX_ll*(fk_new_pose.veT_llnk - fk_ref_pose.veT_llnk);
        delta_rl = vX_rl*(fk_new_pose.veT_rlnk - fk_ref_pose.veT_rlnk);

        error_lnk = vertcat(delta_ll, delta_rl);
    elseif (~clink_conf.contact.left && clink_conf.contact.right)
        % only the right link is in contact with the ground/object:
        % create the mixed velocity transformation of the right contact link:
        Er_rl = WBM.utilities.eul2angVelTF(eul_rl);
        vX_rl = WBM.utilities.mixveltfm(Er_rl);

        error_lnk = vX_rl*(fk_new_pose.veT_rlnk - fk_ref_pose.veT_rlnk);
    elseif (clink_conf.contact.left && ~clink_conf.contact.right)
        % only the left link is in contact with the ground/object:
        % create the mixed velocity transformation of the left contact link:
        Er_ll = WBM.utilities.eul2angVelTF(eul_ll);
        vX_ll = WBM.utilities.mixveltfm(Er_ll);

        error_lnk = vX_ll*(fk_new_pose.veT_llnk - fk_ref_pose.veT_llnk);
    else
        % both links are not in contact with the ground or object:
        error_lnk = 0;
    end

    % Calculation of the contact force vector for a closed-loop control system with additional
    % velocity and position correction for the contact links (position-regulation system):
    % For further details about the basic formula see,
    %   [1] Control Strategies for Robots in Contact, J. Park, PhD-Thesis, Artificial Intelligence Laboratory,
    %       Department of Computer Science, Stanford University, 2006, chapter 5, pp. 106-110, eq. (5.5)-(5.14),
    %       <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>.
    %   [2] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 269-270, eq. (6.5) & (6.6).
    Jc_t      = Jc.';
    JcMinv    = Jc / M; % x*M = Jc --> x = Jc*M^(-1)
    Upsilon_c = JcMinv * Jc_t; % inverse mass matrix in contact space Upsilon_c = (Jc * M^(-1) * Jc^T) ... (= inverse "pseudo-kinetic energy matrix"?)
    tau_fr    = frictionForces(obj, dq_j); % friction torques (negated torque values)
    tau_gen   = vertcat(zeros(6,1), tau + tau_fr); % generalized forces tau_gen = S_j*(tau + (-tau_fr)), S_j ... joint selection matrix

    % contact constraint forces (generated by the environment) ...
    f_c = (Upsilon_c \ (a_c + JcMinv*(c_qv - tau_gen) - djcdq - k_v.*(Jc*nu) - k_p.*error_lnk)) - f_e;

    % Joint Acceleration q_ddot (derived from the state-space equation):
    % For further details see:
    %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv); % ddq_j = M^(-1) * (...)

    fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'f_e', f_e, 'a_c', a_c);
end
