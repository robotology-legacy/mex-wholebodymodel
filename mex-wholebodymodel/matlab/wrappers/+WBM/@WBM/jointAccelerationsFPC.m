function [ddq_j, acc_data] = jointAccelerationsFPC(obj, varargin)
    switch nargin
        case 9 % optimized mode:
            M         = varargin{1,1};
            c_qv      = varargin{1,2};
            dq_j      = varargin{1,3};
            nu        = varargin{1,4};
            tau       = varargin{1,5};
            Jc        = varargin{1,6};
            djcdq     = varargin{1,7};
            feet_conf = varargin{1,8};

            % get the initial transformation vectors (VE-Transformations*) of the feet:
            % *) veT - position vector with Euler angles (in this case it represents a
            %    joint motion m(t) = (p(t), e(t))^T, where p(t) in R^3 and e(t) in S^3).

            % fk_init_veT.l_foot = feet_conf.veT_init.l_sole; % reference motions
            % fk_init_veT.r_foot = feet_conf.veT_init.r_sole;
            fk_ref_pose.veT_lfoot = feet_conf.pose.veT_lfoot; % reference motions
            fk_ref_pose.veT_rfoot = feet_conf.pose.veT_rfoot;

            % calculate the new positions and orientations (VQ-Transformations) for the feet:
            fk_new_pose.vqT_lfoot = mexWholeBodyModel('forward-kinematics', 'l_sole');
            fk_new_pose.vqT_rfoot = mexWholeBodyModel('forward-kinematics', 'r_sole');
        case 8 % normal mode:
            % wf_R_b  = varargin{1}
            wf_p_b    = varargin{1,2};
            q_j       = varargin{1,3};
            dq_j      = varargin{1,4};
            v_b       = varargin{1,5};
            tau       = varargin{1,6};
            feet_conf = varargin{1,7};
            nu        = vertcat(v_b, dq_j); % mixed generalized velocity

            % initial transformation vectors of the feet:
            fk_ref_pose.veT_lfoot = feet_conf.pose.veT_lfoot;
            fk_ref_pose.veT_rfoot = feet_conf.pose.veT_rfoot;

            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            M    = mexWholeBodyModel('mass-matrix', wf_R_b_arr, wf_p_b, q_j);
            c_qv = mexWholeBodyModel('generalized-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);

            % compute for each contact constraint the Jacobian and the derivative Jacobian:
            [Jc, djcdq] = contactJacobians(obj, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);

            % get the new VQ-Transformations for the feet:
            fk_new_pose.vqT_lfoot = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, wf_p_b, q_j, 'l_sole');
            fk_new_pose.vqT_rfoot = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, wf_p_b, q_j, 'r_sole');
        case 5 % optimized mode:
            dq_j      = varargin{1,1};
            v_b       = varargin{1,2};
            tau       = varargin{1,3};
            feet_conf = varargin{1,4};
            nu        = vertcat(v_b, dq_j);

            fk_ref_pose.veT_lfoot = feet_conf.pose.veT_lfoot;
            fk_ref_pose.veT_rfoot = feet_conf.pose.veT_rfoot;

            M    = mexWholeBodyModel('mass-matrix');
            c_qv = mexWholeBodyModel('generalized-forces');

            [Jc, djcdq] = contactJacobians(obj);

            fk_new_pose.vqT_lfoot = mexWholeBodyModel('forward-kinematics', 'l_sole');
            fk_new_pose.vqT_rfoot = mexWholeBodyModel('forward-kinematics', 'r_sole');
        otherwise
            error('WBM::jointAccelerationsExt: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
    k_p = feet_conf.ctrl_gains.k_p; % control gain for correcting the feet positions (position feedback).
    k_v = feet_conf.ctrl_gains.k_v; % control gain for correcting the velocities (rate feedback).

    % get the vector Euler-angle transformations (veT) from the feet frames:
    [p_lf, eul_lf] = WBM.utilities.frame2posEul(fk_new_pose.vqT_lfoot);
    [p_rf, eul_rf] = WBM.utilities.frame2posEul(fk_new_pose.vqT_rfoot);
    fk_new_pose.veT_lfoot = vertcat(p_lf, eul_lf); % current motions
    fk_new_pose.veT_rfoot = vertcat(p_rf, eul_rf);

    % check which foot is in contact with the ground and calculate the corresponding error
    % between the reference (desired) and the new feet-transformations:
    if (feet_conf.ground.left && feet_conf.ground.right)
        % both feet have contact with the ground:
        % get the Euler angle velocity transformation of each foot:
        Er_lf = WBM.utilities.eul2angVelTF(eul_lf);
        Er_rf = WBM.utilities.eul2angVelTF(eul_rf);
        % create for each foot the mixed velocity transformation matrix:
        vX_lf = WBM.utilities.mixveltfm(Er_lf);
        vX_rf = WBM.utilities.mixveltfm(Er_rf);

        % delta_f  =  vX * (current transf. T   -   desired transf. T*)
        %                    (curr. motion)           (ref. motion)
        delta_lf = vX_lf*(fk_new_pose.veT_lfoot - fk_ref_pose.veT_lfoot);
        delta_rf = vX_rf*(fk_new_pose.veT_rfoot - fk_ref_pose.veT_rfoot);

        error_feet = vertcat(delta_lf, delta_rf);
    elseif (~feet_conf.ground.left && feet_conf.ground.right)
        % only the right foot is in contact with the ground:
        % create the mixed velocity transformation of the right foot:
        Er_rf = WBM.utilities.eul2angVelTF(eul_rf);
        vX_rf = WBM.utilities.mixveltfm(Er_rf);

        error_feet = vX_rf*(fk_new_pose.veT_rfoot - fk_ref_pose.veT_rfoot);
    elseif (feet_conf.ground.left && ~feet_conf.ground.right)
        % only the left foot is in contact with the ground:
        % create the mixed velocity transformation of the left foot:
        Er_lf = WBM.utilities.eul2angVelTF(eul_lf);
        vX_lf = WBM.utilities.mixveltfm(Er_lf);

        error_feet = vX_lf*(fk_new_pose.veT_lfoot - fk_ref_pose.veT_lfoot);
    else
        % both feet have no contact to the ground. Either the robot is
        % lifted into the air, or is jumping (or is flying ;-) ) ...
        error_feet = 0;
    end

    % Calculation of the contact force vector for a closed-loop control system with additional
    % velocity and position correction for the feet (position-regulation system):
    % Further details about the basic formula see,
    %   [1] Control Strategies for Robots in Contact, J. Park, PhD-Thesis, Artificial Intelligence Laboratory,
    %       Department of Computer Science, Stanford University, 2006, chapter 5, pp. 106-110, eq. (5.5)-(5.14),
    %       <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>.
    %   [2] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 269-270, eq. (6.5) & (6.6).
    Jc_t      = Jc.';
    JcMinv    = Jc / M; % x*M = Jc --> x = Jc*M^(-1)
    Upsilon_c = JcMinv * Jc_t; % inverse mass matrix in contact space Upsilon_c = (Jc * M^(-1) * Jc^T) ... (= inverse "pseudo-kinetic energy matrix"?)
    tau_fr    = frictionForces(obj, dq_j); % friction torques (negative torque values)
    tau_gen   = vertcat(zeros(6,1), tau + tau_fr); % generalized forces tau_gen = S_j*(tau + (-tau_fr)), S_j ... joint selection matrix

    % contact (constraint) forces ...
    f_c = -(Upsilon_c \ (JcMinv*(c_qv - tau_gen) - djcdq - k_v.*(Jc*nu) - k_p.*error_feet));

    % Joint Acceleration q_ddot (derived from the state-space equation):
    % For further details see:
    %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
    ddq_j = M \ (tau_gen - c_qv - Jc_t*f_c); % ddq_j = M^(-1) * (...)

    acc_data = struct('f_c', f_c, 'tau', tau, 'tau_gen', tau_gen);
end
