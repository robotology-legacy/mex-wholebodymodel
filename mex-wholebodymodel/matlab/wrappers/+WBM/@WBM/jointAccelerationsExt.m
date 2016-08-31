function [ddq_j, acc_data] = jointAccelerationsExt(obj, varargin)
    switch nargin
        case 9 % optimized mode:
            M         = varargin{1,1};
            C_qv      = varargin{1,2};
            dq_j      = varargin{1,3};
            nu        = varargin{1,4};
            tau       = varargin{1,5};
            Jc        = varargin{1,6};
            dJcdq     = varargin{1,7};
            foot_conf = varargin{1,8};

            % get the initial transformation vectors (VE-Transformations*) of the feet:
            % *) veT - position vector with Euler angles (in this case it represents a
            %    joint motion m(t) = (p(t), e(t))^T, where p(t) in R^3 and e(t) in S^3).
            fk_init_veT.l_foot = foot_conf.veT_init.l_sole; % reference motions
            fk_init_veT.r_foot = foot_conf.veT_init.r_sole;

            % calculate the new positions and orientations (VQ-Transformations) for the feet:
            fk_new_vqT.l_sole = mexWholeBodyModel('forward-kinematics', 'l_sole');
            fk_new_vqT.r_sole = mexWholeBodyModel('forward-kinematics', 'r_sole');
        case 8 % normal mode:
            % wf_R_b  = varargin{1}
            wf_p_b    = varargin{1,2};
            q_j       = varargin{1,3};
            dq_j      = varargin{1,4};
            v_b       = varargin{1,5};
            tau       = varargin{1,6};
            foot_conf = varargin{1,7};
            nu        = vertcat(v_b, dq_j); % mixed generalized velocity

            % initial transformation vectors of the feet:
            fk_init_veT.l_foot = foot_conf.veT_init.l_sole;
            fk_init_veT.r_foot = foot_conf.veT_init.r_sole;

            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            M    = mexWholeBodyModel('mass-matrix', wf_R_b_arr, wf_p_b, q_j);
            C_qv = mexWholeBodyModel('generalised-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);

            % compute for each contact constraint the Jacobian and the derivative Jacobian:
            [Jc, dJcdq] = contactJacobians(obj, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);

            % get the new VQ-Transformations for the feet:
            fk_new_vqT.l_sole = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, wf_p_b, q_j, 'l_sole');
            fk_new_vqT.r_sole = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, wf_p_b, q_j, 'r_sole');
        case 5 % optimized mode:
            dq_j      = varargin{1,1};
            v_b       = varargin{1,2};
            tau       = varargin{1,3};
            foot_conf = varargin{1,4};
            nu        = vertcat(v_b, dq_j);

            fk_init_veT.l_foot = foot_conf.veT_init.l_sole;
            fk_init_veT.r_foot = foot_conf.veT_init.r_sole;

            M    = mexWholeBodyModel('mass-matrix');
            C_qv = mexWholeBodyModel('generalised-forces');

            [Jc, dJcdq] = contactJacobians(obj);

            fk_new_vqT.l_sole = mexWholeBodyModel('forward-kinematics', 'l_sole');
            fk_new_vqT.r_sole = mexWholeBodyModel('forward-kinematics', 'r_sole');
        otherwise
            error('WBM::jointAccelerationsExt: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
    % set the correction values for the feet to avoid numerical integration errors:
    k_p = foot_conf.ctrl_gains.k_p; % control gain for correcting the feet positions (position feedback).
    k_v = foot_conf.ctrl_gains.k_v; % control gain for correcting the velocities (rate feedback).

    % get from the VQ-Transformations of the feet the translations and Euler angles:
    [p_lfoot, eul_lfoot] = WBM.utilities.frame2posEul(fk_new_vqT.l_sole);
    [p_rfoot, eul_rfoot] = WBM.utilities.frame2posEul(fk_new_vqT.r_sole);
    % create new transformation vectors (veT) for the feet:
    fk_new_veT.l_foot = vertcat(p_lfoot, eul_lfoot); % current motions
    fk_new_veT.r_foot = vertcat(p_rfoot, eul_rfoot);

    % check which foot is on the ground and calculate the error (difference) between
    % the initial and the new feet-transformations (position & orientation):
    if (foot_conf.ground.left && foot_conf.ground.right)
        % error    =           current transf.   -    desired transf.
        %                      (curr. motion)          (ref. motion)
        delta_feet = vertcat( (fk_new_veT.l_foot - fk_init_veT.l_foot), ...
                              (fk_new_veT.r_foot - fk_init_veT.r_foot) );

    elseif (~foot_conf.ground.left && foot_conf.ground.right)
        delta_feet = (fk_new_veT.r_foot - fk_init_veT.r_foot);

    elseif (foot_conf.ground.left && ~foot_conf.ground.right)
        delta_feet = (fk_new_veT.l_foot - fk_init_veT.l_foot);
    end
    % else, both feet are not fixed onto the ground. Either the robot
    % is lifted into the air or is jumping, or is flying ;-) ...

    % Calculation of the contact force vector for a closed-loop control system with additional
    % velocity and position correction for the feet (position-regulation system):
    % Further details about the basic formula see,
    %   [1] Control Strategies for Robots in Contact, J. Park, PhD-Thesis, Artificial Intelligence Laboratory,
    %       Department of Computer Science, Stanford University, 2006, chapter 5, pp. 106-110, eq. (5.5)-(5.14),
    %       <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>.
    %   [2] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 269-270, eq. (6.5) & (6.6).
    Jc_t      = Jc.';
    JcMinv    = Jc / M; % x*M = Jc --> x = Jc*M^(-1)
    JcMinvJct = JcMinv * Jc_t; % inverse mass matrix in contact space
    tau_fr    = frictionForces(obj, dq_j); % friction torques (negative torque values)
    tau_gen   = vertcat(zeros(6,1), tau + tau_fr); % generalized forces tau_gen = tau + (-tau_fr)
    % contact (constraint) forces ...
    f_c = -(JcMinvJct \ (JcMinv*(C_qv - tau_gen) - dJcdq - k_v.*(Jc*nu) - k_p.*delta_feet));

    % Joint Acceleration q_ddot (derived from the state-space equation):
    % For further details see:
    %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
    ddq_j = M \ (tau_gen - C_qv - Jc_t*f_c); % ddq_j = M^(-1) * (...)

    acc_data = struct('f_c', f_c, 'tau', tau, 'tau_gen', tau_gen);
end
