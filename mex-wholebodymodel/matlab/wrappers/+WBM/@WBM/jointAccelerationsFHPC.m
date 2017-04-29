function [ddq_j, fd_prms] = jointAccelerationsFHPC(obj, feet_conf, hand_conf, tau, fe_h, varargin)
    switch nargin
        case 11 % normal modes:
            ac_f   = varargin{1,1};
            wf_R_b = varargin{1,2};
            wf_p_b = varargin{1,3};
            q_j    = varargin{1,4};
            dq_j   = varargin{1,5};
            v_b    = varargin{1,6};
        case 10
            ac_f   = zeroCtcAcc(obj, feet_conf);
            wf_R_b = varargin{1,1};
            wf_p_b = varargin{1,2};
            q_j    = varargin{1,3};
            dq_j   = varargin{1,4};
            v_b    = varargin{1,5};
        case 8 % optimized modes:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};
            v_b  = varargin{1,3};
        case 7
            ac_f = zeroCtcAcc(obj, feet_conf);
            dq_j = varargin{1,1};
            v_b  = varargin{1,2};
        otherwise
            error('WBM::jointAccelerationsFHPC: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
    hand_idx_list = horzcat(hand_conf.lnk_idx_l, hand_conf.lnk_idx_r);

    if (nargin > 8)
        % normal mode:
        wf_R_b_arr = reshape(wf_R_b, 9, 1);
        [M, c_qv, Jc_f, djcdq_f] = rigidBodyDynCJacobiansCS(obj, feet_conf, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
        [Jc_h, djcdq_h] = contactJacobians(obj, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b, hand_idx_list);
    else
        % optimized mode:
        [M, c_qv, Jc_f, djcdq_f] = rigidBodyDynCJacobiansCS(obj, feet_conf);
        [Jc_h, djcdq_h] = contactJacobians(obj, hand_idx_list);
    end

    Jcf_t = Jc_f.';
    nu = vertcat(v_b, dq_j); % mixed generalized velocity
    % calculate the feet contact forces with the corresponding generalized forces ...
    fe_0 = zeroExtForces(obj, feet_conf);
    [fc_f, tau_gen] = contactForcesCLPC(obj, feet_conf, tau, fe_0, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j, nu); % optimized mode
    % calculate the joint accelerations with the feet contact constraints ...
    ddqj_f = M \ (tau_gen + Jcf_t*fc_f - c_qv);

    % get the accelerations of the hands ...
    ac_h = Jc_h*ddqj_f + djcdq_h;
    % calculate the contact forces of the hands ...
    [fc_h,~] = contactForcesCLPC(obj, hand_conf, tau, fe_h, ac_h, Jc_h, djcdq_h, M, c_qv, dq_j, nu); % optimized mode

    % calculate the total joint acceleration vector ddq_j in dependency
    % of the contact forces of the feet and hands:
    J_c  = vertcat(Jc_f, Jc_h);
    f_c  = vertcat(fc_f, fc_h);
    Jc_t = J_c.';
    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv);

    % build the data structure for the forward dynamics parameters ...
    f_e = vertcat(fe_0, fe_h);
    a_c = vertcat(ac_f, ac_h);
    fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'f_e', f_e, 'a_c', a_c);
end
