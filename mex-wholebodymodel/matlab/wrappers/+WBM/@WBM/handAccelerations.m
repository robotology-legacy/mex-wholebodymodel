function [ac_h, a_prms] = handAccelerations(obj, feet_conf, hand_conf, tau, varargin)
    ctc_l = hand_conf.contact.left;
    ctc_r = hand_conf.contact.right;

    % check the contact state (CS) of the hands:
    if (ctc_l && ctc_r)
        % both hands:
        hand_idx_list = horzcat(hand_conf.lnk_idx_l, hand_conf.lnk_idx_r);
    elseif ctc_l
        % only left hand:
        hand_idx_list = hand_conf.lnk_idx_l;
    elseif ctc_r
        % only right hand:
        hand_idx_list = hand_conf.lnk_idx_r;
    else
        % no contacts:
        ac_h = obj.ZERO_CVEC_12;
        if (nargout == 2)
            a_prms = struct(); % empty structure ...
        end
        return
    end

    f_data = true;
    switch nargin
        case 11
            if iscolumn(varargin{1,4})
                % normal mode:
                ac_f   = varargin{1,1};
                wf_R_b = varargin{1,2};
                wf_p_b = varargin{1,3};
                q_j    = varargin{1,4};
                dq_j   = varargin{1,5};
                v_b    = varargin{1,6};
                nu     = varargin{1,7}; % mixed generalized velocity

                wf_R_b_arr = reshape(wf_R_b, 9, 1);
                [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, feet_conf, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                [Jc_h, djcdq_h] = contactJacobians(obj, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b, hand_idx_list);
            else
                % optimized mode:
                ac_f    = varargin{1,1};
                Jc_f    = varargin{1,2};
                djcdq_f = varargin{1,3};
                M       = varargin{1,4};
                c_qv    = varargin{1,5};
                dq_j    = varargin{1,6};
                nu      = varargin{1,7};

                [Jc_h, djcdq_h] = contactJacobians(obj, hand_idx_list);
                f_data = false;
            end
        case 7
            % optimized mode:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};
            nu   = varargin{1,3};

            [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, feet_conf);
            [Jc_h, djcdq_h] = contactJacobians(obj, hand_idx_list);
        otherwise
            error('WBM::handAccelerations: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    Jcf_t = Jc_f.';
    % compute the feet contact forces with the corresponding generalized forces ...
    fe_0 = zeroExtForces(obj, feet_conf);
    [fc_f, tau_gen] = contactForcesCLPC(obj, feet_conf, tau, fe_0, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j, nu); % optimized mode
    % calculate the joint accelerations with the feet contact constraints ...
    ddqj_f = M \ (tau_gen + Jcf_t*fc_f - c_qv);

    % calculate the mixed acceleration of the hand(s) at the contact link(s):
    ac_h = Jc_h*ddqj_f + djcdq_h;

    if (nargout == 2)
        % data structure of the calculated acceleration parameters:
        if f_data
            % with feet data Jc_f, djcdq_f, M and c_qv ...
            a_prms = struct('M', M, 'c_qv', c_qv, 'Jc_f', Jc_f, 'Jc_h', Jc_h, 'djcdq_f', djcdq_f, ...
                            'djcdq_h', djcdq_h, 'fc_f', fc_f, 'tau_gen', tau_gen);
            return
        end
        % else, without Jc_f, djcdq_f, M and c_qv (existing outside) ...
        a_prms = struct('Jc_h', Jc_h, 'djcdq_h', djcdq_h, 'fc_f', fc_f, 'tau_gen', tau_gen);
    end
end
