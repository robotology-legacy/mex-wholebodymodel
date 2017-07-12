function [ddq_j, fd_prms] = jointAccelerationsFHPC(obj, feet_conf, hand_conf, tau, fe_h, varargin)
    switch nargin
        case 12 % normal modes:
            % ac_f   = varargin{1}
            % wf_R_b = varargin{2}
            % wf_p_b = varargin{3}
            % q_j    = varargin{4}
            % v_b    = varargin{6}
            dq_j = varargin{1,5};
            nu   = varargin{1,7};

            % get the mixed acceleration of the hands at the contact links ...
            [ac_h, a_prms] = handAccelerations(obj, feet_conf, hand_conf, tau, varargin{1:7});
        case 11
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % v_b    = varargin{5}
            dq_j = varargin{1,4};
            nu   = varargin{1,6};

            ac_f = zeroCtcAcc(obj, feet_conf);
            [ac_h, a_prms] = handAccelerations(obj, feet_conf, hand_conf, tau, ac_f, varargin{1:6});
        case 8 % optimized modes:
            % ac_f = varargin{1}
            dq_j = varargin{1,2};
            nu   = varargin{1,3};

            [ac_h, a_prms] = handAccelerations(obj, feet_conf, hand_conf, tau, varargin{1:3});
        case 7
            dq_j = varargin{1,1};
            nu   = varargin{1,2};

            ac_f = zeroCtcAcc(obj, feet_conf);
            [ac_h, a_prms] = handAccelerations(obj, feet_conf, hand_conf, tau, ac_f, varargin{1:2});
        otherwise
            error('WBM::jointAccelerationsFHPC: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
    % compute the contact forces of the hands -- (optimized mode):
    [fc_h,~] = contactForcesCLPC(obj, hand_conf, tau, fe_h, ac_h, a_prms.Jc_h, ...
                                 a_prms.djcdq_h, a_prms.M, a_prms.c_qv, dq_j, nu);
    % calculate the total joint acceleration vector ddq_j in
    % dependency of the contact forces of the feet and hands:
    J_c  = vertcat(Jc_f, Jc_h);
    f_c  = vertcat(fc_f, fc_h);
    Jc_t = J_c.';
    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        f_e = vertcat(fe_0, fe_h);
        a_c = vertcat(ac_f, ac_h);
        fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'f_e', f_e, 'a_c', a_c);
    end
end
