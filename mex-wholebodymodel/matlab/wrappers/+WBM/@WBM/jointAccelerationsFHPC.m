function [ddq_j, fd_prms] = jointAccelerationsFHPC(obj, feet_conf, hand_conf, tau, fe_h, varargin)
    switch nargin
        case 12
            % get the mixed acceleration of the hands at the contact links ...
            [ac_h, a_prms] = handAccelerations(obj, feet_conf, hand_conf, tau, varargin{1:7});

            if iscolumn(varargin{1,4})
                % normal mode:
                % wf_R_b = varargin{2}
                % wf_p_b = varargin{3}
                % q_j    = varargin{4}
                % v_b    = varargin{6}
                ac_f = varargin{1,1};
                dq_j = varargin{1,5};

                Jc_f = a_prms.Jc_f;
                M    = a_prms.M;
                c_qv = a_prms.c_qv;
            else
                % optimized mode:
                % djcdq_f = varargin{3}
                % M       = varargin{4}
                % c_qv    = varargin{5}
                ac_f = varargin{1,1};
                Jc_f = varargin{1,2};
                dq_j = varargin{1,6};
            end
            nu = varargin{1,7};
        case 11
            ac_f = zeroCtcAcc(obj, feet_conf);
            [ac_h, a_prms] = handAccelerations(obj, feet_conf, hand_conf, tau, ac_f, varargin{1:6});

            if iscolumn(varargin{1,3})
                % normal mode:
                % wf_R_b = varargin{1}
                % wf_p_b = varargin{2}
                % q_j    = varargin{3}
                % v_b    = varargin{5}
                dq_j = varargin{1,4};

                Jc_f = a_prms.Jc_f;
                M    = a_prms.M;
                c_qv = a_prms.c_qv;
            else
                % optimized mode:
                % djcdq_f = varargin{2}
                % M       = varargin{3}
                % c_qv    = varargin{4}
                Jc_f = varargin{1,1};
                dq_j = varargin{1,5};
            end
            nu = varargin{1,6};
        case 8 % optimized modes:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};
            nu   = varargin{1,3};

            [ac_h, a_prms] = handAccelerations(obj, feet_conf, hand_conf, tau, varargin{1:3});
            Jc_f = a_prms.Jc_f;
            M    = a_prms.M;
            c_qv = a_prms.c_qv;
        case 7
            dq_j = varargin{1,1};
            nu   = varargin{1,2};

            ac_f = zeroCtcAcc(obj, feet_conf);
            [ac_h, a_prms] = handAccelerations(obj, feet_conf, hand_conf, tau, ac_f, varargin{1:2});
            Jc_f = a_prms.Jc_f;
            M    = a_prms.M;
            c_qv = a_prms.c_qv;
        otherwise
            error('WBM::jointAccelerationsFHPC: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % compute the contact forces of the hands -- (optimized mode):
    [fc_h,~] = contactForcesCLPC(obj, hand_conf, tau, fe_h, ac_h, a_prms.Jc_h, ...
                                 a_prms.djcdq_h, M, c_qv, dq_j, nu);
    % calculate the total joint acceleration vector ddq_j in
    % dependency of the contact forces of the feet and hands:
    J_c  = vertcat(Jc_f, a_prms.Jc_h);
    f_c  = vertcat(a_prms.fc_f, fc_h);
    Jc_t = J_c.';
    ddq_j = M \ (a_prms.tau_gen + Jc_t*f_c - c_qv);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        fe_0 = zeroExtForces(obj, feet_conf);
        f_e = vertcat(fe_0, fe_h);
        a_c = vertcat(ac_f, ac_h);
        fd_prms = struct('tau_gen', a_prms.tau_gen, 'f_c', f_c, 'f_e', f_e, 'a_c', a_c);
    end
end
