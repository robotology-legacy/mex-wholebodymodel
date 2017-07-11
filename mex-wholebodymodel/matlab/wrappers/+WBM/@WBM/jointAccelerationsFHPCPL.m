function [ddq_j, fd_prms] = jointAccelerationsFHPCPL(obj, feet_conf, hand_conf, fhTotCWrench, f_cp, tau, varargin)
    switch nargin
        case 12 % normal modes:
            % ac_f   = varargin{1}
            % wf_R_b = varargin{2}
            % wf_p_b = varargin{3}
            % q_j    = varargin{4}
            % v_b    = varargin{6}
            dq_j = varargin{1,5};
            nu   = varargin{1,7};

            % get the mixed acceleration of the hand(s) at the contact link(s) ...
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
        case 9 % optimized modes:
            % ac_f = varargin{1}
            % v_b  = varargin{3}
            dq_j = varargin{1,2};
            nu   = varargin{1,4};

            [ac_h, a_prms] = handAccelerations(obj, feet_conf, hand_conf, tau, varargin{1:4});
        case 8
            % v_b  = varargin{2}
            dq_j = varargin{1,1};
            nu   = varargin{1,3};

            ac_f = zeroCtcAcc(obj, feet_conf);
            [ac_h, a_prms] = handAccelerations(obj, feet_conf, hand_conf, tau, ac_f, varargin{1:3});
        otherwise
            error('WBM::jointAccelerationsFHPCPL: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end
    % get the mixed velocity and acceleration of the payload(s) ...
    v_pl = a_prms.Jc_h * dq_j;
    a_pl = ac_h;
    % calculate the payload force(s):
    f_pl = payloadForces(obj, hand_conf, fhTotCWrench, f_cp, v_pl, a_pl);

    % compute the contact force(s) of the hand(s) -- (optimized mode):
    [fc_h,~] = contactForcesCLPC(obj, hand_conf, tau, f_pl, ac_h, a_prms.Jc_h, ...
                                 a_prms.djcdq_h, a_prms.M, a_prms.c_qv, dq_j, nu);
    % calculate the total joint acceleration vector ddq_j in
    % dependency of the contact forces of the feet and hands:
    J_c  = vertcat(a_prms.Jc_f, a_prms.Jc_h);
    f_c  = vertcat(a_prms.fc_f, fc_h);
    Jc_t = J_c.';
    ddq_j = M \ (a_prms.tau_gen + Jc_t*f_c - c_qv);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        a_c = vertcat(ac_f, ac_h);
        fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'f_pl', f_pl, 'a_c', a_c);
    end
end
