function [ddq_j, fd_prms] = jointAccelerationsFHPCPL(obj, foot_conf, hand_conf, tau, fhTotCWrench, f_cp, varargin)
    switch nargin
        case 13
            % get the mixed accelerations of the hands ...
            [wf_a_lnk, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, varargin{1:7});

            if iscolumn(varargin{1,4})
                % normal mode:
                % wf_R_b = varargin{2}
                % wf_p_b = varargin{3}
                % q_j    = varargin{4}
                % v_b    = varargin{6}
                ac_f = varargin{1,1};
                dq_j = varargin{1,5};

                [M, c_qv, Jc_f] = getWBDynFeet(obj, a_prms);
            else
                % optimized mode:
                % djcdq_f = varargin{3}
                ac_f = varargin{1,1};
                Jc_f = varargin{1,2};
                M    = varargin{1,4};
                c_qv = varargin{1,5};
                dq_j = varargin{1,6};
            end
            nu = varargin{1,7};
        case 12
            ac_f = zeroCtcAcc(obj, foot_conf);
            [wf_a_lnk, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, ac_f, varargin{1:6});

            if iscolumn(varargin{1,3})
                % normal mode:
                % wf_R_b = varargin{1}
                % wf_p_b = varargin{2}
                % q_j    = varargin{3}
                % v_b    = varargin{5}
                dq_j = varargin{1,4};

                [M, c_qv, Jc_f] = getWBDynFeet(obj, a_prms);
            else
                % optimized mode:
                % djcdq_f = varargin{2}
                Jc_f = varargin{1,1};
                M    = varargin{1,3};
                c_qv = varargin{1,4};
                dq_j = varargin{1,5};
            end
            nu = varargin{1,6};
        case 10 % optimized modes:
            % v_b = varargin{3}
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};
            nu   = varargin{1,4};

            [wf_a_lnk, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, varargin{1:4});
            [M, c_qv, Jc_f]    = getWBDynFeet(obj, a_prms);
        case 9
            % v_b = varargin{2}
            dq_j = varargin{1,1};
            nu   = varargin{1,3};

            ac_f = zeroCtcAcc(obj, foot_conf);
            [wf_a_lnk, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, ac_f, varargin{1:3});
            [M, c_qv, Jc_f]    = getWBDynFeet(obj, a_prms);
        otherwise
            error('WBM::jointAccelerationsFHPCPL: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % mixed velocity of the hands at the contact links {lnk} ...
    wf_v_lnk = a_prms.Jc_h * dq_j;

    % apply velocity and acceleration saturation:
    % (to prevent overload & integration problems)
    wf_v_lnk = WBM.utilities.mbd.satVel(wf_v_lnk);
    wf_a_lnk = WBM.utilities.mbd.satAcc(wf_a_lnk);

    % calculate the payload forces of the hands in contact space {c} = {lnk}:
    f_pl = handPayloadForces(obj, hand_conf, fhTotCWrench, f_cp, wf_v_lnk, wf_a_lnk);

    % compute the contact forces of the hands (optimized mode):
    [fc_h,~] = contactForcesCLPCEF(obj, hand_conf, tau, f_pl, wf_a_lnk, a_prms.Jc_h, ...
                                   a_prms.djcdq_h, M, c_qv, dq_j, nu);

    % calculate the total joint acceleration vector ddq_j in
    % dependency of the contact forces of the feet and hands:
    J_c   = vertcat(Jc_f, a_prms.Jc_h);
    f_c   = vertcat(a_prms.fc_f, fc_h);
    Jc_t  = J_c.';
    ddq_j = M \ (a_prms.tau_gen + Jc_t*f_c - c_qv);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        a_c = vertcat(ac_f, wf_a_lnk);
        fd_prms = struct('tau_gen', a_prms.tau_gen, 'f_c', f_c, 'a_c', a_c, 'f_pl', f_pl);
    end
end
