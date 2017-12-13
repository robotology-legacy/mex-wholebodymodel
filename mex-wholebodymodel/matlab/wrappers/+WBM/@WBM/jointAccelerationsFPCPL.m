function [ddq_j, fd_prms] = jointAccelerationsFPCPL(obj, foot_conf, hand_conf, tau, fhTotCWrench, f_cp, varargin)
    switch nargin
        case 13
            % with given foot contact acceleration:
            % get the mixed accelerations of the hands (with FPC) ...
            [ac_h, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, varargin{1:7});

            if iscolumn(varargin{1,4})
                % normal mode:
                % wf_R_b = varargin{2}
                % wf_p_b = varargin{3}
                % q_j    = varargin{4}
                % v_b    = varargin{6}
                % nu     = varargin{7}
                ac_f = varargin{1,1};
                dq_j = varargin{1,5};

                [M, c_qv, Jc_f] = getWBDynFeet(obj, a_prms);
            else
                % optimized mode:
                % djcdq_f = varargin{3}
                % nu      = varargin{7}
                ac_f = varargin{1,1};
                Jc_f = varargin{1,2};
                M    = varargin{1,4};
                c_qv = varargin{1,5};
                dq_j = varargin{1,6};
            end
        case 12
            % with zero foot contact acceleration:
            ac_f = zeroCtcAcc(obj, foot_conf);
            [ac_h, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, ac_f, varargin{1:6}); % with FPC

            if iscolumn(varargin{1,3})
                % normal mode:
                % wf_R_b = varargin{1}
                % wf_p_b = varargin{2}
                % q_j    = varargin{3}
                % v_b    = varargin{5}
                % nu     = varargin{6}
                dq_j = varargin{1,4};

                [M, c_qv, Jc_f] = getWBDynFeet(obj, a_prms);
            else
                % optimized mode:
                % djcdq_f = varargin{2}
                % nu      = varargin{6}
                Jc_f = varargin{1,1};
                M    = varargin{1,3};
                c_qv = varargin{1,4};
                dq_j = varargin{1,5};
            end
        case 10 % optimized modes:
            % with foot contact acceleration:
            % v_b = varargin{3}
            % nu  = varargin{4}
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};

            [ac_h, a_prms]  = handAccelerations(obj, foot_conf, hand_conf, tau, varargin{1:4}); % with FPC
            [M, c_qv, Jc_f] = getWBDynFeet(obj, a_prms);
        case 9
            % zero foot contact acceleration:
            % v_b = varargin{2}
            % nu  = varargin{3}
            dq_j = varargin{1,1};

            ac_f = zeroCtcAcc(obj, foot_conf);
            [ac_h, a_prms]  = handAccelerations(obj, foot_conf, hand_conf, tau, ac_f, varargin{1:3}); % with FPC
            [M, c_qv, Jc_f] = getWBDynFeet(obj, a_prms);
        otherwise
            error('WBM::jointAccelerationsFPCPL: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % get the mixed velocity and acceleration of the payload(s) ...
    v_pl = a_prms.Jc_h * dq_j;
    a_pl = ac_h;
    % calculate the payload forces of the hands:
    f_pl = handPayloadForces(obj, hand_conf, fhTotCWrench, f_cp, v_pl, a_pl);

    % compute the contact forces of the hands (without pose corrections):
    [fc_h,~] = contactForcesEF(obj, tau, f_pl, ac_h, a_prms.Jc_h, a_prms.djcdq_h, M, c_qv, dq_j); % optimized mode

    % calculate the total joint acceleration vector ddq_j in
    % dependency of the contact forces of the feet and hands:
    J_c  = vertcat(Jc_f, a_prms.Jc_h);
    f_c  = vertcat(a_prms.fc_f, fc_h);
    Jc_t = J_c.';
    ddq_j = M \ (a_prms.tau_gen + Jc_t*f_c - c_qv);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        a_c = vertcat(ac_f, ac_h);
        fd_prms = struct('tau_gen', a_prms.tau_gen, 'f_c', f_c, 'a_c', a_c, 'f_pl', f_pl);
    end
end
