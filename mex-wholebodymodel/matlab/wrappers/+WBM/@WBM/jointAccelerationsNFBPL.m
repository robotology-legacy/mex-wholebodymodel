function [ddq_j, fd_prms] = jointAccelerationsNFBPL(obj, hand_conf, tau, fhTotCWrench, f_cp, varargin)
    switch nargin
        case 10
            % normal mode:
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % v_b    = varargin{5}
            dq_j = varargin{1,4};

            % compute the whole body dynamics of the hands ...
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [M, c_qv, Jc_h, djcdq_h] = wholeBodyDynamicsCS(obj, hand_conf, wf_R_b_arr, varargin{1,2}, ...
                                                           varargin{1,3}, dq_j, varargin{1,5});
        case 8 % optimized modes:
            M    = varargin{1,1};
            c_qv = varargin{1,2};
            dq_j = varargin{1,3};

            [Jc_h, djcdq_h] = contactJacobiansCS(obj, hand_conf);
        case 6
            dq_j = varargin{1,1};
            [M, c_qv, Jc_h, djcdq_h] = wholeBodyDynamicsCS(obj, hand_conf);
        otherwise
            error('WBM::jointAccelerationsPL: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % remove the floating base part ...
    M    = M(7:end,7:end);
    c_qv = c_qv(7:end,1);
    Jc_h = Jc_h(:,7:end);

    % calculate the mixed accelerations of the hands at the contact links:
    % note: since this forward dynamics model has no floating base, there
    %       are no contact forces and accelerations on the ground, i.e.
    %       ddqj_f = 0 and thus, Jc_h*ddqj_f = 0 -- the same as a robot
    %       with a fixed base.
    ac_h = djcdq_h;

    % get the mixed velocity and acceleration of the payload(s) ...
    v_pl = Jc_h * dq_j;
    a_pl = ac_h;
    % calculate the payload forces of the hands:
    f_pl = handPayloadForces(obj, hand_conf, fhTotCWrench, f_cp, v_pl, a_pl);

    % compute the contact forces of the hands (with friction):
    [fc_h, tau_gen] = contactForcesEF(obj, tau, f_pl, ac_h, Jc_h, djcdq_h, M, c_qv, dq_j);

    % calculate the total joint acceleration vector ddq_j without
    % floating base and in dependency of the hand contact forces:
    Jc_t  = Jc_h.';
    ddq_j = M \ (tau_gen + Jc_t*fc_h - c_qv);
    ddq_j = vertcat(zeros(6,1), ddq_j);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        fd_prms = struct('tau_gen', tau_gen, 'f_c', fc_h, 'a_c', ac_h, 'f_pl', f_pl);
    end
end
