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
            error('WBM::jointAccelerationsNFBPL: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % remove the floating base part ...
    M    = M(7:end,7:end);
    c_qv = c_qv(7:end,1);
    Jc_h = Jc_h(:,7:end);

    % get the generalized forces with friction ...
    tau_fr  = frictionForces(obj, dq_j); % friction torques (negated torque values)
    tau_gen = tau + tau_fr;              % generalized forces tau_gen = S_j*(tau + (-tau_fr)),
                                         % S_j = [0_(6xn); I_(nxn)] ... joint selection matrix.

    % calculate the mixed accelerations of the hands at the contact links {lnk}:
    % note: since this forward dynamics model has no floating base, there
    %       are no contact forces and accelerations on the ground, i.e.
    %       Jcf_t*fc_f = 0, the same as a robot with a fixed base.
    ddq      = M \ (tau_gen - c_qv);
    wf_a_lnk = Jc_h*ddq + djcdq_h;
    % mixed velocity of the hands at the contact links {lnk} ...
    wf_v_lnk = Jc_h * dq_j;

    % apply velocity and acceleration saturation:
    % (to prevent overload & integration problems)
    wf_v_lnk = WBM.utilities.mbd.satVel(wf_v_lnk);
    wf_a_lnk = WBM.utilities.mbd.satAcc(wf_a_lnk);

    % calculate the payload forces of the hands in contact space {c} = {lnk}:
    f_pl = handPayloadForces(obj, hand_conf, fhTotCWrench, f_cp, wf_v_lnk, wf_a_lnk);

    % compute the contact forces of the hands (with friction):
    [fc_h,~] = contactForcesEF(obj, tau, f_pl, wf_a_lnk, Jc_h, djcdq_h, M, c_qv, dq_j);

    % calculate the total joint acceleration vector ddq_j without floating base,
    % in dependency of the current payload forces at each contact point p_c:
    Jc_t  = Jc_h.';
    ddq_j = M \ (tau_gen + Jc_t*fc_h - c_qv);
    % ddq_j = M \ (tau_gen + Jc_t*f_pl - c_qv);
    ddq_j = vertcat(zeros(6,1), ddq_j);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        fd_prms = struct('tau_gen', tau_gen, 'f_c', fc_h, 'a_c', wf_a_lnk, 'f_pl', f_pl);
    end
end
