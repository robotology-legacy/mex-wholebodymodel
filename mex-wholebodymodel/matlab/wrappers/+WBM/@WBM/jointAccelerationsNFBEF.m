function [ddq_j, fd_prms] = jointAccelerationsNFBEF(obj, clnk_conf, tau, fe_c, ac, varargin)
    switch nargin
        case 10
            % normal mode:
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % v_b    = varargin{5}
            dq_j = varargin{1,4};

            % compute the whole body dynamics of each contact constraint
            % of the given contact links configuration ...
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [M, c_qv, J_c, djcdq] = wholeBodyDynamicsCS(obj, clnk_conf, wf_R_b_arr, varargin{1,2}, ...
                                                        varargin{1,3}, dq_j, varargin{1,5});
        case 8 % optimized modes:
            M    = varargin{1,1};
            c_qv = varargin{1,2};
            dq_j = varargin{1,3};

            [J_c,~] = contactJacobiansCS(obj, clnk_conf);
        case 6
            dq_j = varargin{1,1};
            [M, c_qv, J_c, djcdq] = wholeBodyDynamicsCS(obj, clink_conf);
        otherwise
            error('WBM::jointAccelerationsNFBEF: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % remove the floating base part ...
    M    = M(7:end,7:end);
    c_qv = c_qv(7:end,1);
    J_c  = J_c(:,7:end);

    % % get the generalized forces with friction ...
    % tau_fr  = frictionForces(obj, dq_j); % friction torques (negated torque values)
    % tau_gen = tau + tau_fr;

    % contact forces and generalized forces (with friction):
    [f_c, tau_gen] = contactForcesEF(obj, tau, fe_c, ac, J_c, djcdq, M, c_qv, dq_j);

    % calculate the total joint acceleration vector ddq_j without
    % floating base and in dependency of the external forces f_e:
    Jc_t  = J_c.';
    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv);
    % ddq_j = M \ (tau_gen - Jc_t*fe_c - c_qv);
    ddq_j = vertcat(zeros(6,1), ddq_j);

    if (nargout == 2)
        % set the forward dynamics parameters ...
        fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'f_e', fe_c);
    end
end
