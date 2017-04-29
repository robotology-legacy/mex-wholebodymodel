function [ddq_j, fd_prms] = jointAccelerationsCLPC(obj, clink_conf, tau, f_e, a_c, varargin)
    switch nargin
        case 14 % normal mode:
            % generalized forces with friction:
            % wf_R_b = varargin{1}
            wf_p_b = varargin{1,2};
            q_j    = varargin{1,3};
            dq_j   = varargin{1,4};
            v_b    = varargin{1,5};

            % compute the whole body dynamics and for each contact constraint
            % the Jacobian and the derivative Jacobian ...
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [M, c_qv, Jc, djcdq] = rigidBodyDynCJacobiansCS(obj, clink_conf, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
            % get the contact forces and the corresponding generalized forces ...
            [f_c, tau_gen] = contactForcesCLPC(obj, clink_conf, tau, f_e, a_c, Jc, djcdq, M, c_qv, ...
                                               wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
        case 13 % semi-optimized mode:
            % general case:
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % nu     = varargin{4}
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [M, c_qv, Jc, djcdq] = rigidBodyDynCJacobiansCS(obj, clink_conf);
            [f_c, tau_gen] = contactForcesCLPC(obj, clink_conf, tau, f_e, a_c, Jc, djcdq, ...
                                               M, c_qv, wf_R_b_arr, varargin{2:4});
        case {10, 11} % optimized modes:
            Jc   = varargin{1,1};
            % djcdq = varargin{2}
            M    = varargin{1,3};
            c_qv = varargin{1,4};
            % with friction:
            %   dq_j = varargin{5}
            %   nu   = varargin{6}
            % general case:
            %   nu   = varargin{5}

            [f_c, tau_gen] = contactForcesCLPC(obj, clink_conf, tau, f_e, a_c, Jc, varargin{1,2}, ...
                                               M, c_qv, varargin{5:end});
        case 7
            dq_j = varargin{1,1};
            v_b  = varargin{1,2};
            nu   = vertcat(v_b, dq_j);

            [M, c_qv, Jc, djcdq] = rigidBodyDynCJacobiansCS(obj, clink_conf);
            [f_c, tau_gen] = contactForcesCLPC(obj, clink_conf, tau, f_e, a_c, Jc, djcdq, M, c_qv, nu);
        otherwise
            error('WBM::jointAccelerationsCLPC: %s', WBM.wbmErrorMsg.WRONG_ARG);
    end

    % Joint Acceleration q_ddot (derived from the state-space equation):
    % For further details see:
    %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
    Jc_t  = Jc.';
    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv); % ddq_j = M^(-1) * (...)

    % forward dynamics parameters ...
    fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'f_e', f_e, 'a_c', a_c);
end
