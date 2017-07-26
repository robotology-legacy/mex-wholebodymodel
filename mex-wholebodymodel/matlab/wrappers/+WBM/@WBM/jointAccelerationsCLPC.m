function [ddq_j, fd_prms] = jointAccelerationsCLPC(obj, clink_conf, tau, f_e, a_c, varargin)
    switch nargin
        case 11
            if iscolumn(varargin{1,3})
                % normal mode (generalized forces with friction):
                % wf_R_b = varargin{1}
                wf_p_b = varargin{1,2};
                q_j    = varargin{1,3};
                dq_j   = varargin{1,4};
                v_b    = varargin{1,5};
                nu     = varargin{1,6};

                % compute the whole body dynamics and for each contact constraint
                % the Jacobian and the derivative Jacobian ...
                wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCS(obj, clink_conf, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                % get the contact forces and the corresponding generalized forces ...
                [f_c, tau_gen] = contactForcesCLPC(obj, clink_conf, tau, f_e, a_c, Jc, djcdq, M, c_qv, ...
                                                   wf_R_b_arr, wf_p_b, q_j, dq_j, v_b, nu);
            else
                % optimized mode (with friction):
                % djcdq = varargin{2}
                % dq_j  = varargin{5}
                % nu    = varargin{6}
                Jc   = varargin{1,1};
                M    = varargin{1,3};
                c_qv = varargin{1,4};

                [f_c, tau_gen] = contactForcesCLPC(obj, clink_conf, tau, f_e, a_c, Jc, varargin{1,2}, ...
                                                   M, c_qv, varargin{5:6});
            end
        case 10
            % optimized mode (without friction):
            % djcdq = varargin{2}
            % nu    = varargin{5}
            Jc   = varargin{1,1};
            M    = varargin{1,3};
            c_qv = varargin{1,4};

            [f_c, tau_gen] = contactForcesCLPC(obj, clink_conf, tau, f_e, a_c, Jc, varargin{1,2}, ...
                                               M, c_qv, varargin{1,5});
        case 9
            % semi-optimized mode (without friction):
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % nu     = varargin{4}
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCS(obj, clink_conf);
            [f_c, tau_gen] = contactForcesCLPC(obj, clink_conf, tau, f_e, a_c, Jc, djcdq, ...
                                               M, c_qv, wf_R_b_arr, varargin{2:4});
        case 6
            % optimized mode (without friction):
            nu = varargin{1,1};

            [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCS(obj, clink_conf);
            [f_c, tau_gen] = contactForcesCLPC(obj, clink_conf, tau, f_e, a_c, Jc, djcdq, M, c_qv, nu);
        otherwise
            error('WBM::jointAccelerationsCLPC: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end

    % Joint Acceleration q_ddot (derived from the state-space equation):
    % For further details see:
    %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
    Jc_t  = Jc.';
    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv); % ddq_j = M^(-1) * (...)

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'a_c', a_c, 'f_e', f_e);
    end
end
