function [ddq_j, fd_prms] = jointAccelerationsNFB(obj, tau, varargin)
    switch nargin
        case 7 % normal modes:
            % generalized forces with friction:
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % v_b    = varargin{5}
            dq_j = varargin{1,4};

            tau_fr  = frictionForces(obj, dq_j); % friction torques (negated torque values)
            tau_gen = tau + tau_fr;              % generalized forces tau_gen = S_j*(tau + (-tau_fr)),
                                                 % S_j = [0_(6xn); I_(nxn)] ... joint selection matrix.
            % compute the whole body dynamics ...
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [M, c_qv] = wholeBodyDyn(obj, wf_R_b_arr, varargin{1,2}, varargin{1,3}, dq_j, varargin{1,5});
        case 6
            % without friction:
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            nu = varargin{1,4};

            len  = obj.mwbm_model.ndof + 6;
            dq_j = nu(7:len,1);
            v_b  = nu(1:6,1);

            tau_gen = tau;

            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            [M, c_qv] = wholeBodyDyn(obj, wf_R_b_arr, varargin{1,2}, varargin{1,3}, dq_j, v_b);

        case 5 % optimized modes:
            % with friction:
            % dq_j = varargin{3}
            M    = varargin{1,1};
            c_qv = varargin{1,2};

            tau_fr  = frictionForces(obj, varargin{1,3});
            tau_gen = tau + tau_fr;
        case 4
            % without friction:
            M    = varargin{1,1};
            c_qv = varargin{1,2};

            tau_gen = tau;
        case 3
            % with friction:
            % dq_j = varargin{1}

            tau_fr  = frictionForces(obj, varargin{1,1});
            tau_gen = tau + tau_fr;

            [M, c_qv] = wholeBodyDyn(obj);
        case 2
            % without friction:
            tau_gen   = tau;
            [M, c_qv] = wholeBodyDyn(obj);
        otherwise
            error('WBM::jointAccelerationsNFB: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % remove the floating base part ...
    M    = M(7:end,7:end);
    c_qv = c_qv(7:end,1);

    % Joint Acceleration q_ddot without floating base (derived from the state-space equation):
    % Note: Since the robot has no floating base, there are no contact forces f_c to the
    %       ground, i.e. f_c = 0. The behavior is the same as a robot with a fixed base.
    %
    % For further details see:
    %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
    ddq_j = M \ (tau_gen - c_qv);
    ddq_j = vertcat(zeros(6,1), ddq_j);

    if (nargout == 2)
        % set the forward dynamics parameters ...
        fd_prms = struct('tau_gen', tau_gen);
    end
end
