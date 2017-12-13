function [ddq_j, fd_prms] = jointAccelerationsEF(obj, foot_conf, clnk_conf, tau, fe_c, ac, varargin)
    % ac_f ... mixed generalized accelerations of the foot contact points (is either zero or constant)
    switch nargin
        case 12 % normal modes:
            % with foot contact acceleration:
            % wf_R_b = varargin{2}
            % wf_p_b = varargin{3}
            % q_j    = varargin{4}
            % v_b    = varargin{6}
            ac_f = varargin{1,1};
            dq_j = varargin{1,5};

            % compute the whole body dynamics of all contact constraints
            % of the given contact link configurations ...
            [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf, ...
                                                                     varargin{2:4}, dq_j, varargin{1,6});
        case 11
            % with zero foot contact acceleration:
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % v_b    = varargin{5}
            dq_j = varargin{1,4};

            ac_f = zeroCtcAcc(obj, foot_conf);
            [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf, ...
                                                                     varargin{1:3}, dq_j, varargin{1,5});
        case 8 % optimized modes:
            % with foot contact acceleration:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};

            [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf);
        case 7
            % with zero foot contact acceleration:
            dq_j = varargin{1,1};

            ac_f = zeroCtcAcc(obj, foot_conf);
            [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf);
        otherwise
            error('WBM::jointAccelerationsEF: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % compute the contact forces with friction (optimized mode):
    [fc_f, tau_gen] = footContactForces(obj, foot_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j);
    [fc,~] = contactForcesEF(obj, tau, fe_c, ac, Jc, djcdq, M, c_qv, dq_j);

    % calculate the total joint acceleration vector ddq_j in
    % dependency of the contact forces of the contact constraints:
    J_c  = vertcat(Jc_f, Jc);
    f_c  = vertcat(fc_f, fc);
    Jc_t = J_c.';

    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv); % ddq_j = M^(-1) * (tau - c_qv - Jc.'*(-f_c))

    if (nargout == 2)
        % set the forward dynamics parameters ...
        fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'a_c', ac, 'f_e', fe_c);
    end
end
