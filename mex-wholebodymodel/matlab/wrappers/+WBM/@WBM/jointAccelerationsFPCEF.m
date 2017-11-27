function [ddq_j, fd_prms] = jointAccelerationsFPCEF(obj, foot_conf, clnk_conf, tau, fe_c, ac, varargin)
    % ac_f ... mixed generalized accelerations of the foot contact points (is either zero or constant)
    switch nargin
        case 13
            % with given foot contact acceleration:
            if iscolumn(varargin{1,4})
                % normal mode:
                % wf_R_b = varargin{2}
                % wf_p_b = varargin{3}
                % q_j    = varargin{4}
                % v_b    = varargin{6}
                ac_f = varargin{1,1};
                dq_j = varargin{1,5};

                % compute the whole body dynamics and for each contact constraint
                % the Jacobian and the corresponding derivative Jacobian ...
                [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf, ...
                                                                         varargin{2:4}, dq_j, varargin{1,6});
            else
                % optimized mode:
                ac_f    = varargin{1,1};
                Jc_f    = varargin{1,2};
                djcdq_f = varargin{1,3};
                M       = varargin{1,4};
                c_qv    = varargin{1,5};
                dq_j    = varargin{1,6};

                [Jc, djcdq] = contactJacobiansCS(obj, clnk_conf);
            end
            nu = varargin{1,7};
        case 11
            % with zero foot contact acceleration:
            if iscolumn(varargin{1,3})
                % normal mode:
                % wf_R_b = varargin{1}
                % wf_p_b = varargin{2}
                % q_j    = varargin{3}
                % v_b    = varargin{5}
                dq_j = varargin{1,4};

                [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf, ...
                                                                         varargin{1:3}, dq_j, varargin{1,5});
            else
                % optimized mode:
                Jc_f    = varargin{1,1};
                djcdq_f = varargin{1,2};
                M       = varargin{1,3};
                c_qv    = varargin{1,4};
                dq_j    = varargin{1,5};

                [Jc, djcdq] = contactJacobiansCS(obj, clnk_conf);
            end
            nu   = varargin{1,6};
            ac_f = zeroCtcAcc(obj, foot_conf);
        case 8 % optimized modes:
            % with foot contact acceleration:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};
            nu   = varargin{1,3};

            [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf);
        case 7
            % zero foot contact acceleration:
            dq_j = varargin{1,1};
            nu   = varargin{1,2};
            ac_f = zeroCtcAcc(obj, foot_conf);

            [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf);
        otherwise
            error('WBM::jointAccelerationsFPCEF: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % compute the contact forces with friction (optimized mode):
    [fc_f, tau_gen] = footContactForcesPC(obj, foot_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j, nu); % with PC
    [fc,~] = contactForcesEF(obj, tau, fe_c, ac, Jc, djcdq, M, c_qv, dq_j);

    % calculate the total joint acceleration vector ddq_j in
    % dependency of the contact forces of the contact constraints:
    J_c  = vertcat(Jc_f, Jc);
    f_c  = vertcat(fc_f, fc);
    Jc_t = J_c.';
    ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv);

    if (nargout == 2)
        % data structure of the calculated forward dynamics parameters ...
        fe_0 = zeroExtForces(obj, foot_conf);
        f_e = vertcat(fe_0, fe_c);
        a_c = vertcat(ac_f, ac);
        fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c, 'a_c', a_c, 'f_e', f_e);
    end
end
