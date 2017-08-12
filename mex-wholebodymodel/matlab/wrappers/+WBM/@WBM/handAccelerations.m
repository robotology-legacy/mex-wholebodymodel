function [ac_h, a_prms] = handAccelerations(obj, feet_conf, hand_conf, tau, varargin)
    % check the contact state (CS) of the hands ...
    hand_idx = getContactIdx(obj, hand_conf);
    if ~hand_idx
        % no contacts:
        ac_h = obj.ZERO_CVEC_12;
        if (nargout == 2)
            a_prms = struct(); % empty structure ...
        end
        return
    end

    f_data = true;
    switch nargin
        case 11 % with pose correction:
            if iscolumn(varargin{1,4})
                % normal mode:
                % wf_R_b = varargin{2}
                % wf_p_b = varargin{3}
                % q_j    = varargin{4}
                % v_b    = varargin{6}
                ac_f = varargin{1,1};
                dq_j = varargin{1,5};
                nu   = varargin{1,7}; % mixed generalized velocity

                [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = wholeBodyDynFHCS(obj, feet_conf, hand_idx, ...
                                                                           varargin{2:4}, dq_j, varargin{1,6});
            else
                % optimized mode:
                ac_f    = varargin{1,1};
                Jc_f    = varargin{1,2};
                djcdq_f = varargin{1,3};
                M       = varargin{1,4};
                c_qv    = varargin{1,5};
                dq_j    = varargin{1,6};
                nu      = varargin{1,7};

                [Jc_h, djcdq_h] = contactJacobians(obj, hand_idx);
                f_data = false;
            end
            % compute the feet contact forces with pose correction (PC) and the
            % corresponding generalized forces with friction ...
            [fc_f, tau_gen] = feetContactForcesPC(obj, feet_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j, nu);
        case 10
            % normal mode (without pose correction):
            % wf_R_b = varargin{2}
            % wf_p_b = varargin{3}
            % q_j    = varargin{4}
            % v_b    = varargin{6}
            ac_f = varargin{1,1};
            dq_j = varargin{1,5};

            [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = wholeBodyDynFHCS(obj, feet_conf, hand_idx, ...
                                                                       varargin{2:4}, dq_j, varargin{1,6});
            % compute the feet contact forces with the corresponding generalized forces ...
            [fc_f, tau_gen] = feetContactForces(obj, feet_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j); % with friction
        case 7 % optimized modes:
            % with pose correction:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};
            nu   = varargin{1,3};

            [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = wholeBodyDynFHCS(obj, feet_conf, hand_idx);
            [fc_f, tau_gen] = feetContactForcesPC(obj, feet_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j, nu);
        case 6
            % without pose correction:
            ac_f = varargin{1,1};
            dq_j = varargin{1,2};

            [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = wholeBodyDynFHCS(obj, feet_conf, hand_idx);
            [fc_f, tau_gen] = feetContactForces(obj, feet_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j);
        otherwise
            error('WBM::handAccelerations: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % calculate the joint accelerations with the feet contact constraints ...
    Jcf_t = Jc_f.';
    ddqj_f = M \ (tau_gen + Jcf_t*fc_f - c_qv);

    % calculate the mixed acceleration of the hand(s) at the contact link(s):
    ac_h = Jc_h*ddqj_f + djcdq_h;

    if (nargout == 2)
        % data structure of the calculated acceleration parameters:
        if f_data
            % with feet data Jc_f, djcdq_f, M and c_qv ...
            a_prms = struct('M', M, 'c_qv', c_qv, 'Jc_f', Jc_f, 'Jc_h', Jc_h, 'djcdq_f', djcdq_f, ...
                            'djcdq_h', djcdq_h, 'fc_f', fc_f, 'tau_gen', tau_gen);
            return
        end
        % else, without Jc_f, djcdq_f, M and c_qv (existing outside) ...
        a_prms = struct('Jc_h', Jc_h, 'djcdq_h', djcdq_h, 'fc_f', fc_f, 'tau_gen', tau_gen);
    end
end
%% END of handAccelerations.


%% WHOLE BODY DYNAMICS FOR FEET & HANDS:

function [M, c_qv, Jc_f, djcdq_f, Jc_h, djcdq_h] = wholeBodyDynFHCS(obj, feet_conf, hand_idx, wf_R_b, wf_p_b, q_j, dq_j, v_b) % FH ... Feet and Hands, CS ... Contact State
    switch nargin
        case 8
            % normal mode:
            wf_R_b_arr = reshape(wf_R_b, 9, 1);
            [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, feet_conf, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
            [Jc_h, djcdq_h] = contactJacobians(obj, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b, hand_idx);
        case 3
            % optimized mode:
            [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, feet_conf);
            [Jc_h, djcdq_h] = contactJacobians(obj, hand_idx);
        otherwise
            error('wholeBodyDynFHCS: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
end
