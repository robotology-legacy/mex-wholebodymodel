function [Jc, djcdq] = contactJacobians(obj, varargin)
    n = obj.mwbm_model.ndof + 6;

    switch nargin
        case 7 % normal modes:
            % use only specific contact constraints:
            wf_R_b_arr = varargin{1,1};
            wf_p_b     = varargin{1,2};
            q_j        = varargin{1,3};
            dq_j       = varargin{1,4};
            v_b        = varargin{1,5};
            idx_list   = varargin{1,6};

            nCstrs = getNCstrs(idx_list);
        case 6
            % use all contact constraints:
            wf_R_b_arr = varargin{1,1};
            wf_p_b     = varargin{1,2};
            q_j        = varargin{1,3};
            dq_j       = varargin{1,4};
            v_b        = varargin{1,5};

            nCstrs = obj.mwbm_config.nCstrs;
            if (nCstrs == 0)
                % constraint list is empty ...
                Jc    = zeros(6,n);
                djcdq = obj.ZERO_CVEC_6;
                return
            end
            idx_list = 1:nCstrs;
        case 2 % optimized modes:
            % use only specific constraints:
            idx_list = varargin{1,1};
            nCstrs = getNCstrs(idx_list);
        case 1
            % use all constraints:
            nCstrs = obj.mwbm_config.nCstrs;
            if (nCstrs == 0)
                Jc    = zeros(6,n);
                djcdq = obj.ZERO_CVEC_6;
                return
            end
            idx_list = 1:nCstrs;
        otherwise
    end
    m     = 6*nCstrs;
    Jc    = zeros(m,n);
    djcdq = zeros(m,1);

    % compute for each contact constraint the Jacobian and the derivative Jacobian:
    if (nargin > 2)
        % normal mode:
        for i = 1:nCstrs
            idx = idx_list(1,i);
            ccstr_link = obj.mwbm_config.ccstr_link_names{1,idx};

            Jc(6*i-5:6*i,1:n)  = mexWholeBodyModel('jacobian', wf_R_b_arr, wf_p_b, q_j, ccstr_link); % 6*(i-1)+1 = 6*i-5
            djcdq(6*i-5:6*i,1) = mexWholeBodyModel('dJdq', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b, ccstr_link);
        end
    else
        % optimized mode:
        for i = 1:nCstrs
            idx = idx_list(1,i);
            ccstr_link = obj.mwbm_config.ccstr_link_names{1,idx};

            Jc(6*i-5:6*i,1:n)  = mexWholeBodyModel('jacobian', ccstr_link);
            djcdq(6*i-5:6*i,1) = mexWholeBodyModel('dJdq', ccstr_link);
        end
    end
end
%% END of contactJacobians.


%% NUMBER OF CONSTRAINTS:

function nCstrs = getNCstrs(idx_list)
    if isrow(idx_list) % (including scalar & not empty)
        nCstrs = size(idx_list,2);
        if (nCstrs > 1)
            WBM.utilities.chkfun.checkNumListAscOrder(idx_list, 'WBM::contactJacobians');
        end
    else
        error('WBM::contactJacobians: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
    end
end
