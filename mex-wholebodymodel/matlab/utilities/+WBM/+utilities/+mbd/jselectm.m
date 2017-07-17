function S_j = jselectm(ndof, varargin)
    %% Joint selection matrix:
    %  Sources:
    %   [1] Multibody Dynamics Notation, S. Traversaro & A. Saccon, Eindhoven University of Technology,
    %       Department of Mechanical Engineering, 2016, <http://repository.tue.nl/849895>, p. 14, eq. (70).
    %   [2] Partial  Force  Control  of Constrained  Floating-Base  Robots, Andrea Del Prete & N. Mansard &
    %       F. Nori & G. Metta & L. Natale, CoRR, Volume 1410.4426, 2014, <https://arxiv.org/pdf/1410.4426.pdf>.
    switch nargin
        case 1
            % default - all actuated joints with deactivated floating base:
            S_1 = zeros(6,ndof);  % x_b (floating base position and orientation)
            S_2 = eye(ndof,ndof); % joint configuration q_j (actuated joints)
        case 2
            % only selected actuated joints (without floating base part):
            if isinteger(varargin{1,1})
                idx_list = varargin{1,1};

                if isrow(idx_list)
                    WBM.utilities.chkfun.checkNumListAscOrder(idx_list, 'jselectm');

                    len = size(idx_list,2);
                    WBM.utilities.chkfun.checkIdxListBounds(idx_list, len, ndof, 'jselectm');

                    S_j = zeros(ndof,ndof);
                    for i = 1:len
                        idx = idx_list(1,i);
                        S_j(idx,idx) = 1;
                    end
                    return
                else
                    error('jselectm: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                end
            elseif ischar(varargin{1,1})
                % only the floating base part and all actuated joints are deactivated:
                sel_type = varargin{1,1};

                if strcmp(sel_type, 'fltb')
                    S_1 = eye(6,6);
                    S_2 = zeros(ndof, 6);
                else
                    error('jselectm: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
                end
            else
                error('jselectm: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
        case 3
            if ( ischar(varargin{1,1}) && isinteger(varargin{1,2}) )
                sel_type = varargin{1,1};
                idx_list = varargin{1,2};

                if isrow(idx_list)
                    WBM.utilities.chkfun.checkNumListAscOrder(idx_list, 'jselectm');

                    len = size(idx_list,2);
                    WBM.utilities.chkfun.checkIdxListBounds(idx_list, len, ndof, 'jselectm');

                    switch sel_type
                        case 'jnts'
                            % selected actuated joints with deactivated floating base:
                            S_1 = zeros(6,ndof);
                            S_2 = zeros(ndof,ndof);

                            for i = 1:len
                                idx = idx_list(1,i);
                                S_2(idx,idx) = 1;
                            end
                        case 'fltb'
                            % floating base with selected actuated joints:
                            S_1 = horzcat(eye(6,6), zeros(6,ndof));

                            S_22 = zeros(ndof,ndof);
                            for i = 1:len
                                idx = idx_list(1,i);
                                S_22(idx,idx) = 1;
                            end

                            S_2 = horzcat(zeros(ndof,6), S_22);
                        otherwise
                            error('jselectm: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
                    end
                else
                    error('jselectm: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                end
            else
                error('jselectm: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
        otherwise
            error('jselectm: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % concatenate both selection matrices ...
    S_j = vertcat(S_1, S_2);
end
