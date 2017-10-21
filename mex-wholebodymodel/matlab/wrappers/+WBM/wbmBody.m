classdef wbmBody < handle
    properties(SetAccess = private, GetAccess = public)
       chains@cell matrix
       nChns@uint8 scalar = 0;
       joints@cell matrix
       nJnts@uint8 scalar = 0;
    end

    methods
        function obj = wbmBody(chn_names, chn_idx, jnt_names, jnt_idx)
            if (nargin ~= 4)
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end

            %% Chains:
            % verify the input types ...
            if ( ~iscellstr(chn_names) || ~ismatrix(chn_idx) )
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % check the dimensions ...
            [nRows, nCols] = size(chn_idx);
            if (nCols ~= 2)
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            if (size(chn_names,1) ~= nRows)
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            % store the chain data into the cell array ...
            obj.nChns  = nRows;
            obj.chains = horzcat(chn_names, num2cell(chn_idx));

            %% Joints:
            % check input types ...
            if ( ~iscellstr(jnt_names) || ~isvector(jnt_idx) )
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % check dimensions ...
            nJnts = size(jnt_names,1);
            if (nJnts ~= size(jnt_idx,1))
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            % store the joint data ...
            obj.nJnts  = nJnts;
            obj.joints = horzcat(jnt_names, num2cell(jnt_idx));
        end

        function jnt_idx = getChainIndices(obj, chn_name)
            pos = find(strcmp(obj.chains(1:obj.nChns,1), chn_name));
            if isempty(pos)
                error('wbmBody::getChainIndices: %s', WBM.wbmErrorMsg.NAME_NOT_EXIST);
            end

            start_idx = obj.chains{pos,2};
            end_idx   = obj.chains{pos,3};
            jnt_idx   = start_idx:end_idx;
        end

        function jnt_idx = getJointIndex(obj, jnt_name)
            jnt_idx = find(strcmp(obj.joints(1:obj.nJnts,1), jnt_name));
        end

        function jnt_names = getJointNames(obj, jnt_idx)
            % check ranges ...
            if isscalar(jnt_idx)
                if ( (jnt_idx > obj.nJnts) || (jnt_idx < 1) )
                    error('wbmBody::getJointNames: %s', WBM.wbmErrorMsg.IDX_OUT_OF_BOUNDS);
                end

                jnt_names = obj.joints{jnt_idx, 1};
                return
            end
            % else ...
            if ~isrow(jnt_idx)
                error('wbmBody::getJointNames: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            % we assume that the elements are in ascending or descending order ...
            n = size(jnt_idx,2);
            if ( (jnt_idx(1,1) > obj.nJnts) || (jnt_idx(1,1) < 1) || ...
                 (jnt_idx(1,n) > obj.nJnts) || (jnt_idx(1,n) < 1) )
                error('wbmBody::getJointNames: %s', WBM.wbmErrorMsg.IDX_OUT_OF_BOUNDS);
            end

            jnt_names = obj.joints(jnt_idx,1);
        end

        function chn_tbl = getChainTable(obj)
            chn_tbl = cell2table(obj.chains, 'VariableNames', {'chain_name', 'start_idx', 'end_idx'});
        end

        function jnt_tbl = getJointTable(obj)
            jnt_tbl = cell2table(obj.joints, 'VariableNames', {'joint_name', 'idx'});
        end

    end
end
