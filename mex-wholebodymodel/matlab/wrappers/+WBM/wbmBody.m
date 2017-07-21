classdef wbmBody
    properties(SetAccess = private, GetAccess = public)
       chains@cell    matrix
       nChains@uint16 scalar = 0;
       joints@cell    matrix
       nJoints@uint16 scalar = 0;
    end

    methods
        function obj = wbmBody(chain_names, chain_idx, joint_names, joint_idx)
            if (nargin ~= 4)
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end

            %% Chains:
            % verify the input types ...
            if ( ~iscellstr(chain_names) || ~ismatrix(chain_idx) )
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % check the dimensions ...
            [nRows, nCols] = size(chain_idx);
            if (nCols ~= 2)
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            if (size(chain_names,1) ~= nRows)
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            % store the chain data into the cell array ...
            obj.nChains = nRows;
            obj.chains  = horzcat(chain_names, num2cell(chain_idx));

            %% Joints:
            % check input types ...
            if ( ~iscellstr(joint_names) || ~isvector(joint_idx) )
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % check dimensions ...
            nJnts = size(joint_names,1);
            if (nJnts ~= size(joint_idx,1))
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            % store the joint data ...
            obj.nJoints = nJnts;
            obj.joints  = horzcat(joint_names, num2cell(joint_idx));
        end

        function jnt_idx = getChainIndices(obj, chain_name)
            pos = find(strcmp(obj.chains(1:obj.nChains,1), chain_name));
            if isempty(pos)
                error('wbmBody::getChainIndices: %s', WBM.wbmErrorMsg.NAME_NOT_EXIST);
            end

            start_idx = obj.chains{pos,2};
            end_idx   = obj.chains{pos,3};
            jnt_idx   = start_idx:end_idx;
        end

        function jnt_idx = getJointIndex(obj, joint_name)
            jnt_idx = find(strcmp(obj.joints(1:obj.nJoints,1), joint_name));
        end

        function jnt_names = getJointNames(obj, joint_idx)
            % check ranges ...
            if isscalar(joint_idx)
                if ( (joint_idx > obj.nJoints) || (joint_idx < 1) )
                    error('wbmBody::getJointNames: %s', WBM.wbmErrorMsg.IDX_OUT_OF_BOUNDS);
                end

                jnt_names = obj.joints{joint_idx, 1};
                return
            end
            % else ...
            if ~isrow(joint_idx)
                error('wbmBody::getJointNames: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            % we assume that the elements are in ascending or descending order ...
            n = size(joint_idx,2);
            if ( (joint_idx(1,1) > obj.nJoints) || (joint_idx(1,1) < 1) || ...
                 (joint_idx(1,n) > obj.nJoints) || (joint_idx(1,n) < 1) )
                error('wbmBody::getJointNames: %s', WBM.wbmErrorMsg.IDX_OUT_OF_BOUNDS);
            end

            jnt_names = obj.joints(joint_idx,1);
        end

        function chn_tbl = getChainTable(obj)
            chn_tbl = cell2table(obj.chains, 'VariableNames', {'chain_name', 'start_idx', 'end_idx'});
        end

        function jnt_tbl = getJointTable(obj)
            jnt_tbl = cell2table(obj.joints, 'VariableNames', {'joint_name', 'idx'});
        end

    end
end
