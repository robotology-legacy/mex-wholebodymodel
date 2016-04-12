classdef wbmBody
    properties(SetAccess = private, GetAccess = public)
       chains@cell    matrix = {};
       nChains@uint16 scalar = 0;
       joints@cell    matrix = {};
       nJoints@uint16 scalar = 0;
    end

    methods
        function obj = wbmBody(chain_names, chain_pos, joint_names, joint_idx)
            if ( (nargin ~= 2) && (nargin ~= 4) )
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            %% Chains:
            % verify the input types ...
            if ( ~iscell(chain_names) || ~ismatrix(chain_pos) )
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);                     
            end
            % check the dimensions ...
            [nrows, ncols] = size(chain_pos);
            if (ncols ~= 2)
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);       
            end

            if (size(chain_names,1) ~= nrows)
                error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            % store the chain data into the cell array ...
            obj.nChains = nrows;
            obj.chains = horzcat(chain_names, num2cell(chain_pos));

            %% Joints:
            if (nargin == 4)
                % check input types ...
                if ( ~iscell(joint_names) || ~isvector(joint_idx) )
                    error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);                     
                end
                % check dimensions ...
                len = size(joint_names,1);
                if (len ~= size(joint_idx,1))
                    error('wbmBody::wbmBody: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
                end
                % store the joint data ...
                obj.nJoints = len;
                obj.joints = horzcat(joint_names, num2cell(joint_idx));
            end
        end

        function chn_tbl = getChainTable(obj)
            chn_tbl = cell2table(obj.chains, 'VariableNames', {'chain_name', 'start_idx', 'end_idx'});
        end

        function jnt_tbl = getJointTable(obj)
            jnt_tbl = cell2table(obj.joints, 'VariableNames', {'joint_name', 'idx'});
        end

    end
end
