classdef WBM < WBM.WBMBase
    properties(Dependent)
        stvChiInit@double vector
        stvLen@uint16     scalar
        vqTInit@double    vector
        stvqT@double      vector
        robot_init_state@WBM.wbmStateParams
        robot_config@WBM.wbmBaseRobotConfig
        robot_body@WBM.wbmBody
    end

    properties(Access = protected)
        mwbm_config@WBM.wbmBaseRobotConfig
    end

    methods
        % Constructor:
        function obj = WBM(robot_model, robot_config, wf2fixLnk)
            % call the constructor of the superclass ...
            obj = obj@WBM.WBMBase(robot_model);

            if ~exist('robot_config', 'var')
                error('WBM::WBM: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            if ~exist('wf2fixLnk', 'var')
                wf2fixLnk = false; % default value ...
            else
                if ~islogical(wf2fixLnk)
                    error('WBM::WBM: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                end
            end

            obj.initConfig(robot_config);
            if wf2fixLnk
                % set the world frame (WF) at the initial roto-translation from
                % the chosen fixed link, i.e. the first entry of the constraint list:
                obj.setWorldFrameFromFixedLink(obj.mwbm_config.cstr_link_names{1});
            end
            % retrieve and update the initial roto-translation (VQS-Transf.) of the robot base (world frame) ...
            obj.updateInitRotoTranslation();
        end

        % Copy-function:
        function newObj = copy(obj)
            newObj = copy@WBM.WBMBase(obj);
        end

        % Destructor:
        function delete(obj)
            delete@WBM.WBMBase(obj);
        end

        function setWorldFrameFromFixedLink(obj, urdf_link_name, q_j, dq_j, v_b, g_wf)
            if (nargin < 6)
                switch nargin
                    case 5
                        % use the default gravity vector ...
                        g_wf = obj.mwbm_model.g_wf;
                    case 2
                        % use the initial state values ...
                        v_b  = vertcat(obj.mwbm_config.init_state_params.dx_b, obj.mwbm_config.init_state_params.omega_b);
                        q_j  = obj.mwbm_config.init_state_params.q_j;
                        dq_j = obj.mwbm_config.init_state_params.dq_j;
                        g_wf = obj.mwbm_model.g_wf;
                    otherwise
                        error('WBM::setWorldFrameFromFixedLink: %s', WBM.wbmErrorMsg.WRONG_ARG);
                end
            end
            obj.urdfLinkName = urdf_link_name; % replace the old default link with the new link ...

            obj.setState(q_j, dq_j, v_b); % update the robot state (important for initializations) ...
            [w_p_b, w_R_b] = obj.getWorldFrameFromFixedLink(urdf_link_name); % use optimized mode
            obj.setWorldFrame(w_R_b, w_p_b, g_wf);
        end

        function setWorldFrameFromDfltFixedLink(obj, q_j, dq_j, v_b, g_wf)
            if (nargin < 5)
                switch nargin
                    case 4
                        % use the default gravity values ...
                        g_wf = obj.mwbm_model.g_wf;
                    case 1
                        % use the initial state values ...
                        v_b  = vertcat(obj.mwbm_config.init_state_params.dx_b, obj.mwbm_config.init_state_params.omega_b);
                        q_j  = obj.mwbm_config.init_state_params.q_j;
                        dq_j = obj.mwbm_config.init_state_params.dq_j;
                        g_wf = obj.mwbm_model.g_wf;
                    otherwise
                        error('WBM::setWorldFrameFromDfltFixedLink: %s', WBM.wbmErrorMsg.WRONG_ARG);
                end
            end
            obj.setState(q_j, dq_j, v_b); % update state (for initializations, else precautionary) ...
            [w_p_b, w_R_b] = obj.getWorldFrameFromDfltFixedLink(); % optimized mode
            obj.setWorldFrame(w_R_b, w_p_b, g_wf);
        end

        function updateInitRotoTranslation(obj)
            vqT_init = obj.stvqT; % get the vector-quaternion transf. of the current state ...
            obj.mwbm_config.init_state_params.x_b  = vqT_init(1:3,1); % translation/position
            obj.mwbm_config.init_state_params.qt_b = vqT_init(4:7,1); % orientation (quaternion)
        end

        function wf_vqT_rlnk = computeFKinRotoTranslation(obj, urdf_link_name, q_j, vqT, g_wf)
            % calculate the forward kinematic roto-translation of a specified link frame:
            if (nargin < 4)
                error('WBM::computeFKinRotoTranslation: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            % get the roto-translation form the base state ...
            [p_b, R_b] = WBM.utilities.frame2posRotm(vqT);
            % set the world frame to the base ...
            if ~exist('g_wf', 'var')
                obj.setWorldFrame(R_b, p_b); % use the default gravity vector ...
            else
                obj.setWorldFrame(R_b, p_b, g_wf);
            end
            % compute the forward kinematics of the link frame ...
            wf_vqT_rlnk = obj.forwardKinematics(R_b, p_b, q_j, urdf_link_name);
        end

        [dstvChi, h] = forwardDynamics(obj, t, stvChi, ctrlTrqs)

        sim_config = setupSimulation(~, sim_config)

        [] = visualizeForwardDynamics(obj, x_out, sim_config, sim_tstep, vis_ctrl)

        function simulateForwardDynamics(obj, x_out, sim_config, sim_tstep, nRpts, vis_ctrl)
            if ~exist('vis_ctrl', 'var')
                % use the default ctrl-values ...
                for i = 1:nRpts
                    obj.visualizeForwardDynamics(x_out, sim_config, sim_tstep);
                end
                return
            end
            % else ...
            for i = 1:nRpts
                obj.visualizeForwardDynamics(x_out, sim_config, sim_tstep, vis_ctrl);
            end
        end

        function [chn_q, chn_dq] = getStateChains(obj, chain_names, q_j, dq_j)
            switch nargin
                case {2, 4}
                    if isempty(chain_names)
                        error('WBM::getStateChains: %s', WBM.wbmErrorMsg.EMPTY_CELL_ARR);
                    end
                    % check if the body components are defined ...
                    if isempty(obj.mwbm_config.body)
                        error('WBM::getStateChains: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
                    end

                    if (nargin == 2)
                        [~,q_j,~,dq_j] = obj.getState(); % get the current state values ...
                    end

                    len = length(chain_names);
                    if (len > obj.mwbm_config.body.nChains)
                        error('WBM::getStateChains: %s', WBM.wbmErrorMsg.WRONG_ARR_SIZE);
                    end

                    % get the joint angles and velocities of each chain ...
                    ridx = find(ismember(obj.mwbm_config.body.chains(:,1), chain_names));
                    if ( isempty(ridx) || (length(ridx) ~= len) )
                        error('WBM::getStateChains: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
                    end
                    chn_q  = cell(len,1); % chains ...
                    chn_dq = chn_q;

                    for i = 1:len
                        idx = ridx(i); % for each idx of row-idx ...
                        start_idx = obj.mwbm_config.body.chains{idx,2};
                        end_idx   = obj.mwbm_config.body.chains{idx,3};

                        chn_q{i,1}  = q_j(start_idx:end_idx,1);  % joint angles
                        chn_dq{i,1} = dq_j(start_idx:end_idx,1); % joint velocities
                    end
                otherwise
                    error('WBM::getStateChains: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function [jnt_q, jnt_dq] = getStateJointNames(obj, joint_names, q_j, dq_j)
            switch nargin
                case {2, 4}
                    if isempty(joint_names)
                        error('WBM::getStateJointNames: %s', WBM.wbmErrorMsg.EMPTY_CELL_ARR);
                    end
                    % check if the body parts are defined ...
                    if isempty(obj.mwbm_config.body)
                        error('WBM::getStateJointNames: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
                    end

                    if (nargin == 2)
                        [~,q_j,~,dq_j] = obj.getState(); % get the state values ...
                    end
                    len = length(joint_names);

                    % get the row indices ...
                    ridx = find(ismember(obj.mwbm_config.body.joints(:,1), joint_names));
                    if ( isempty(ridx) || (length(ridx) ~= len) )
                        error('WBM::getStateJointNames: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
                    end
                    % get the angles and velocities ...
                    [jnt_q, jnt_dq] = obj.getJointValues(q_j, dq_j, ridx, len);
                otherwise
                    error('WBM::getStateJointNames: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function [jnt_q, jnt_dq] = getStateJointIdx(obj, joint_idx, q_j, dq_j)
            switch nargin
                case {2, 4}
                    % check the index list ...
                    if isempty(joint_idx)
                        error('WBM::getStateJointIdx: %s', WBM.wbmErrorMsg.EMPTY_VECTOR);
                    end
                    if ( ~isvector(joint_idx) && ~isinteger(joint_idx) )
                        error('WBM::getStateJointIdx: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                    end

                    if (nargin == 2)
                        [~,q_j,~,dq_j] = obj.getState(); % get the values ...
                    end
                    len = length(joint_idx);

                    % get the angle and velocity of each joint ...
                    [jnt_q, jnt_dq] = obj.getJointValues(q_j, dq_j, joint_idx, len);
                otherwise
                    error('WBM::getStateJointIdx: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function stParams = getStateParams(obj, stvChi)
            len = obj.mwbm_config.stvLen;
            if ( ~iscolumn(stvChi) || (size(stvChi,1) ~= len) )
               error('WBM::getStateParams: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            ndof     = obj.mwbm_model.ndof;
            stParams = WBM.wbmStateParams;

            % get the base/joint positions and the base orientation ...
            stParams.x_b  = stvChi(1:3,1);
            stParams.qt_b = stvChi(4:7,1);
            stParams.q_j  = stvChi(8:ndof+7,1);
            % the corresponding velocities ...
            stParams.dx_b    = stvChi(ndof+8:ndof+10,1);
            stParams.omega_b = stvChi(ndof+11:ndof+13,1);
            stParams.dq_j    = stvChi(ndof+14:len,1);
        end

        function stParams = getStateParamsData(obj, chi)
            len = obj.mwbm_config.stvLen;

            [m, n] = size(chi);
            if (n ~= len)
                error('WBM::getStateParamsData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            ndof     = obj.mwbm_model.ndof;
            stParams = WBM.wbmStateParams;

            % extract all values ...
            stParams.x_b  = chi(1:m,1:3);
            stParams.qt_b = chi(1:m,4:7);
            stParams.q_j  = chi(1:m,8:ndof+7);

            stParams.dx_b    = chi(1:m,ndof+8:ndof+10);
            stParams.omega_b = chi(1:m,ndof+11:ndof+13);
            stParams.dq_j    = chi(1:m,ndof+14:len);
        end

        function stvPos = getPositions(obj, stvChi)
            if ( ~iscolumn(stvChi) || (size(stvChi,1) ~= obj.mwbm_config.stvLen) )
               error('WBM::getPositions: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            % extract the base VQS-Transformation (without S)
            % and the joint positions ...
            cutp = obj.mwbm_model.ndof + 7;
            stvPos = stvChi(1:cutp,1); % [x_b; qt_b; q_j]
        end

        function stmPos = getPositionsData(obj, chi)
            [m, n] = size(chi);
            if (n ~= obj.mwbm_config.stvLen)
                error('WBM::getPositionsData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            cutp = obj.mwbm_model.ndof + 7;
            stmPos = chi(1:m,1:cutp); % m -by- [x_b, qt_b, q_j]
        end

        function stvVel = getVelocities(obj, stvChi)
            if ( ~iscolumn(stvChi) || (size(stvChi,1) ~= obj.mwbm_config.stvLen) )
               error('WBM::getVelocities: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            len  = obj.mwbm_config.stvLen;
            cutp = obj.mwbm_model.ndof + 8;
            % extract the velocities ...
            stvVel = stvChi(cutp:len,1); % [dx_b; omega_b; dq_j]
        end

        function stmVel = getVelocitiesData(obj, chi)
            [m, n] = size(chi);
            if (n ~= obj.mwbm_config.stvLen)
                error('WBM::getVelocitiesData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            cutp = obj.mwbm_model.ndof + 8;
            stmVel = chi(1:m,cutp:len); % m -by- [dx_b, omega_b, dq_j]
        end

        function stvBsVel = getBaseVelocities(obj, stvChi)
            if ( ~iscolumn(stvChi) || (size(stvChi,1) ~= obj.mwbm_config.stvLen) )
               error('WBM::getBaseVelocities: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            cutp1 = obj.mwbm_model.ndof + 8;
            cutp2 = obj.mwbm_model.ndof + 13;
            stvBsVel = stvChi(cutp1:cutp2,1); % [dx_b; omega_b]
        end

        function stmBsVel = getBaseVelocitiesData(obj, chi)
            [m, n] = size(chi);
            if (n ~= obj.mwbm_config.stvLen)
                error('WBM::getBaseVelocitiesData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            cutp1 = obj.mwbm_model.ndof + 8;
            cutp2 = obj.mwbm_model.ndof + 13;
            stmBsVel = chi(1:m,cutp1:cutp2); % m -by- [dx_b, omega_b]
        end

        function [dx_b, omega_b] = getBaseVelocitiesParams(~, v_b)
            if ( ~iscolumn(v_b) || (size(v_b,1) ~= 6) )
               error('WBM::getBaseVelocitiesParams: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            dx_b    = v_b(1:3,1);
            omega_b = v_b(4:6,1);
        end

        function vqT = getRotoTranslation(~, stParams)
            if isempty(stParams)
                error('WBM::getRotoTranslation: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end

            vqT = vertcat(stParams.x_b, stParams.qt_b);
        end

        function stvChi = toStateVector(~, stParams)
            if isempty(stParams)
                error('WBM::toStateVector: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end

            stvChi = vertcat(stParams.x_b, stParams.qt_b, stParams.q_j, ...
                             stParams.dx_b, stParams.omega_b, stParams.dq_j);
        end

        function setLinkPayloads(obj, link_names, pl_data)
            % verify the input types ...
            if ( ~iscell(link_names) || ~ismatrix(pl_data) )
                error('WBM::setLinkPayloads: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % check dimensions ...
            [m, n] = size(pl_data);
            if (n ~= 4)
                error('WBM::setLinkPayloads: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            if (size(link_names,1) ~= m)
                error('WBM::setLinkPayloads: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end

            obj.mwbm_config.nPlds = m; % number of payloads ...
            obj.mwbm_config.lnk_payloads(1:m,1) = WBM.wbmLinkPayload;
            for i = 1:m
                obj.mwbm_config.lnk_payloads(i,1).urdf_link_name = link_names{i,1};
                obj.mwbm_config.lnk_payloads(i,1).pt_mass        = pl_data(i,1);
                obj.mwbm_config.lnk_payloads(i,1).wf_p_rlnk      = pl_data(i,2:4).';
            end
        end

        function [lnk_plds, nPlds] = getLinkPayloads(obj)
            lnk_plds = obj.mwbm_config.lnk_payloads;
            nPlds    = obj.mwbm_config.nPlds;
        end

        function pl_tbl = getPayloadTable(obj)
            nPlds = obj.mwbm_config.nPlds;
            if (nPlds == 0)
                pl_tbl = table();
                return
            end

            plds  = obj.mwbm_config.lnk_payloads;
            clnk_names = cell(nPlds,1);
            mass       = zeros(nPlds,1);
            cpos       = clnk_names;

            for i = 1:nPlds
                clnk_names{i,1} = plds(i,1).urdf_link_name;
                mass(i,1)       = plds(i,1).pt_mass;
                cpos{i,1}       = plds(i,1).wf_p_rlnk;
            end
            cplds  = horzcat(clnk_names, num2cell(mass), cpos);
            pl_tbl = cell2table(cplds, 'VariableNames', {'link_name', 'mass', 'pos'});
        end

        function stvChi_init = get.stvChiInit(obj)
            stInit = obj.mwbm_config.init_state_params;
            stvChi_init = vertcat(stInit.x_b, stInit.qt_b, stInit.q_j, ...
                                  stInit.dx_b, stInit.omega_b, stInit.dq_j);
        end

        function stvLen = get.stvLen(obj)
            stvLen = obj.mwbm_config.stvLen;
        end

        function vqT_init = get.vqTInit(obj)
            stInit = obj.mwbm_config.init_state_params;
            vqT_init = vertcat(stInit.x_b, stInit.qt_b);
        end

        function stvqT = get.stvqT(obj)
            [stvqT,~,~,~] = obj.getState();
        end

        function set.robot_init_state(obj, stInit)
            if ~obj.checkInitStateDimensions(stInit)
                error('WBM::set.robot_init_state: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end

            obj.mwbm_config.init_state_params = stInit;
        end

        function stInit = get.robot_init_state(obj)
            stInit = obj.mwbm_config.init_state_params;
        end

        function robot_config = get.robot_config(obj)
            robot_config = obj.mwbm_config;
        end

        function robot_body = get.robot_body(obj)
            robot_body = obj.mwbm_config.body;
        end

        function dispWBMConfig(obj, prec)
            if ~exist('prec', 'var')
                prec = 2;
            end
            nPlds  = obj.mwbm_config.nPlds;
            stInit = obj.mwbm_config.init_state_params;

            clnk_names     = [num2cell(1:obj.mwbm_config.nCstrs); obj.mwbm_config.cstr_link_names];
            strLnkNamesLst = sprintf('  %d  %s\n', clnk_names{:});

            cinit_st = cell(6,1);
            cinit_st{1,1} = sprintf('  q_j:      %s', mat2str(stInit.q_j, prec));
            cinit_st{2,1} = sprintf('  dq_j:     %s', mat2str(stInit.dq_j, prec));
            cinit_st{3,1} = sprintf('  x_b:      %s', mat2str(stInit.x_b, prec));
            cinit_st{4,1} = sprintf('  qt_b:     %s', mat2str(stInit.qt_b, prec));
            cinit_st{5,1} = sprintf('  dx_b:     %s', mat2str(stInit.dx_b, prec));
            cinit_st{6,1} = sprintf('  omega_b:  %s', mat2str(stInit.omega_b, prec));
            strInitState    = sprintf('%s\n%s\n%s\n%s\n%s\n%s', cinit_st{1,1}, cinit_st{2,1}, ...
                                      cinit_st{3,1}, cinit_st{4,1}, cinit_st{5,1}, cinit_st{6,1});

            strPldTbl = sprintf('  none\n');
            if (nPlds > 0)
                % print the payload data in table form:
                plds  = obj.mwbm_config.lnk_payloads;

                clnk_names = cell(nPlds,1);
                cmass      = clnk_names;
                cpos       = clnk_names;
                % put the data in cell-arrays ...
                for i = 1:nPlds
                    clnk_names{i,1} = plds(i,1).urdf_link_name;
                    cmass{i,1}      = num2str(plds(i,1).pt_mass, prec);
                    cpos{i,1}       = mat2str(plds(i,1).wf_p_rlnk, prec);
                end
                % get the string lengths and the max. string lengths ...
                slen1 = cellfun('length', clnk_names);
                slen2 = cellfun('length', cmass);
                msl1  = max(slen1);
                msl2  = max(slen2);
                % compute the number of spaces ...
                if (msl1 <= 9) % length('link_name') = 9
                    nspc = 13;       % 9 + 4
                else
                    nspc = msl1 - 5; % msl1 - 9 + 4
                end
                % create the formatted table in string form ...
                strPldTbl = sprintf('  idx   link_name%smass%spos\\n', blanks(nspc), blanks(msl2));
                for i = 1:nPlds
                    nspc_1 = msl1 - slen1(i,1) + 4;
                    nspc_2 = msl2 - slen2(i,1) + 4;
                    str = sprintf('   %d    %s%s%s%s%s\\n', ...
                                  i, clnk_names{i,1}, blanks(nspc_1), cmass{i,1}, ...
                                  blanks(nspc_2), cpos{i,1});
                    strPldTbl = strcat(strPldTbl, str);
                end
                strPldTbl = sprintf(strPldTbl);
            end

            strConfig = sprintf(['Robot Configuration:\n\n' ...
                                 ' #constraints: %d\n' ...
                                 ' constraint link names:\n%s\n' ...
                                 ' initial state:\n%s\n\n' ...
                                 ' #payloads: %d\n' ...
                                 ' link payloads:\n%s'], ...
                                obj.mwbm_config.nCstrs, strLnkNamesLst, ...
                                strInitState, nPlds, strPldTbl);
            disp(strConfig);
        end

    end

    methods(Access = private)
        function initConfig(obj, robot_config)
            % check if robot_config is an instance of a class that
            % is derived from "wbmBaseRobotConfig" ...
            if ~isa(robot_config, 'WBM.wbmBaseRobotConfig')
                error('WBM::initConfig: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % further error checks ...
            if (length(robot_config.cstr_link_names) ~= robot_config.nCstrs)
                error('WBM::initConfig: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            if isempty(robot_config.init_state_params)
                error('WBM::initConfig: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end

            obj.mwbm_config = WBM.wbmBaseRobotConfig;
            obj.mwbm_config.stvLen          = 2*obj.mwbm_model.ndof + 13;
            obj.mwbm_config.cstr_link_names = robot_config.cstr_link_names;
            obj.mwbm_config.nCstrs          = robot_config.nCstrs;

            if ~isempty(robot_config.body)
                obj.mwbm_config.body = robot_config.body;
            end

            if ~WBM.utilities.isStateEmpty(robot_config.init_state_params)
                % check all parameter dimensions in "init_state_params", summed size
                % is either: 0 (= empty), 'stvLen' or 'stvLen-7' ...
                if ~obj.checkInitStateDimensions(robot_config.init_state_params)
                    error('WBM::initConfig: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
                end
                % check the number of joints ...
                if (size(robot_config.init_state_params.q_j,1) > obj.MAX_NUM_JOINTS)
                    error('WBM::initConfig: %s', WBM.wbmErrorMsg.MAX_JOINT_LIMIT);
                end
            end
            obj.mwbm_config.init_state_params = robot_config.init_state_params;
        end

        function [jnt_q, jnt_dq] = getJointValues(obj, q_j, dq_j, joint_idx, len)
            if (len > obj.mwbm_config.body.nJoints)
                error('WBM::getJointValues: %s', WBM.wbmErrorMsg.WRONG_VEC_SIZE);
            end
            % get the joint values of the index list ...
            jnt_q(1:len,1)  = q_j(joint_idx,1);  % angle
            jnt_dq(1:len,1) = dq_j(joint_idx,1); % velocity
        end

        function result = checkInitStateDimensions(obj, stInit)
            len = size(stInit.x_b,1) + size(stInit.qt_b,1) + size(stInit.q_j,1) + ...
                  size(stInit.dx_b,1) + size(stInit.omega_b,1) + size(stInit.dq_j,1);

            if (len ~= obj.mwbm_config.stvLen) % allowed length: 'stvLen' or 'stvLen-7'
                if (len ~= (obj.mwbm_config.stvLen - 7)) % length without x_b & qt_b (they will be updated afterwards)
                    result = false;
                    return
                end
            end
            result = true;
        end

    end
end
