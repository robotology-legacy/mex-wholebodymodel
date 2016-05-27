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

    methods%(Access = public)
        % Constructor:
        function obj = WBM(model_params, robot_config, wf2FixLnk)
            % call the constructor of the superclass ...
            obj = obj@WBM.WBMBase(model_params);

            if ~exist('robot_config', 'var')
                error('WBM::WBM: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            if ~exist('wf2FixLnk', 'var')
                wf2FixLnk = false; % default value ...
            else
                if ~islogical(wf2FixLnk)
                    error('WBM::WBM: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                end
            end

            obj.initConfig(robot_config);
            if wf2FixLnk
                % set the world frame (WF) at the initial roto-translation from
                % the chosen fixed link, i.e. the first entry of the constraint list:
                obj.setWorldFrameFromFixedLink(obj.mwbm_config.cstrLinkNames{1});
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
                        g_wf = obj.mwbm_params.g_wf;
                    case 2
                        % use the initial state values ...
                        v_b  = vertcat(obj.mwbm_config.initStateParams.dx_b, obj.mwbm_config.initStateParams.omega_b);
                        q_j  = obj.mwbm_config.initStateParams.q_j;
                        dq_j = obj.mwbm_config.initStateParams.dq_j;
                        g_wf = obj.mwbm_params.g_wf;
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
                        g_wf = obj.mwbm_params.g_wf;
                    case 1
                        % use the initial state values ...
                        v_b  = vertcat(obj.mwbm_config.initStateParams.dx_b, obj.mwbm_config.initStateParams.omega_b);
                        q_j  = obj.mwbm_config.initStateParams.q_j;
                        dq_j = obj.mwbm_config.initStateParams.dq_j;
                        g_wf = obj.mwbm_params.g_wf;
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
            obj.mwbm_config.initStateParams.x_b  = vqT_init(1:3,1); % translation/position
            obj.mwbm_config.initStateParams.qt_b = vqT_init(4:7,1); % orientation (quaternion)
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

            ndof     = obj.mwbm_config.ndof;
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

            ndof     = obj.mwbm_config.ndof;
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
            cutp = obj.mwbm_config.ndof + 7;
            stvPos = stvChi(1:cutp,1); % [x_b; qt_b; q_j]
        end

        function stmPos = getPositionsData(obj, chi)
            [m, n] = size(chi);
            if (n ~= obj.mwbm_config.stvLen)
                error('WBM::getPositionsData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            cutp = obj.mwbm_config.ndof + 7;
            stmPos = chi(1:m,1:cutp); % m -by- [x_b, qt_b, q_j]
        end

        function stvVel = getVelocities(obj, stvChi)
            if ( ~iscolumn(stvChi) || (size(stvChi,1) ~= obj.mwbm_config.stvLen) )
               error('WBM::getVelocities: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            len  = obj.mwbm_config.stvLen;
            cutp = obj.mwbm_config.ndof + 8;
            % extract the velocities ...
            stvVel = stvChi(cutp:len,1); % [dx_b; omega_b; dq_j]
        end

        function stmVel = getVelocitiesData(obj, chi)
            [m, n] = size(chi);
            if (n ~= obj.mwbm_config.stvLen)
                error('WBM::getVelocitiesData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            cutp = obj.mwbm_config.ndof + 8;
            stmVel = chi(1:m,cutp:len); % m -by- [dx_b, omega_b, dq_j]
        end

        function stvBsVel = getBaseVelocities(obj, stvChi)
            if ( ~iscolumn(stvChi) || (size(stvChi,1) ~= obj.mwbm_config.stvLen) )
               error('WBM::getBaseVelocities: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            cutp1 = obj.mwbm_config.ndof + 8;
            cutp2 = obj.mwbm_config.ndof + 13;
            stvBsVel = stvChi(cutp1:cutp2,1); % [dx_b; omega_b]
        end

        function stmBsVel = getBaseVelocitiesData(obj, chi)
            [m, n] = size(chi);
            if (n ~= obj.mwbm_config.stvLen)
                error('WBM::getBaseVelocitiesData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            cutp1 = obj.mwbm_config.ndof + 8;
            cutp2 = obj.mwbm_config.ndof + 13;
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

        function stvChiInit = get.stvChiInit(obj)
            stInit = obj.mwbm_config.initStateParams;
            stvChiInit = vertcat(stInit.x_b, stInit.qt_b, stInit.q_j, ...
                                 stInit.dx_b, stInit.omega_b, stInit.dq_j);
        end

        function stvLen = get.stvLen(obj)
            stvLen = obj.mwbm_config.stvLen;
        end

        function vqTInit = get.vqTInit(obj)
            stInit = obj.mwbm_config.initStateParams;
            vqTInit = vertcat(stInit.x_b, stInit.qt_b);
        end

        function stvqT = get.stvqT(obj)
            [stvqT,~,~,~] = obj.getState();
        end

        function set.robot_init_state(obj, stInit)
            if ~obj.checkInitStateDimensions(stInit)
                error('WBM::set.robot_init_state: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end

            obj.mwbm_config.initStateParams = stInit;
        end

        function stInit = get.robot_init_state(obj)
            stInit = obj.mwbm_config.initStateParams;
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

            cellLnkNames = [num2cell(1:obj.mwbm_config.nCstrs); obj.mwbm_config.cstrLinkNames];
            strLnkNamesLst = sprintf('  %d  %s\n', cellLnkNames{:});
            stInit = obj.mwbm_config.initStateParams;

            cellInitSt{1} = sprintf('  q_j:      %s', mat2str(stInit.q_j, prec));
            cellInitSt{2} = sprintf('  dq_j:     %s', mat2str(stInit.dq_j, prec));
            cellInitSt{3} = sprintf('  x_b:      %s', mat2str(stInit.x_b, prec));
            cellInitSt{4} = sprintf('  qt_b:     %s', mat2str(stInit.qt_b, prec));
            cellInitSt{5} = sprintf('  dx_b:     %s', mat2str(stInit.dx_b, prec));
            cellInitSt{6} = sprintf('  omega_b:  %s', mat2str(stInit.omega_b, prec));
            strInitState  = sprintf('%s\n%s\n%s\n%s\n%s\n%s', cellInitSt{1}, cellInitSt{2}, ...
                                    cellInitSt{3}, cellInitSt{4}, cellInitSt{5}, cellInitSt{6});

            strConfig = sprintf(['Robot Configuration:\n\n' ...
                                 ' NDOFs:         %d\n' ...
                                 ' # constraints: %d\n\n' ...
                                 ' constraint link names:\n\n%s\n' ...
                                 ' damping coefficient: %f\n\n' ...
                                 ' initial state:\n\n%s\n'], ...
                                obj.mwbm_config.ndof, obj.mwbm_config.nCstrs, ...
                                strLnkNamesLst, obj.mwbm_config.dampCoeff, ...
                                strInitState);
            disp(strConfig);
        end

    end

    methods(Access = private)
        function initConfig(obj, robot_config)
            % check if robot_config is an instance of a class that
            % is derived from "wbmBaseRobotConfig" ...
            if ~isa(robot_config, 'WBM.wbmBaseRobotConfig')
                error('WBM::initWBM: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % further error checks ...
            if (robot_config.ndof == 0)
                error('WBM::initWBM: %s', WBM.wbmErrorMsg.VALUE_IS_ZERO);
            end
            if (robot_config.ndof > obj.MAX_NUM_JOINTS)
                error('WBM::initWBM: %s', WBM.wbmErrorMsg.MAX_NUM_LIMIT);
            end
            if (length(robot_config.cstrLinkNames) ~= robot_config.nCstrs)
                error('WBM::initWBM: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            if isempty(robot_config.initStateParams)
                error('WBM::initWBM: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end

            obj.mwbm_config = WBM.wbmBaseRobotConfig;
            obj.mwbm_config.ndof          = robot_config.ndof;
            obj.mwbm_config.stvLen        = 2*obj.mwbm_config.ndof + 13;

            obj.mwbm_config.cstrLinkNames = robot_config.cstrLinkNames;
            obj.mwbm_config.nCstrs        = robot_config.nCstrs;
            obj.mwbm_config.dampCoeff     = robot_config.dampCoeff;

            if ~isempty(robot_config.body)
                obj.mwbm_config.body = robot_config.body;
            end

            if ~WBM.utilities.isStateEmpty(robot_config.initStateParams)
                % check all parameter dimensions in "initStateParams", summed size
                % is either: 0 (= empty), 'stvLen' or 'stvLen-7' ...
                if ~obj.checkInitStateDimensions(robot_config.initStateParams)
                    error('WBM::initWBM: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
                end
                % check the number of joints ...
                if (size(robot_config.initStateParams.q_j,1) > obj.MAX_NUM_JOINTS)
                    error('WBM::initWBM: %s', WBM.wbmErrorMsg.MAX_JOINT_LIMIT);
                end
            end
            obj.mwbm_config.initStateParams = robot_config.initStateParams;
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
