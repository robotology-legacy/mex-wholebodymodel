classdef WBM < WBM.WBMBase
    properties(Dependent)
        stvChiInit@double vector
        stvLen@uint16     scalar
        vqTInit@double    vector
        stvqT@double      vector
        robot_body@WBM.wbmBody
        robot_config@WBM.wbmBaseRobotConfig
        base_robot_params@WBM.wbmBaseRobotParams
        robot_init_state@WBM.wbmStateParams
    end

    properties(Constant)
        MAX_NUM_TOOLS = 2;
    end

    properties(Access = protected)
        mwbm_config@WBM.wbmBaseRobotConfig
        mwf2fixLnk@logical scalar
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
                obj.mwf2fixLnk = false; % default value ...
            else
                obj.mwf2fixLnk = wf2fixLnk;
            end

            obj.initConfig(robot_config);
            if obj.mwf2fixLnk
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

        [dstvChi, C_qv] = forwardDynamics(obj, t, stvChi, fhCtrlTrqs)

        function [t, stmChi] = intForwardDynamics(obj, fhCtrlTrqs, tspan, stvChi_0, ode_opt)
            if ~isa(fhCtrlTrqs, 'function_handle')
                error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE)
            end
            if (obj.mwbm_config.nCstrs == 0)
                error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.VALUE_IS_ZERO);
            end

            if ~exist('ode_opt', 'var')
                % setup the default error tolerances ...
                ode_opt = odeset('RelTol', 1e-2, 'AbsTol', 1e-4);
            end

            fhFwdDyn    = @(t, chi)obj.forwardDynamics(t, chi, fhCtrlTrqs);
            [t, stmChi] = ode15s(fhFwdDyn, tspan, stvChi_0, ode_opt); % ODE-Solver
        end

        sim_config = setupSimulation(~, sim_config)

        [] = visualizeForwardDynamics(obj, pos_out, sim_config, sim_tstep, vis_ctrl)

        function simulateForwardDynamics(obj, pos_out, sim_config, sim_tstep, nRpts, vis_ctrl)
            if ~exist('vis_ctrl', 'var')
                % use the default ctrl-values ...
                for i = 1:nRpts
                    obj.visualizeForwardDynamics(pos_out, sim_config, sim_tstep);
                end
                return
            end
            % else ...
            for i = 1:nRpts
                obj.visualizeForwardDynamics(pos_out, sim_config, sim_tstep, vis_ctrl);
            end
        end

        function plotCoMTrajectory(obj, stmChi, prop)
            len = obj.mwbm_config.stvLen;

            [m, n] = size(stmChi);
            if (n ~= len)
                error('WBM::plotCoMTrajectory: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            if ~exist('prop', 'var')
                % use the default plot properties ...
                prop.fwnd_title   = 'iCub - CoM-trajectory:';
                prop.title        = '';
                prop.title_fnt_sz = 15;
                prop.line_color   = 'blue';
                prop.marker       = '*';
                prop.mkr_color    = 'red';
                prop.label_fnt_sz = 15;
            end

            % extract all base position values ...
            x_b = stmChi(1:m,1:3);

            figure('Name', prop.fwnd_title, 'NumberTitle', 'off');

            % draw the trajectory-line:
            %         x-axis      y-axis      z-axis
            plot3(x_b(1:m,1), x_b(1:m,2), x_b(1:m,3), 'Color', prop.line_color);
            hold on;
            % mark the start point ...
            plot3(x_b(1,1), x_b(1,2), x_b(1,3), 'Marker', prop.marker, 'MarkerEdgeColor', prop.mkr_color);

            % add title and axis-lables ...
            if ~isempty(prop.title)
                title(prop.title, 'Interpreter', 'latex', 'FontSize', prop.title_fnt_sz);
            end
            xlabel('$x_{\mathbf{x_b}}$', 'Interpreter', 'latex', 'FontSize', prop.label_fnt_sz);
            ylabel('$y_{\mathbf{x_b}}$', 'Interpreter', 'latex', 'FontSize', prop.label_fnt_sz);
            zlabel('$z_{\mathbf{x_b}}$', 'Interpreter', 'latex', 'FontSize', prop.label_fnt_sz);

            grid on;
            axis square;
        end

        % Experimental: method does not work at the moment (Matlab hangs when it calls the mex-function)
        % function visualizeTrajectory_iCubGUI(obj, t, stmChi)
        %     if (nargin < 2)
        %         error('WBM::visualizeTrajectory_iCubGUI: %s', WBM.wbmErrorMsg.WRONG_ARG);
        %     end

        %     [m, n] = size(stmChi);
        %     if (n ~= obj.mwbm_config.stvLen)
        %         error('WBM::visualizeTrajectory_iCubGUI: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
        %     end
        %     cutp = obj.mwbm_model.ndof + 7;
        %     vqT_b = stmChi(1:m,1:7);    % m -by- vqT_b
        %     q_j   = stmChi(1:m,8:cutp); % m -by- q_j

        %     mexWholeBodyModel('visualize-trajectory', t, q_j, vqT_b);
        % end

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
            if (size(link_names,2) ~= m) % the list must be a row-vector ...
                error('WBM::setLinkPayloads: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end

            obj.mwbm_config.nPlds = m; % number of payloads ...
            obj.mwbm_config.payload_links(1:m,1) = WBM.wbmPayloadLink;
            for i = 1:m
                obj.mwbm_config.payload_links(i,1).urdf_link_name = link_names{1,i};
                obj.mwbm_config.payload_links(i,1).pt_mass        = pl_data(i,1);
                obj.mwbm_config.payload_links(i,1).rlnk_p_pl      = pl_data(i,2:4).';
            end
        end

        function [pl_lnks, nPlds] = getPayloadLinks(obj)
            pl_lnks = obj.mwbm_config.payload_links;
            nPlds   = obj.mwbm_config.nPlds;
        end

        function pl_tbl = getPayloadTable(obj)
            nPlds = obj.mwbm_config.nPlds;
            if (nPlds == 0)
                pl_tbl = table();
                return
            end

            pl_lnks    = obj.mwbm_config.payload_links;
            clnk_names = cell(nPlds,1);
            mass       = zeros(nPlds,1);
            cpos       = clnk_names;

            for i = 1:nPlds
                clnk_names{i,1} = pl_lnks(i,1).urdf_link_name;
                mass(i,1)       = pl_lnks(i,1).pt_mass;
                cpos{i,1}       = pl_lnks(i,1).rlnk_p_pld;
            end
            cplds  = horzcat(clnk_names, num2cell(mass), cpos);
            pl_tbl = cell2table(cplds, 'VariableNames', {'link_name', 'mass', 'pos'});
        end

        function wf_H_pl = payloadFrame(obj, varargin)
            if (obj.mwbm_config.nPlds == 0)
                error('WBM::payloadFrame: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end
            % wf_R_rootLnk = varargin{1}
            % wf_p_rootLnk = varargin{2}
            % q_j          = varargin{3}
            switch nargin
                case 5 % normal modes:
                    lnk_idx = varargin{1,4};
                    urdf_link_name = getLinkName(obj, obj.mwbm_config.payload_links, lnk_idx);
                    rlnk_p_pl      = obj.mwbm_config.payload_links(lnk_idx,1).rlnk_p_pl;

                    wf_R_rlnk_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_rlnk = mexWholeBodyModel('rototranslation-matrix', wf_R_rlnk_arr, varargin{1,2}, varargin{1,3}, urdf_link_name);
                case 4
                    % use the values of the default payload link ...
                    rlnk_p_pl = obj.mwbm_config.payload_links(1,1).rlnk_p_pl;

                    wf_R_rlnk_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_rlnk = mexWholeBodyModel('rototranslation-matrix', wf_R_rlnk_arr, varargin{1,2}, varargin{1,3}, ...
                                                  obj.mwbm_config.payload_links(1,1).urdf_link_name);
                case 2 % optimized modes:
                    lnk_idx = varargin{1,1};
                    urdf_link_name = getLinkName(obj, obj.mwbm_config.payload_links, lnk_idx);
                    rlnk_p_pl      = obj.mwbm_config.payload_links(lnk_idx,1).rlnk_p_pl;

                    wf_H_rlnk = mexWholeBodyModel('rototranslation-matrix', urdf_link_name);
                case 1
                    rlnk_p_pl = obj.mwbm_config.payload_links(1,1).rlnk_p_pl;
                    wf_H_rlnk = mexWholeBodyModel('rototranslation-matrix', obj.mwbm_config.tool_links(1,1).urdf_link_name);
                otherwise
                    error('WBM::payloadFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            wf_H_pl = wf_H_rlnk;
            % translation - we assume that both frames (rlnk & pl) have the same orientation:
            %                wf_p_rlnk (= position of the origin of the ref. link)
            wf_H_pl(1:3,4) = wf_H_rlnk(1:3,4) + rlnk_p_pl; % wf_p_pl = wf_p_rlnk + rlnk_p_pl (= pos. of the CoM of the payload)
        end

        % function tau_pl = payloadForces(obj, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)

        % end

        function setToolLinks(obj, link_names, pos_data)
            % verify the input types ...
            if ( ~iscell(link_names) || ~ismatrix(pos_data) )
                error('WBM::setToolLinks: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % check dimensions ...
            [m, n] = size(pos_data);
            if (n ~= 3)
                error('WBM::setToolLinks: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            if (size(link_names,2) ~= m) % the list must be a row-vector ...
                error('WBM::setToolLinks: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            if (m > obj.MAX_NUM_TOOLS)
                error('WBM::setToolLinks: %s', WBM.wbmErrorMsg.MAX_NUM_LIMIT);
            end

            obj.mwbm_config.nTools = m; % number of tools ...
            obj.mwbm_config.tool_links(1:m,1) = WBM.wbmToolLink;
            for i = 1:m
                obj.mwbm_config.tool_links(i,1).urdf_link_name = link_names{1,i};
                obj.mwbm_config.tool_links(i,1).rlnk_p_tp      = pos_data(i,1:3).';
            end
        end

        function [tool_lnks, nTools] = getToolLinks(obj)
            tool_lnks = obj.mwbm_config.tool_links;
            nTools    = obj.mwbm_config.nTools;
        end

        function updateToolLink(obj, lnk_idx, new_pos_tp)
            if (obj.mwbm_config.nTools == 0)
                error('WBM::updateToolLink: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end
            if (lnk_idx > obj.MAX_NUM_TOOLS)
                error('WBM::updateToolLink: %s', WBM.wbmErrorMsg.MAX_NUM_LIMIT);
            end
            if (size(new_pos_tp,1) ~= 3)
                error('WBM::updateToolLink: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            % update the position of the tool-tip with the new position ...
            obj.mwbm_config.tool_links(lnk_idx,1).rlnk_p_tp = new_pos_tp;
        end

        function tool_tbl = getToolTable(obj)
            nTools = obj.mwbm_config.nTools;
            if (nTools == 0)
                tool_tbl = table();
                return
            end

            tool_lnks  = obj.mwbm_config.tool_links;
            clnk_names = cell(nTools,1);
            cpos       = clnk_names;

            for i = 1:nTools
                clnk_names{i,1} = tool_lnks(i,1).urdf_link_name;
                cpos{i,1}       = tool_lnks(i,1).wf_p_rlnk;
            end
            ctools = horzcat(clnk_names, cpos);

            tool_tbl = cell2table(ctools, 'VariableNames', {'link_name', 'pos'});
        end

        function wf_H_tp = toolFrame(obj, varargin) % is this calculation correct?
            if (obj.mwbm_config.nTools == 0)
                error('WBM::toolFrame: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end
            % wf_R_rootLnk = varargin{1}
            % wf_p_rootLnk = varargin{2}
            % q_j          = varargin{3}
            switch nargin
                case 5 % normal modes:
                    lnk_idx = varargin{1,4};
                    urdf_link_name = getLinkName(obj, obj.mwbm_config.tool_links, lnk_idx);
                    rlnk_p_tp      = obj.mwbm_config.tool_links(lnk_idx,1).rlnk_p_tp;

                    wf_R_rlnk_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_rlnk = mexWholeBodyModel('rototranslation-matrix', wf_R_rlnk_arr, varargin{1,2}, varargin{1,3}, urdf_link_name);
                case 4
                    % use the values of the default tool link ...
                    rlnk_p_tp = obj.mwbm_config.payload_links(1,1).rlnk_p_tp;

                    wf_R_rlnk_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_rlnk = mexWholeBodyModel('rototranslation-matrix', wf_R_rlnk_arr, varargin{1,2}, varargin{1,3}, ...
                                                  obj.mwbm_config.tool_links(1,1).urdf_link_name);
                case 2 % optimized modes:
                    lnk_idx = varargin{1,1};
                    urdf_link_name = getLinkName(obj, obj.mwbm_config.payload_links, lnk_idx);
                    rlnk_p_tp      = obj.mwbm_config.tool_links(lnk_idx,1).rlnk_p_tp;

                    wf_H_rlnk = mexWholeBodyModel('rototranslation-matrix', urdf_link_name);
                case 1
                    rlnk_p_tp = obj.mwbm_config.payload_links(1,1).rlnk_p_tp;
                    wf_H_rlnk = mexWholeBodyModel('rototranslation-matrix', obj.mwbm_config.tool_links(1,1).urdf_link_name);
                otherwise
                    error('WBM::getToolFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            wf_H_tp = wf_H_rlnk;
            % translation - we assume that both frames (rlnk & tp) have the same orientation:
            %                wf_p_rlnk (= position of the origin of the ref. link)
            wf_H_tp(1:3,4) = wf_H_rlnk(1:3,4) + rlnk_p_tp; % wf_p_tp = wf_p_rlnk + rlnk_p_tp (= pos. of the tooltip)
        end

        function J_tp = jacobianTool(obj, wf_R_rootLnk, wf_p_rootLnk, q_j, lnk_idx) % Jacobian matrix in tool frame (end-effector frame)
            % is this kind of calculation correct? (not sure)
            switch nargin
                case 5
                    urdf_link_name = obj.mwbm_config.tool_links(lnk_idx,1).urdf_link_name;
                    wf_H_tp = obj.toolFrame(wf_R_rootLnk, wf_p_rootLnk, q_j, lnk_idx); % transformation matrix
                case 4
                    % use the default tool link (1st element of the list) ...
                    urdf_link_name = obj.mwbm_config.tool_links(1,1).urdf_link_name;
                    wf_H_tp = obj.toolFrame(wf_R_rootLnk, wf_p_rootLnk, q_j);
                otherwise
                    error('WBM::jacobianTool: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            % get the orientation and the translation of the tool-tip ...
            [p_tp, R_tp] = WBM.utilities.tform2posRotm(wf_H_tp);
            % compute the jacobian of the tool-tip:
            R_tp_arr = reshape(R_tp, 9, 1);
            J_tp = mexWholeBodyModel('jacobian', R_tp_arr, p_tp, q_j, urdf_link_name);
        end

        function [chn_q, chn_dq] = getStateChains(obj, chain_names, q_j, dq_j)
            switch nargin
                case {2, 4}
                    if isempty(chain_names)
                        error('WBM::getStateChains: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
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
                        error('WBM::getStateJointNames: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
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

        function stParams = getStateParams(obj, stChi)
            len      = obj.mwbm_config.stvLen;
            ndof     = obj.mwbm_model.ndof;
            stParams = WBM.wbmStateParams;

            if iscolumn(stChi)
                if (size(stChi,1) ~= len)
                   error('WBM::getStateParams: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                end

                % get the base/joint positions and the base orientation ...
                stParams.x_b  = stChi(1:3,1);
                stParams.qt_b = stChi(4:7,1);
                stParams.q_j  = stChi(8:ndof+7,1);
                % the corresponding velocities ...
                stParams.dx_b    = stChi(ndof+8:ndof+10,1);
                stParams.omega_b = stChi(ndof+11:ndof+13,1);
                stParams.dq_j    = stChi(ndof+14:len,1);
                return
            elseif ismatrix(stChi)
                [m, n] = size(stChi);
                if (n ~= len)
                    error('WBM::getStateParams: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end

                % extract all values ...
                stParams.x_b  = stChi(1:m,1:3);
                stParams.qt_b = stChi(1:m,4:7);
                stParams.q_j  = stChi(1:m,8:ndof+7);

                stParams.dx_b    = stChi(1:m,ndof+8:ndof+10);
                stParams.omega_b = stChi(1:m,ndof+11:ndof+13);
                stParams.dq_j    = stChi(1:m,ndof+14:len);
                return
            end
            % else ...
            error('WBM::getStateParams: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end

        function [vqT_b, q_j] = getPositions(obj, stChi)
            len  = obj.mwbm_config.stvLen;
            cutp = obj.mwbm_model.ndof + 7; % 3 + 4 + ndof

            if iscolumn(stChi)
                if (size(stChi,1) ~= len)
                   error('WBM::getPositions: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                end

                % extract the base VQS-Transformation (without S)
                % and the joint positions ...
                vqT_b = stChi(1:7,1); % [x_b; qt_b]
                q_j   = stChi(8:cutp,1);
                return
            elseif ismatrix(stChi)
                [m, n] = size(stChi);
                if (n ~= len)
                    error('WBM::getPositions: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end

                vqT_b = stChi(1:m,1:7);    % m -by- [x_b, qt_b]
                q_j   = stChi(1:m,8:cutp); % m -by- q_j
                return
            end
            % else ...
            error('WBM::getPositions: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end

        function stmPos = getPositionsData(obj, stmChi)
            [m, n] = size(stmChi);
            if (n ~= obj.mwbm_config.stvLen)
                error('WBM::getPositionsData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            cutp   = obj.mwbm_model.ndof + 7; % 3 + 4 + ndof
            stmPos = stmChi(1:m,1:cutp);      % m -by- [x_b, qt_b, q_j]
        end

        function [v_b, dq_j] = getVelocities(obj, stChi)
            len   = obj.mwbm_config.stvLen;
            ndof  = obj.mwbm_model.ndof;

            if iscolumn(stChi)
                if (size(stChi,1) ~= len)
                   error('WBM::getVelocities: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                end

                % extract the velocities ...
                v_b  = stChi(ndof+8:ndof+13,1); % [dx_b; omega_b]
                dq_j = stChi(ndof+14:len,1);
                return
            elseif ismatrix(stChi)
                [m, n] = size(stChi);
                if (n ~= len)
                    error('WBM::getVelocities: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end

                v_b  = stChi(1:m,ndof+8:ndof+13); % m -by- [dx_b; omega_b]
                dq_j = stChi(1:m,ndof+14:len,1); % m -by- dq_j
                return
            end
            % else ...
            error('WBM::getVelocities: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end

        function v_b = getBaseVelocities(obj, stChi)
            len   = obj.mwbm_config.stvLen;
            ndof  = obj.mwbm_model.ndof;

            if iscolumn(stChi)
                if (size(stChi,1) ~= len)
                   error('WBM::getBaseVelocities: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                end

                v_b = stChi(ndof+8:ndof+13,1); % [dx_b; omega_b]
                return
            elseif ismatrix(stChi)
                [m, n] = size(stChi);
                if (n ~= len)
                    error('WBM::getBaseVelocities: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end

                v_b = stChi(1:m,ndof+8:ndof+13); % m -by- [dx_b; omega_b]
                return
            end
            % else ...
            error('WBM::getBaseVelocities: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end

        function stvChi = get.stvChiInit(obj)
            stInit = obj.mwbm_config.init_state_params;
            stvChi = vertcat(stInit.x_b, stInit.qt_b, stInit.q_j, ...
                             stInit.dx_b, stInit.omega_b, stInit.dq_j);
        end

        function stvLen = get.stvLen(obj)
            stvLen = obj.mwbm_config.stvLen;
        end

        function vqT_b = get.vqTInit(obj)
            stInit = obj.mwbm_config.init_state_params;
            vqT_b  = vertcat(stInit.x_b, stInit.qt_b);
        end

        function vqT_b = get.stvqT(obj)
            [vqT_b,~,~,~] = getState(obj);
        end

        function robot_body = get.robot_body(obj)
            robot_body = obj.mwbm_config.body;
        end

        function robot_config = get.robot_config(obj)
            robot_config = obj.mwbm_config;
        end

        function base_params = get.base_robot_params(obj)
            base_params = WBM.wbmBaseRobotParams;
            base_params.robot_model  = obj.mwbm_model;
            base_params.robot_config = obj.mwbm_config;
            base_params.wf2fixLnk    = obj.mwf2fixLnk;
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

        function dispConfig(obj, prec)
            if ~exist('prec', 'var')
                prec = 2;
            end
            nPlds  = obj.mwbm_config.nPlds;
            nCstrs = obj.mwbm_config.nCstrs;
            stInit = obj.mwbm_config.init_state_params;

            clnk_names     = vertcat(num2cell(1:nCstrs), obj.mwbm_config.cstr_link_names);
            strLnkNamesLst = sprintf('  %d  %s\n', clnk_names{1:2,1:nCstrs});

            cinit_st = cell(6,1);
            cinit_st{1,1} = sprintf('  q_j:      %s', mat2str(stInit.q_j, prec));
            cinit_st{2,1} = sprintf('  dq_j:     %s', mat2str(stInit.dq_j, prec));
            cinit_st{3,1} = sprintf('  x_b:      %s', mat2str(stInit.x_b, prec));
            cinit_st{4,1} = sprintf('  qt_b:     %s', mat2str(stInit.qt_b, prec));
            cinit_st{5,1} = sprintf('  dx_b:     %s', mat2str(stInit.dx_b, prec));
            cinit_st{6,1} = sprintf('  omega_b:  %s', mat2str(stInit.omega_b, prec));
            strInitState  = sprintf('%s\n%s\n%s\n%s\n%s\n%s', cinit_st{1,1}, cinit_st{2,1}, ...
                                    cinit_st{3,1}, cinit_st{4,1}, cinit_st{5,1}, cinit_st{6,1});

            strPldTbl = sprintf('  none\n');
            if (nPlds > 0)
                % print the payload data in table form:
                pl_lnks  = obj.mwbm_config.payload_links;

                clnk_names = cell(nPlds,1);
                cmass      = clnk_names;
                cpos       = clnk_names;
                % put the data in cell-arrays ...
                for i = 1:nPlds
                    clnk_names{i,1} = pl_lnks(i,1).urdf_link_name;
                    cmass{i,1}      = num2str(pl_lnks(i,1).pt_mass, prec);
                    cpos{i,1}       = mat2str(pl_lnks(i,1).rlnk_p_pl, prec);
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
            nCstrs = size(robot_config.cstr_link_names,2);
            if ( ~isrow(robot_config.cstr_link_names) || ...
                 ((robot_config.nCstrs > 0) && (nCstrs ~= robot_config.nCstrs)) )
                error('WBM::initConfig: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            if isempty(robot_config.init_state_params)
                error('WBM::initConfig: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end

            obj.mwbm_config = WBM.wbmBaseRobotConfig;
            obj.mwbm_config.stvLen          = 2*obj.mwbm_model.ndof + 13;
            obj.mwbm_config.nCstrs          = nCstrs;
            obj.mwbm_config.cstr_link_names = robot_config.cstr_link_names;

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

        function lnk_name = getLinkName(~, lnk_list, idx)
            % check range ...
            if ( (idx > size(lnk_list,1)) || (idx < 1) )
                error('WBM::getLinkName: %s', WBM.wbmErrorMsg.IDX_OUT_OF_BOUNDS);
            end

            lnk_name = lnk_list(idx,1).urdf_link_name;
        end

    end
end
