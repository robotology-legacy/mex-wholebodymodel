classdef WBM < WBM.WBMBase
    properties(Dependent)
        stvChiInit@double vector
        stvLen@uint16     scalar
        vqTInit@double    vector
        stvqT@double      vector
        robot_body@WBM.wbmBody
        robot_config@WBM.wbmBaseRobotConfig
        robot_params@WBM.wbmBaseRobotParams
        init_state@WBM.wbmStateParams
    end

    properties(Constant)
        DF_STIFFNESS  = 2.5; % default control gain for the position correction.
        MAX_NUM_TOOLS = 2;
        % zero contact acceleration vectors:
        ZERO_CTC_ACC_12 = zeros(12,1);
        ZERO_CTC_ACC_6  = zeros(6,1);
        % zero external force vectors:
        ZERO_EX_FORCE_12 = zeros(12,1);
        ZERO_EX_FORCE_6  = zeros(6,1);
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
                if (obj.mwbm_config.nCstrs > 0)
                    % set the world frame (WF) at the initial VQ-Transformation from
                    % the chosen fixed link, i.e. the first entry of the constraint list:
                    obj.setWorldFrameAtFixLnk(obj.mwbm_config.cstr_link_names{1});
                else
                    error('WBM::WBM: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
                end
            end
            % retrieve and update the initial VQ-Transformation of the robot base (world frame) ...
            obj.updateInitVQTransformation();
        end

        % Copy-function:
        function newObj = copy(obj)
            newObj = copy@WBM.WBMBase(obj);
        end

        % Destructor:
        function delete(obj)
            delete@WBM.WBMBase(obj);
        end

        function setWorldFrameAtFixLnk(obj, urdf_fixed_link, q_j, dq_j, v_b, g_wf)
            if (nargin < 6)
                switch nargin
                    case 5
                        % use the default gravity vector ...
                        g_wf = obj.mwbm_model.g_wf;
                    case 2
                        % use the initial state values (possibly changed from outside) ...
                        v_b  = vertcat(obj.mwbm_config.init_state_params.dx_b, obj.mwbm_config.init_state_params.omega_b);
                        q_j  = obj.mwbm_config.init_state_params.q_j;
                        dq_j = obj.mwbm_config.init_state_params.dq_j;
                        g_wf = obj.mwbm_model.g_wf;
                    otherwise
                        error('WBM::setWorldFrameAtFixLnk: %s', WBM.wbmErrorMsg.WRONG_ARG);
                end
            end
            obj.fixed_link = urdf_fixed_link; % replace the old default fixed link with the new fixed link ...

            obj.setState(q_j, dq_j, v_b); % update the robot state (important for initializations) ...
            [p_b, R_b] = obj.getWorldFrameFromFixLnk(urdf_fixed_link); % use optimized mode
            obj.setWorldFrame(R_b, p_b, g_wf);
        end

        function updateWorldFrameFromFixLnk(obj, q_j, dq_j, v_b, g_wf)
            if (nargin < 5)
                switch nargin
                    case 4
                        % use the default gravity values ...
                        g_wf = obj.mwbm_model.g_wf;
                    case 1
                        % use the initial state values (possibly changed from outside) ...
                        v_b  = vertcat(obj.mwbm_config.init_state_params.dx_b, obj.mwbm_config.init_state_params.omega_b);
                        q_j  = obj.mwbm_config.init_state_params.q_j;
                        dq_j = obj.mwbm_config.init_state_params.dq_j;
                        g_wf = obj.mwbm_model.g_wf;
                    otherwise
                        error('WBM::updateWorldFrameFromFixLnk: %s', WBM.wbmErrorMsg.WRONG_ARG);
                end
            end
            obj.setState(q_j, dq_j, v_b); % update state ...
            [p_b, R_b] = obj.getWorldFrameFromDfltFixLnk(); % optimized mode
            obj.setWorldFrame(R_b, p_b, g_wf); % update the world frame with the new values ...
        end

        function updateInitVQTransformation(obj)
            vqT_init = obj.stvqT; % get the vector-quaternion transf. of the current state ...
            obj.mwbm_config.init_state_params.x_b  = vqT_init(1:3,1); % translation/position
            obj.mwbm_config.init_state_params.qt_b = vqT_init(4:7,1); % orientation (quaternion)
        end

        function vqT_lnk = fkinVQTransformation(obj, urdf_link_name, q_j, vqT_b, g_wf)
            % computes the forward kinematic vector-quaternion transf. of a specified link frame:
            if (nargin < 4)
                error('WBM::fkinVQTransformation: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            % get the VQ-Transformation form the base state ...
            [p_b, R_b] = WBM.utilities.frame2posRotm(vqT_b);
            % set the world frame to the base ...
            if ~exist('g_wf', 'var')
                obj.setWorldFrame(R_b, p_b); % use the default gravity vector ...
            else
                obj.setWorldFrame(R_b, p_b, g_wf);
            end
            % compute the forward kinematics of the link frame ...
            vqT_lnk = obj.forwardKinematics(R_b, p_b, q_j, urdf_link_name);
        end

        function [Jc, djcdq] = contactJacobians(obj, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b, idx_list)
            switch nargin
                case {2, 7}
                    % use only specific contact constraints ...
                    if isrow(idx_list)
                        WBM.utilities.checkNumListAscOrder(idx_list, 'WBM::contactJacobians');
                        nCstrs = size(idx_list,2);
                    elseif isscalar(idx_list)
                        nCstrs = 1;
                    else
                        error('contactJacobians: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                    end
                case {1, 6}
                    % use the all contact constraints ...
                    nCstrs   = obj.mwbm_config.nCstrs;
                    idx_list = 1:nCstrs;
                otherwise
                    error('WBM::contactJacobians: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            n = obj.mwbm_model.ndof + 6;

            if (nCstrs == 0)
                Jc = zeros(6,n);
                djcdq = zeros(6,1);
                return
            end
            % else ...
            m = 6*nCstrs;
            Jc = zeros(m,n);
            djcdq = zeros(m,1);

            % compute for each contact constraint the Jacobian and the derivative Jacobian:
            if (nargin > 2)
                % normal mode:
                for i = 1:nCstrs
                    idx = idx_list(1,i);
                    cstr_link = obj.mwbm_config.cstr_link_names{1,idx};

                    Jc(6*i-5:6*i,1:n)  = mexWholeBodyModel('jacobian', wf_R_b_arr, wf_p_b, q_j, cstr_link); % 6*(i-1)+1 = 6*i-5
                    djcdq(6*i-5:6*i,1) = mexWholeBodyModel('dJdq', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b, cstr_link);
                end
            else
                % optimized mode:
                for i = 1:nCstrs
                    idx = idx_list(1,i);
                    cstr_link = obj.mwbm_config.cstr_link_names{1,idx};

                    Jc(6*i-5:6*i,1:n)  = mexWholeBodyModel('jacobian', cstr_link);
                    djcdq(6*i-5:6*i,1) = mexWholeBodyModel('dJdq', cstr_link);
                end
            end
        end

        function [f_c, tau_gen] = contactForces(obj, tau, Jc, djcdq, M, c_qv, dq_j)
            switch nargin
                case 7
                    % generalized forces with friction:
                    tau_fr  = frictionForces(obj, dq_j);         % friction torques (negated torque values)
                    tau_gen = vertcat(zeros(6,1), tau + tau_fr); % generalized forces tau_gen = S_j*(tau + (-tau_fr)),
                                                                 % S_j = [0_(6xn); I_(nxn)] ... joint selection matrix
                case 6
                    % general case:
                    tau_gen = vertcat(zeros(6,1), tau);
                otherwise
                    error('WBM::jointAccelerations: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            % Calculation of the contact (constraint) force vector:
            % For further details about the formula see,
            %   [1] Control Strategies for Robots in Contact, J. Park, PhD-Thesis, Artificial Intelligence Laboratory,
            %       Department of Computer Science, Stanford University, 2006, chapter 5, pp. 106-110, eq. (5.5)-(5.14),
            %       <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>.
            %   [2] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 269-270, eq. (6.5) & (6.6).
            Jc_t      = Jc.';
            JcMinv    = Jc / M; % x*M = Jc --> x = Jc*M^(-1)
            Upsilon_c = JcMinv * Jc_t; % inverse mass matrix in contact space Upsilon_c = (Jc * M^(-1) * Jc^T) ... (= "inverse pseudo-kinetic energy matrix"?)
            % contact constraint forces f_c (generated by the environment):
            f_c = Upsilon_c \ (JcMinv*(c_qv - tau_gen) - djcdq); % Upsilon_c*f_c = (...) --> f_c = Upsilon_c^(-1)*(...)
        end

        [f_c, tau_gen] = contactForcesCLPC(obj, clink_conf, tau, f_e, a_c, Jc, djcdq, M, c_qv, varargin) % CLPC ... contact link pose correction

        function [ddq_j, fd_prms] = jointAccelerations(obj, tau, varargin)
            switch nargin
                case 7 % normal modes:
                    % generalized forces with friction:
                    % wf_R_b = varargin{1}
                    wf_p_b = varargin{1,2};
                    q_j    = varargin{1,3};
                    dq_j   = varargin{1,4};
                    v_b    = varargin{1,5};

                    % compute the whole body dynamics and for each contact constraint
                    % the Jacobian and the derivative Jacobian ...
                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    [M, c_qv, Jc, djcdq] = rigidBodyDynCJacobians(obj, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                    % get the contact forces and the corresponding generalized forces ...
                    [f_c, tau_gen] = contactForces(obj, tau, Jc, djcdq, M, c_qv, dq_j);
                case 6
                    % general case:
                    % wf_R_b = varargin{1}
                    wf_p_b = varargin{1,2};
                    q_j    = varargin{1,3};
                    nu     = varargin{1,4};

                    len  = obj.mwbm_model.ndof + 6;
                    dq_j = nu(7:len,1);
                    v_b  = nu(1:6,1);

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    [M, c_qv, Jc, djcdq] = rigidBodyDynCJacobians(obj, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                    [f_c, tau_gen] = contactForces(obj, tau, Jc, djcdq, M, c_qv);
                case 3 % optimized modes:
                    % with friction:
                    dq_j = varargin{1,1};

                    [M, c_qv, Jc, djcdq] = rigidBodyDynCJacobians(obj);
                    [f_c, tau_gen] = contactForces(obj, tau, Jc, djcdq, M, c_qv, dq_j);
                case 2
                    % general case:
                    [M, c_qv, Jc, djcdq] = rigidBodyDynCJacobians(obj);
                    [f_c, tau_gen] = contactForces(obj, tau, Jc, djcdq, M, c_qv);
                otherwise
                    error('WBM::jointAccelerations: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            % Joint Acceleration q_ddot (derived from the state-space equation):
            % For further details see:
            %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
            Jc_t  = Jc.';
            ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv); % ddq_j = M^(-1) * (tau - c_qv - Jc.'*(-f_c))

            % forward dynamics parameters ...
            fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c);
        end

        [ddq_j, fd_prms] = jointAccelerationsCLPC(obj, clink_conf, tau, f_e, a_c, varargin)   % CLPC ... contact link pose correction

        function [ddq_j, fd_prms] = jointAccelerationsFPC(obj, feet_conf, tau, ac_f, varargin) % FPC  ... feet pose correction
            fe_0 = zeroExtForces(obj, feet_conf);
            [ddq_j, fd_prms] = jointAccelerationsCLPC(obj, feet_conf, tau, fe_0, ac_f, varargin{:});
        end

        function [ddq_j, fd_prms] = jointAccelerationsHPC(obj, hand_conf, tau, fe_h, ac_h, varargin) % HPC  ... hand pose correction
            [ddq_j, fd_prms] = jointAccelerationsCLPC(obj, hand_conf, tau, fe_h, ac_h, varargin{:});
        end

        [ddq_j, fd_prms] = jointAccelerationsFHPC(obj, feet_conf, hand_conf, tau, fe_h, ac_f, varargin) % FHPC  ... feet & hand pose correction

        dstvChi = forwardDynamics(obj, t, stvChi, fhTrqControl)

        dstvChi = forwardDynamicsFPC(obj, t, stvChi, fhTrqControl, feet_conf, ac_f)

        dstvChi = forwardDynamicsHPC(obj, t, stvChi, fhTrqControl, hand_conf, fe_h, ac_h)

        dstvChi = forwardDynamicsFHPC(obj, t, stvChi, fhTrqControl, feet_conf, hand_conf, fe_h, ac_f)

        function [t, stmChi] = intForwardDynamics(obj, fhTrqControl, tspan, stvChi_0, ode_opt, feet_conf, ac_f)
            if ~isa(fhTrqControl, 'function_handle')
                error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE)
            end
            if (obj.mwbm_config.nCstrs == 0)
                error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.VALUE_LTE_ZERO);
            end

            switch nargin
                case 7
                    % use the extended function with the feet pose correction:
                    if ~isstruct(feet_conf)
                        error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                    end
                    fhFwdDyn    = @(t, chi)obj.forwardDynamicsFPC(t, chi, fhTrqControl, feet_conf, ac_f);
                    [t, stmChi] = ode15s(fhFwdDyn, tspan, stvChi_0, ode_opt); % ODE-Solver
                case 5
                    % use the standard function (without pose correction):
                    fhFwdDyn    = @(t, chi)obj.forwardDynamics(t, chi, fhTrqControl);
                    [t, stmChi] = ode15s(fhFwdDyn, tspan, stvChi_0, ode_opt); % ODE-Solver
                otherwise
                    error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function ac_0 = zeroCtcAcc(obj, clink_conf)
            nctc = uint8(clink_conf.contact.left) + uint8(clink_conf.contact.right);
            switch nctc
                case 2
                    ac_0 = obj.ZERO_CTC_ACC_12;
                case 1
                    ac_0 = obj.ZERO_CTC_ACC_6;
                case 0
                    error('WBM::zeroCtcAcc: %s', WBM.wbmErrorMsg.NO_LNK_IN_CTC);
            end
        end

        function fe_0 = zeroExtForces(obj, clink_conf)
            nctc = uint8(clink_conf.contact.left) + uint8(clink_conf.contact.right);
            switch nctc
                case 2
                    fe_0 = obj.ZERO_EX_FORCE_12;
                case 1
                    fe_0 = obj.ZERO_EX_FORCE_6;
                case 0
                    error('WBM::zeroExtForces: %s', WBM.wbmErrorMsg.NO_LNK_IN_CTC);
            end
        end

        clink_conf = setCLinkConfigState(obj, clnk_idx_l, clnk_idx_r, varargin)

        function feet_conf = setFeetConfigState(obj, varargin)
            lfoot_idx = find(strcmp(obj.mwbm_config.cstr_link_names, 'l_sole'));
            rfoot_idx = find(strcmp(obj.mwbm_config.cstr_link_names, 'r_sole'));

            if ( isempty(lfoot_idx) || isempty(rfoot_idx) )
                error('WBM::setFeetConfigState: %s', WBM.wbmErrorMsg.LNK_NOT_IN_LIST);
            end

            feet_conf = obj.setCLinkConfigState(lfoot_idx, rfoot_idx, varargin{:});
        end

        function hand_conf = setHandConfigState(obj, varargin)
            lhand_idx = find(strcmp(obj.mwbm_config.cstr_link_names, 'l_hand'));
            rhand_idx = find(strcmp(obj.mwbm_config.cstr_link_names, 'r_hand'));

            if ( isempty(lhand_idx) || isempty(rhand_idx) )
                error('WBM::setHandConfigState: %s', WBM.wbmErrorMsg.LNK_NOT_IN_LIST);
            end

            hand_conf = obj.setCLinkConfigState(lhand_idx, rhand_idx, varargin{:});
        end

        vis_data = getFDynVisualizationData(obj, stmChi, fhTrqControl, varargin)

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

            axis square;
            grid on;

            % add title and axis-lables ...
            if ~isempty(prop.title)
                title(prop.title, 'Interpreter', 'latex', 'FontSize', prop.title_fnt_sz);
            end
            xlabel('$x_{\mathbf{x_b}}$', 'Interpreter', 'latex', 'FontSize', prop.label_fnt_sz);
            ylabel('$y_{\mathbf{x_b}}$', 'Interpreter', 'latex', 'FontSize', prop.label_fnt_sz);
            zlabel('$z_{\mathbf{x_b}}$', 'Interpreter', 'latex', 'FontSize', prop.label_fnt_sz);
        end

        function setPayloadLinks(obj, link_names, pl_data)
            % verify the input types ...
            if ( ~iscell(link_names) || ~ismatrix(pl_data) )
                error('WBM::setPayloadLinks: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % check dimensions ...
            [m, n] = size(pl_data);
            if (n ~= 4)
                error('WBM::setPayloadLinks: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            if (size(link_names,2) ~= m) % the list must be a row-vector ...
                error('WBM::setPayloadLinks: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end

            obj.mwbm_config.nPlds = m; % number of payloads ...
            obj.mwbm_config.payload_links(1:m,1) = WBM.wbmPayloadLink;
            for i = 1:m
                obj.mwbm_config.payload_links(i,1).urdf_link_name = link_names{1,i};
                obj.mwbm_config.payload_links(i,1).pt_mass        = pl_data(i,1);
                obj.mwbm_config.payload_links(i,1).lnk_p_pl       = pl_data(i,2:4).';
            end
        end

        function [pl_links, nPlds] = getPayloadLinks(obj)
            pl_links = obj.mwbm_config.payload_links;
            nPlds    = obj.mwbm_config.nPlds;
        end

        function pl_tbl = getPayloadTable(obj)
            nPlds = obj.mwbm_config.nPlds;
            if (nPlds == 0)
                pl_tbl = table(); % empty table ...
                return
            end

            pl_links   = obj.mwbm_config.payload_links;
            clnk_names = cell(nPlds,1);
            mass       = zeros(nPlds,1);
            cpos       = clnk_names;

            for i = 1:nPlds
                clnk_names{i,1} = pl_links(i,1).urdf_link_name;
                mass(i,1)       = pl_links(i,1).pt_mass;
                cpos{i,1}       = pl_links(i,1).lnk_p_pl;
            end
            cplds  = horzcat(clnk_names, num2cell(mass), cpos);
            pl_tbl = cell2table(cplds, 'VariableNames', {'link_name', 'mass', 'pos'});
        end

        function wf_H_pl = payloadFrame(obj, varargin)
            if (obj.mwbm_config.nPlds == 0)
                error('WBM::payloadFrame: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            switch nargin
                case 5 % normal modes:
                    pl_idx       = varargin{1,4};
                    pl_link_name = getLinkName(obj, obj.mwbm_config.payload_links, pl_idx);
                    lnk_p_pl     = obj.mwbm_config.payload_links(pl_idx,1).lnk_p_pl;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_lnk = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, pl_link_name);
                case 4
                    % use the values of the default payload-link ...
                    lnk_p_pl = obj.mwbm_config.payload_links(1,1).lnk_p_pl;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_lnk = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, ...
                                                 obj.mwbm_config.payload_links(1,1).urdf_link_name);
                case 2 % optimized modes:
                    pl_idx       = varargin{1,1};
                    pl_link_name = getLinkName(obj, obj.mwbm_config.payload_links, pl_idx);
                    lnk_p_pl     = obj.mwbm_config.payload_links(pl_idx,1).lnk_p_pl;

                    wf_H_lnk = mexWholeBodyModel('transformation-matrix', pl_link_name);
                case 1
                    lnk_p_pl = obj.mwbm_config.payload_links(1,1).lnk_p_pl;
                    wf_H_lnk = mexWholeBodyModel('transformation-matrix', obj.mwbm_config.payload_links(1,1).urdf_link_name);
                otherwise
                    error('WBM::payloadFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            % Transformation: We assume that the orientation of the payload-frame (pl) has the same
            %                 orientation as the frame of the given link (lnk), e.g. the link of a
            %                 hand, torso, leg, etc.
            %
            % get the homog. transformation of the payload-frame (relative to the link-frame):
            lnk_H_pl = eye(4,4);
            lnk_H_pl(1:3,4) = lnk_p_pl; % position of the CoM of the payload

            wf_H_pl = wf_H_lnk * lnk_H_pl; % payload transformation matrix
        end

        % Note: The payload forces cannot be calculated, since the mex-subroutine of the whole body model for
        %       YARP-based robots does not support this at the moment. Moreover, an added payload to a link
        %       would affect the complete dynamics (inertia, Coriolis, gravity term, etc.) of the robot.
        %       To include this case in the subroutine either the complete dynamics must be reimplemented,
        %       or new dynamic-functions must be added that treating also the payload-term.
        %
        %       To overcome this problem the calculation with additional payloads on specific links can
        %       be approximated by adding the payload directly to the mass of the desired link in the
        %       corresponding URDF-file.
        %
        % function tau_pl = payloadForces(obj, wf_R_b, wf_p_b, q_j, dq_j, v_b)

        % end

        function setToolLinks(obj, ee_link_names, frames_tt)
            % verify the input types ...
            if ( ~iscell(ee_link_names) || ~ismatrix(frames_tt) )
                error('WBM::setToolLinks: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % check dimensions ...
            [m, n] = size(frames_tt);
            if (m ~= 7)
                error('WBM::setToolLinks: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            if (size(ee_link_names,2) ~= n) % the list must be a row-vector ...
                error('WBM::setToolLinks: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            if (n > obj.MAX_NUM_TOOLS)
                error('WBM::setToolLinks: %s', WBM.wbmErrorMsg.MAX_NUM_LIMIT);
            end

            obj.mwbm_config.nTools = n; % number of tools ...
            obj.mwbm_config.tool_links(1:n,1) = WBM.wbmToolLink;
            for i = 1:n
                obj.mwbm_config.tool_links(i,1).urdf_link_name = ee_link_names{1,i};
                obj.mwbm_config.tool_links(i,1).ee_vqT_tt      = frames_tt(1:7,i);
            end
        end

        function [tool_links, nTools] = getToolLinks(obj)
            tool_links = obj.mwbm_config.tool_links;
            nTools     = obj.mwbm_config.nTools;
        end

        function tool_tbl = getToolTable(obj)
            nTools = obj.mwbm_config.nTools;
            if (nTools == 0)
                tool_tbl = table(); % empty table ...
                return
            end

            tool_links = obj.mwbm_config.tool_links;
            clnk_names = cell(nTools,1);
            cfrms      = clnk_names;

            for i = 1:nTools
                clnk_names{i,1} = tool_links(i,1).urdf_link_name;
                cfrms{i,1}      = tool_links(i,1).ee_vqT_tt;
            end
            ctools = horzcat(clnk_names, cfrms);

            tool_tbl = cell2table(ctools, 'VariableNames', {'link_name', 'frame_tt'});
        end

        function updateToolFrame(obj, t_idx, new_frm_tt)
            if (obj.mwbm_config.nTools == 0)
                error('WBM::updateToolFrame: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end
            if (t_idx > obj.MAX_NUM_TOOLS)
                error('WBM::updateToolFrame: %s', WBM.wbmErrorMsg.MAX_NUM_LIMIT);
            end
            if (size(new_frm_tt,1) ~= 7)
                error('WBM::updateToolFrame: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            % update the tool-frame (VQ-Transformation) of the selected tool with the new frame ...
            obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt = new_frm_tt;
        end

        function wf_H_tt = toolFrame(obj, varargin)
            if (obj.mwbm_config.nTools == 0)
                error('WBM::toolFrame: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end

            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            switch nargin
                case 5 % normal modes:
                    t_idx        = varargin{1,4};
                    ee_link_name = getLinkName(obj, obj.mwbm_config.tool_links, t_idx);
                    ee_vqT_tt    = obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_ee = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, ee_link_name);
                case 4
                    % use the values of the default tool link (1st element of the list) ...
                    ee_vqT_tt = obj.mwbm_config.tool_links(1,1).ee_vqT_tt;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_ee = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, ...
                                                obj.mwbm_config.tool_links(1,1).urdf_link_name);
                case 2 % optimized modes:
                    t_idx        = varargin{1,1};
                    ee_link_name = getLinkName(obj, obj.mwbm_config.tool_links, t_idx);
                    ee_vqT_tt    = obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt;

                    wf_H_ee = mexWholeBodyModel('transformation-matrix', ee_link_name);
                case 1
                    % use the default tool link ...
                    ee_vqT_tt = obj.mwbm_config.tool_links(1,1).ee_vqT_tt;
                    wf_H_ee   = mexWholeBodyModel('transformation-matrix', obj.mwbm_config.tool_links(1,1).urdf_link_name);
                otherwise
                    error('WBM::toolFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            % transformation: we assume the general case, that the orientation of the tool-frame (tt)
            %                 does not have the same orientation as the frame of the end-effector (ee),
            %                 i.e. the frame of a hand (or of a finger).
            %
            % get the homog. transformation of the tool-frame (relative to the ee-frame):
            ee_H_tt = WBM.utilities.frame2tform(ee_vqT_tt);
            wf_H_tt = wf_H_ee * ee_H_tt; % tool transformation matrix
        end

        function wf_J_tt = jacobianTool(obj, varargin) % Jacobian matrix in tool-frame
            if (obj.mwbm_config.nTools == 0)
                error('WBM::jacobianTool: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end

            % wf_R_b = varargin{1}
            switch nargin
                case 5 % normal modes:
                    wf_p_b = varargin{1,2};
                    q_j    = varargin{1,3};
                    t_idx  = varargin{1,4};

                    ee_link_name = obj.mwbm_config.tool_links(t_idx,1).urdf_link_name;
                    ee_vqT_tt    = obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_ee = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, wf_p_b, q_j, ee_link_name);
                    wf_J_ee = mexWholeBodyModel('jacobian', wf_R_b_arr, wf_p_b, q_j, ee_link_name);
                case 4
                    % use the values of the default tool-link (1st element of the list) ...
                    wf_p_b = varargin{1,2};
                    q_j    = varargin{1,3};

                    ee_link_name = obj.mwbm_config.tool_links(1,1).urdf_link_name;
                    ee_vqT_tt    = obj.mwbm_config.tool_links(1,1).ee_vqT_tt;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_ee = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, wf_p_b, q_j, ee_link_name);
                    wf_J_ee = mexWholeBodyModel('jacobian', wf_R_b_arr, wf_p_b, q_j, ee_link_name);
                case 2 % optimized modes:
                    t_idx = varargin{1,1};

                    ee_link_name = obj.mwbm_config.tool_links(t_idx,1).urdf_link_name;
                    ee_vqT_tt    = obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt;

                    wf_H_ee = mexWholeBodyModel('transformation-matrix', ee_link_name);
                    wf_J_ee = mexWholeBodyModel('jacobian', ee_link_name);
                case 1
                    % use the default tool-link ...
                    ee_link_name = obj.mwbm_config.tool_links(1,1).urdf_link_name;
                    ee_vqT_tt    = obj.mwbm_config.tool_links(1,1).ee_vqT_tt;

                    wf_H_ee = mexWholeBodyModel('transformation-matrix', ee_link_name);
                    wf_J_ee = mexWholeBodyModel('jacobian', ee_link_name);
                otherwise
                    error('WBM::jacobianTool: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            %% Velocity transformation matrix:
            %   The transformation matrix X maps the velocities of the geometric Jacobian
            %   in frame {ee} to velocities in frame {tt} and is defined as
            %
            %                        | I    -S(wf_R_ee * ee_p_tt)*I |
            %      tt[wf]_X_ee[wf] = |                              |,
            %                        | 0                 I          |
            %
            %   s.t. wf_J_tt = tt[wf]_X_ee[wf] * ee[wf]_J_ee. The notations tt[wf] and ee[wf]
            %   are denoting frames with origin o_tt and o_ee with orientation [wf].
            %
            % For further details see:
            %   [1] Multibody Dynamics Notation, S. Traversaro & A. Saccon, Eindhoven University of Technology,
            %       Department of Mechanical Engineering, 2016, <http://repository.tue.nl/849895>, p. 6, eq. (27).
            %   [2] Robotics: Modelling, Planning and Control, B. Siciliano & L. Sciavicco & L. Villani & G. Oriolo,
            %       Springer, 2010, p. 150, eq. (3.112).
            %   [3] Introduction to Robotics: Mechanics and Control, John J. Craig, 3rd Edition, Pearson/Prentice Hall, 2005,
            %       p. 158, eq. (5.103).
            [~, wf_R_ee]    = WBM.utilities.tform2posRotm(wf_H_ee);   % orientation of the end-effector (ee).
            [p_tt, ee_R_tt] = WBM.utilities.frame2posRotm(ee_vqT_tt); % position & orientation of the tool-tip (tt).

            % calculate the position from the tool-tip (tt) to the world-frame (wf) ...
            wf_p_tt = wf_R_ee * (ee_R_tt * p_tt); % = wf_R_ee * ee_p_tt
            % get the velocity transformation matrix ...
            tt_X_wf = WBM.utilities.adjoint(wf_p_tt);

            % compute wf_J_tt by performing velocity addition ...
            wf_J_tt = tt_X_wf * wf_J_ee; % = tt[wf]_X_ee[wf] * ee[wf]_J_ee
        end

        function [chn_q, chn_dq] = getStateJntChains(obj, chain_names, q_j, dq_j)
            switch nargin
                case {2, 4}
                    if isempty(chain_names)
                        error('WBM::getJntChainsState: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
                    end
                    % check if the body components are defined ...
                    if isempty(obj.mwbm_config.body)
                        error('WBM::getJntChainsState: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
                    end

                    if (nargin == 2)
                        [~,q_j,~,dq_j] = obj.getState(); % get the current state values ...
                    end

                    len = length(chain_names);
                    if (len > obj.mwbm_config.body.nChains)
                        error('WBM::getJntChainsState: %s', WBM.wbmErrorMsg.WRONG_ARR_SIZE);
                    end

                    % get the joint angles and velocities of each chain ...
                    ridx = find(ismember(obj.mwbm_config.body.chains(:,1), chain_names));
                    if ( isempty(ridx) || (length(ridx) ~= len) )
                        error('WBM::getJntChainsState: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
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
                    error('WBM::getJntChainsState: %s', WBM.wbmErrorMsg.WRONG_ARG);
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
                WBM.utilities.checkCVecDim(stChi, len, 'WBM::getStateParams');
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
                WBM.utilities.checkCVecDim(stChi, len, 'WBM::getPositions');
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

        function [v_b, dq_j] = getMixedVelocities(obj, stChi)
            len   = obj.mwbm_config.stvLen;
            ndof  = obj.mwbm_model.ndof;

            if iscolumn(stChi)
                WBM.utilities.checkCVecDim(stChi, len, 'WBM::getMixedVelocities');
                % extract the velocities ...
                v_b  = stChi(ndof+8:ndof+13,1); % [dx_b; omega_b]
                dq_j = stChi(ndof+14:len,1);
                return
            elseif ismatrix(stChi)
                [m, n] = size(stChi);
                if (n ~= len)
                    error('WBM::getMixedVelocities: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end
                v_b  = stChi(1:m,ndof+8:ndof+13); % m -by- [dx_b; omega_b]
                dq_j = stChi(1:m,ndof+14:len,1);  % m -by- dq_j
                return
            end
            % else ...
            error('WBM::getMixedVelocities: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end

        function v_b = getBaseVelocities(obj, stChi)
            len   = obj.mwbm_config.stvLen;
            ndof  = obj.mwbm_model.ndof;

            if iscolumn(stChi)
                WBM.utilities.checkCVecDim(stChi, len, 'WBM::getBaseVelocities');

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

        function robot_params = get.robot_params(obj)
            robot_params = WBM.wbmBaseRobotParams;
            robot_params.model     = obj.mwbm_model;
            robot_params.config    = obj.mwbm_config;
            robot_params.wf2fixLnk = obj.mwf2fixLnk;
        end

        function set.init_state(obj, stInit)
            if ~obj.checkInitStateDimensions(stInit)
                error('WBM::set.init_state: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            obj.mwbm_config.init_state_params = stInit;
        end

        function stInit = get.init_state(obj)
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
                    cpos{i,1}       = mat2str(pl_lnks(i,1).lnk_p_pl, prec);
                end
                % get the string lengths and the max. string lengths ...
                slen1 = cellfun('length', clnk_names);
                slen2 = cellfun('length', cmass);
                msl1  = max(slen1);
                msl2  = max(slen2);
                % compute the number of spaces ...
                nspc = msl1 - 9 + 5; % length('link_name') = 9

                % create the formatted table in string form ...
                strPldTbl = sprintf('  idx   link_name%smass%spos\\n', blanks(nspc), blanks(msl2-1));
                for i = 1:nPlds
                    nspc_1 = msl1 - slen1(i,1) + 5;
                    nspc_2 = msl2 - slen2(i,1) + 3;
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
            fprintf('%s\n', strConfig);
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
            nCstrs = robot_config.nCstrs; % by default 0, when value is not given ...
            if (nCstrs > 0)
                if (nCstrs ~= size(robot_config.cstr_link_names,2))
                    % the list is not a row vector or the sizes are different ...
                    error('WBM::initConfig: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
                end
            else
                % the length is not given, try to get it ...
                nCstrs = size(robot_config.cstr_link_names,2);
            end

            if isempty(robot_config.init_state_params)
                error('WBM::initConfig: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end

            obj.mwbm_config = WBM.wbmBaseRobotConfig;
            obj.mwbm_config.nCstrs          = nCstrs;
            obj.mwbm_config.cstr_link_names = robot_config.cstr_link_names;

            if ~isempty(robot_config.body)
                obj.mwbm_config.body = robot_config.body;
            end

            if ~WBM.utilities.isStateEmpty(robot_config.init_state_params)
                if (obj.mwbm_model.ndof > 0)
                    obj.mwbm_config.stvLen = 2*obj.mwbm_model.ndof + 13;
                else
                    % the DoF is unknown or is not set --> use the vector length ...
                    vlen = size(robot_config.init_state_params.q_j,1);
                    obj.mwbm_config.stvLen = 2*vlen + 13;
                end

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

        function [M, c_qv, Jc, djcdq] = rigidBodyDynCJacobiansCL(obj, clink_conf, varargin)
            ctc_l = clink_conf.contact.left;
            ctc_r = clink_conf.contact.right;
            % wf_R_b_arr = varargin{1}
            % wf_p_b     = varargin{2}
            % q_j        = varargin{3}
            % dq_j       = varargin{4}
            % v_b        = varargin{5}

            % check which link is in contact with the ground/object and calculate the
            % rigid-body dynamics and corresponding the contact Jacobians:
            if (ctc_l && ctc_r)
                % both links have contact with the ground/object:
                idx_list = horzcat(clink_conf.lnk_idx_l, clink_conf.lnk_idx_r);
                [M, c_qv, Jc, djcdq] = rigidBodyDynCJacobians(obj, varargin{1,1}, varargin{1,2}, varargin{1,3}, ...
                                                              varargin{1,4}, varargin{1,5}, idx_list);
            elseif ctc_l
                % only the left link has contact with the ground/object:
                [M, c_qv, Jc, djcdq] = rigidBodyDynCJacobians(obj, varargin{1,1}, varargin{1,2}, varargin{1,3}, ...
                                                              varargin{1,4}, varargin{1,5}, clink_conf.lnk_idx_l);
            elseif ctc_r
                % only the right link has contact with the ground/object:
                [M, c_qv, Jc, djcdq] = rigidBodyDynCJacobians(obj, varargin{1,1}, varargin{1,2}, varargin{1,3}, ...
                                                              varargin{1,4}, varargin{1,5}, clink_conf.lnk_idx_r);
            else
                % both links have no contact to the ground/object ...
                error('WBM::rigidBodyDynCJacobiansCL: %s', WBM.wbmErrorMsg.NO_LNK_IN_CTC); % only temporary ...
            end
        end

        function [M, c_qv, Jc, djcdq] = rigidBodyDynCJacobians(obj, varargin)
            switch nargin
                case 7 % normal modes:
                    % wf_R_b_arr = varargin{1}
                    % wf_p_b     = varargin{2}
                    % q_j        = varargin{3}
                    % dq_j       = varargin{4}
                    % v_b        = varargin{5}
                    % idx_list   = varargin{6}
                    [M, c_qv]   = rigidBodyDyn(obj, varargin{1,1}, varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5});
                    [Jc, djcdq] = contactJacobians(obj, varargin{1,1}, varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5}, varargin{1,6});
                case 6
                    [M, c_qv]   = rigidBodyDyn(obj, varargin{1,1}, varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5});
                    [Jc, djcdq] = contactJacobians(obj, varargin{1,1}, varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5});
                case 2 % optimized modes:
                    % idx_list = varargin{1}
                    [M, c_qv]   = rigidBodyDyn(obj);
                    [Jc, djcdq] = contactJacobians(obj, varargin{1,1});
                case 1
                    [M, c_qv]   = rigidBodyDyn(obj);
                    [Jc, djcdq] = contactJacobians(obj);
                otherwise
                    error('WBM::rigidBodyDynCJacobians: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function [M, c_qv] = rigidBodyDyn(~, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b)
            switch nargin
                case 6 % normal mode:
                    M    = mexWholeBodyModel('mass-matrix', wf_R_b_arr, wf_p_b, q_j);
                    c_qv = mexWholeBodyModel('generalized-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                case 1 % optimized mode:
                    M    = mexWholeBodyModel('mass-matrix');
                    c_qv = mexWholeBodyModel('generalized-forces');
                otherwise
                    error('WBM::rigidBodyDyn: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

    end
end
