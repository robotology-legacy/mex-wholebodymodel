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

        function wf_vqT_lnk = fkinVQTransformation(obj, urdf_link_name, q_j, vqT, g_wf)
            % computes the forward kinematic vector-quaternion transf. of a specified link frame:
            if (nargin < 4)
                error('WBM::fkinVQTransformation: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            % get the VQ-Transformation form the base state ...
            [p_b, R_b] = WBM.utilities.frame2posRotm(vqT);
            % set the world frame to the base ...
            if ~exist('g_wf', 'var')
                obj.setWorldFrame(R_b, p_b); % use the default gravity vector ...
            else
                obj.setWorldFrame(R_b, p_b, g_wf);
            end
            % compute the forward kinematics of the link frame ...
            wf_vqT_lnk = obj.forwardKinematics(R_b, p_b, q_j, urdf_link_name);
        end

        function [Jc, dJcdq] = contactJacobians(obj, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b)
            nCstrs = obj.mwbm_config.nCstrs; % must be >= 1!

            m = 6*nCstrs;
            n = 6 + obj.mwbm_model.ndof;
            Jc = zeros(m,n);
            dJcdq = zeros(m,1);

            % compute for each contact constraint the Jacobian and the derivative Jacobian:
            switch nargin
                case 6
                    for i = 1:nCstrs
                        cstr_link = obj.mwbm_config.cstr_link_names{1,i};
                        Jc(6*i-5:6*i,1:n)  = mexWholeBodyModel('jacobian', wf_R_b_arr, wf_p_b, q_j, cstr_link); % 6*(i-1)+1 = 6*i-5
                        dJcdq(6*i-5:6*i,1) = mexWholeBodyModel('djdq', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b, cstr_link);
                    end
                case 1
                    for i = 1:nCstrs
                        cstr_link = obj.mwbm_config.cstr_link_names{1,i};
                        Jc(6*i-5:6*i,1:n)  = mexWholeBodyModel('jacobian', cstr_link);
                        dJcdq(6*i-5:6*i,1) = mexWholeBodyModel('djdq', cstr_link);
                    end
                otherwise
                    error('WBM::contactJacobians: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function [ddq_j, acc_data] = jointAccelerations(obj, varargin)
            switch nargin
                case 7 % normal mode:
                    % wf_R_b = varargin{1}
                    wf_p_b = varargin{1,2};
                    q_j          = varargin{1,3};
                    dq_j         = varargin{1,4};
                    v_b          = varargin{1,5};
                    tau          = varargin{1,6};

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    M    = mexWholeBodyModel('mass-matrix', wf_R_b_arr, wf_p_b, q_j);
                    C_qv = mexWholeBodyModel('generalised-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);

                    % compute for each contact constraint the Jacobian and the derivative Jacobian:
                    [Jc, dJcdq] = contactJacobians(obj, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                case 3 % optimized mode:
                    dq_j = varargin{1,1};
                    tau  = varargin{1,2};

                    M    = mexWholeBodyModel('mass-matrix');
                    C_qv = mexWholeBodyModel('generalised-forces');

                    [Jc, dJcdq] = contactJacobians(obj);
                otherwise
                    error('WBM::jointAccelerations: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            % Calculation of the contact (constraint) force vector:
            % Further details about the formula see,
            %   [1] Control Strategies for Robots in Contact, J. Park, PhD-Thesis, Artificial Intelligence Laboratory,
            %       Department of Computer Science, Stanford University, 2006, chapter 5, pp. 106-110, eq. (5.5)-(5.14),
            %       <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>.
            %   [2] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 269-270, eq. (6.5) & (6.6).
            Jc_t      = Jc.';
            JcMinv    = Jc / M; % x*M = Jc --> x = Jc*M^(-1)
            JcMinvJct = JcMinv * Jc_t; % inverse mass matrix in contact space
            tau_fr    = frictionForces(obj, dq_j); % friction torques (negative torque values)
            tau_gen   = vertcat(zeros(6,1), tau + tau_fr); % generalized forces tau_gen = tau + (-tau_fr)
            % contact (constraint) forces f_c:
            f_c = -(JcMinvJct \ (JcMinv*(C_qv - tau_gen) - dJcdq)); % JcMinvJct*f_c = (...) --> f_c = JcMinvJct^(-1)*(...)

            % Joint Acceleration q_ddot (derived from the state-space equation):
            % For further details see:
            %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
            ddq_j = M \ (tau_gen - C_qv - Jc_t*f_c); % ddq_j = M^(-1) * (tau - C_qv - Jc.'*(-f_c))
            %ddq_j = M \ (Jc.'*f_c + tau_gen - C_qv); % cause Jc.'*f_c round-off errors?

            acc_data = struct('f_c', f_c, 'tau', tau, 'tau_gen', tau_gen);
        end

        [ddq_j, acc_data] = jointAccelerationsExt(obj, varargin)

        dstvChi = forwardDynamics(obj, t, stvChi, fhTrqControl)

        dstvChi = forwardDynamicsExt(obj, t, stvChi, fhTrqControl, foot_conf)

        function [t, stmChi] = intForwardDynamics(obj, fhTrqControl, tspan, stvChi_0, ode_opt, foot_conf)
            if ~isa(fhTrqControl, 'function_handle')
                error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE)
            end
            if (obj.mwbm_config.nCstrs == 0)
                error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.VALUE_LTE_ZERO);
            end

            if exist('foot_conf', 'var')
                if ~isstruct(foot_conf)
                    error('WBM::intForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                end
                % use the extended function ...
                fhFwdDyn    = @(t, chi)obj.forwardDynamicsExt(t, chi, fhTrqControl, foot_conf);
                [t, stmChi] = ode15s(fhFwdDyn, tspan, stvChi_0, ode_opt); % ODE-Solver
                return
            end
            % else, use the normal function ...
            fhFwdDyn    = @(t, chi)obj.forwardDynamics(t, chi, fhTrqControl);
            [t, stmChi] = ode15s(fhFwdDyn, tspan, stvChi_0, ode_opt); % ODE-Solver
        end

        function foot_conf = initFootBalanceConfig(obj, varargin)
            switch nargin
                case 7
                    qj_init   = varargin{1,1};
                    veT_lfoot = varargin{1,2};
                    veT_rfoot = varargin{1,3};

                    if ( (size(veT_lfoot,1) ~= 6) || (size(veT_rfoot,1) ~= 6) )
                        error('WBM::initFootBalanceConfig: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                    end
                    foot_conf.veT_init.l_sole = veT_lfoot;
                    foot_conf.veT_init.r_sole = veT_rfoot;

                    feet_on_ground = varargin{1,4};
                    k_p            = varargin{1,5}; % gain k_p with the desired closed-loop stiffness (1)
                    k_v            = varargin{1,6}; % gain k_v with the desired closed-loop damping (2)
                case {4, 5}
                    qj_init        = varargin{1,1};
                    feet_on_ground = varargin{1,2};
                    k_p            = varargin{1,3}; % (1)

                    if (nargin == 5)
                        k_v = varargin{1,4}; % (2)
                    else
                        % set k_v for critical damping (3)
                        % Source: Introduction to Robotics: Mechanics and Control, John J. Craig, 3rd Edition, Pearson/Prentice Hall, 2005,
                        %         p. 274, eq. (9.47).
                        k_v = 2*sqrt(k_p);
                    end
                case 3
                    qj_init        = varargin{1,1};
                    feet_on_ground = varargin{1,2};

                    k_p = obj.DF_STIFFNESS; % use the default stiffness value ...
                    k_v = 2*sqrt(k_p); % (3)
                otherwise
                    error('WBM::initFootBalanceConfig: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            % some error checks ...
            if ( ~isrow(feet_on_ground) || ~islogical(feet_on_ground) )
                error('WBM::initFootBalanceConfig: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if ( ~isreal(k_p) || ~isreal(k_v) ) % no complex values are allowed ...
                error('WBM::initFootBalanceConfig: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end

            %% Setup the configuration structure for the feet:
            % set the correction values for the feet to avoid numerical integration errors:
            foot_conf.ctrl_gains.k_p = k_p; % control gain for correcting the feet positions.
            foot_conf.ctrl_gains.k_v = k_v; % control gain for correcting the velocities.

            % define on which foot the robot is balancing on the ground (max. 3 cases):
            foot_conf.ground.left  = feet_on_ground(1,1);
            foot_conf.ground.right = feet_on_ground(1,2);

            if (nargin == 7)
                return
            end
            % else, get the initial position & orientation of the feet:
            stFltb = getFloatingBaseState(obj);
            wf_R_b_arr = reshape(stFltb.wf_R_b, 9, 1);
            vqT_init_l = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, stFltb.wf_p_b, qj_init, 'l_sole');
            vqT_init_r = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, stFltb.wf_p_b, qj_init, 'r_sole');

            % extract the positions and Euler-angles from the VQ-Transformations:
            [p_init_l, eul_init_l] = WBM.utilities.frame2posEul(vqT_init_l);
            [p_init_r, eul_init_r] = WBM.utilities.frame2posEul(vqT_init_r);

            % transformation vectors (veT - position & Euler-angles):
            foot_conf.veT_init.l_sole = vertcat(p_init_l, eul_init_l);
            foot_conf.veT_init.r_sole = vertcat(p_init_r, eul_init_r);
        end

        vis_data = getFwdDynVisualizationData(obj, stmChi, fhTrqControl, foot_conf)

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
        % function visualizeTrajectoryICubGUI(obj, t, stmChi)
        %     if (nargin < 2)
        %         error('WBM::visualizeTrajectoryICubGUI: %s', WBM.wbmErrorMsg.WRONG_ARG);
        %     end

        %     [m, n] = size(stmChi);
        %     if (n ~= obj.mwbm_config.stvLen)
        %         error('WBM::visualizeTrajectoryICubGUI: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
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
                    wf_H_lnk = mexWholeBodyModel('rototranslation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, pl_link_name);
                case 4
                    % use the values of the default payload-link ...
                    lnk_p_pl = obj.mwbm_config.payload_links(1,1).lnk_p_pl;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_lnk = mexWholeBodyModel('rototranslation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, ...
                                                 obj.mwbm_config.payload_links(1,1).urdf_link_name);
                case 2 % optimized modes:
                    pl_idx       = varargin{1,1};
                    pl_link_name = getLinkName(obj, obj.mwbm_config.payload_links, pl_idx);
                    lnk_p_pl     = obj.mwbm_config.payload_links(pl_idx,1).lnk_p_pl;

                    wf_H_lnk = mexWholeBodyModel('rototranslation-matrix', pl_link_name);
                case 1
                    lnk_p_pl = obj.mwbm_config.payload_links(1,1).lnk_p_pl;
                    wf_H_lnk = mexWholeBodyModel('rototranslation-matrix', obj.mwbm_config.payload_links(1,1).urdf_link_name);
                otherwise
                    error('WBM::payloadFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            % transformation: we assume that the orientation of the payload-frame (pl) has the same
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
                    wf_H_ee = mexWholeBodyModel('rototranslation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, ee_link_name);
                case 4
                    % use the values of the default tool link (1st element of the list) ...
                    ee_vqT_tt = obj.mwbm_config.tool_links(1,1).ee_vqT_tt;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_ee = mexWholeBodyModel('rototranslation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, ...
                                                obj.mwbm_config.tool_links(1,1).urdf_link_name);
                case 2 % optimized modes:
                    t_idx        = varargin{1,1};
                    ee_link_name = getLinkName(obj, obj.mwbm_config.tool_links, t_idx);
                    ee_vqT_tt    = obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt;

                    wf_H_ee = mexWholeBodyModel('rototranslation-matrix', ee_link_name);
                case 1
                    % use the default tool link ...
                    ee_vqT_tt = obj.mwbm_config.tool_links(1,1).ee_vqT_tt;
                    wf_H_ee   = mexWholeBodyModel('rototranslation-matrix', obj.mwbm_config.tool_links(1,1).urdf_link_name);
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

        function J_tt = jacobianTool(obj, varargin) % Jacobian matrix in tool-frame
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
                    wf_H_ee = mexWholeBodyModel('rototranslation-matrix', wf_R_b_arr, wf_p_b, q_j, ee_link_name);
                    J_ee    = mexWholeBodyModel('jacobian', wf_R_b_arr, wf_p_b, q_j, ee_link_name);
                case 4
                    % use the values of the default tool-link (1st element of the list) ...
                    wf_p_b = varargin{1,2};
                    q_j    = varargin{1,3};

                    ee_link_name = obj.mwbm_config.tool_links(1,1).urdf_link_name;
                    ee_vqT_tt    = obj.mwbm_config.tool_links(1,1).ee_vqT_tt;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_ee = mexWholeBodyModel('rototranslation-matrix', wf_R_b_arr, wf_p_b, q_j, ee_link_name);
                    J_ee    = mexWholeBodyModel('jacobian', wf_R_b_arr, wf_p_b, q_j, ee_link_name);
                case 2 % optimized modes:
                    t_idx = varargin{1,1};

                    ee_link_name = obj.mwbm_config.tool_links(t_idx,1).urdf_link_name;
                    ee_vqT_tt    = obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt;

                    wf_H_ee = mexWholeBodyModel('rototranslation-matrix', ee_link_name);
                    J_ee    = mexWholeBodyModel('jacobian', ee_link_name);
                case 1
                    % use the default tool-link ...
                    ee_link_name = obj.mwbm_config.tool_links(1,1).urdf_link_name;
                    ee_vqT_tt    = obj.mwbm_config.tool_links(1,1).ee_vqT_tt;

                    wf_H_ee = mexWholeBodyModel('rototranslation-matrix', ee_link_name);
                    J_ee    = mexWholeBodyModel('jacobian', ee_link_name);
                otherwise
                    error('WBM::jacobianTool: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            %% Velocity transformation matrix:
            %   The transformation matrix X maps the velocities of the geometric Jacobian
            %   in frame {ee} to velocities in frame {tt} and is defined as
            %
            %                        | I    -I*S(wf_R_ee * ee_p_tt) |
            %      tt[wf]_X_ee[wf] = |                              |,
            %                        | 0                 I          |
            %
            %   s.t. J_tt = tt[wf]_X_ee[wf] * ee[wf]_J_ee. The notations tt[wf] and ee[wf]
            %   are denoting frames with origin o_tt or o_ee with orientation [wf].
            %
            % For further details see:
            %   [1] Multibody Dynamics Notation, S. Traversaro & A. Saccon, Eindhoven University of Technology,
            %       Department of Mechanical Engineering, 2016, <http://repository.tue.nl/849895>, p. 6, eq. (27).
            %   [2] Introduction to Robotics: Mechanics and Control, John J. Craig, 3rd Edition, Pearson/Prentice Hall, 2005,
            %       p. 158, eq. (5.100).
            %   [3] Robotics, Vision & Control: Fundamental Algorithms in Matlab, Peter I. Corke, Springer, 2011,
            %       p. 175, eq. (8.4).
            [~, wf_R_ee]    = WBM.utilities.tform2posRotm(wf_H_ee);   % orientation of the end-effector (ee).
            [p_tt, ee_R_tt] = WBM.utilities.frame2posRotm(ee_vqT_tt); % position & orientation of the tool-tip (tt).

            % calculate the position of the tool-tip to the world-frame (wf):
            wf_p_tt = wf_R_ee * (ee_R_tt * p_tt); % = wf_R_ee * ee_p_tt

            % compute the skew-symmetric matrix S(p_tt) ...
            Sp_tt  = WBM.utilities.skew(wf_p_tt);
            % create the velocity transformation matrix:
            tt_X_ee = eye(6,6);
            tt_X_ee(1:3,4:6) = -Sp_tt;

            % compute J_tt by performing velocity addition ...
            J_tt = tt_X_ee * J_ee; % = tt[wf]_X_ee[wf] * ee[wf]_J_ee
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

        function [v_b, dq_j] = getMixedVelocities(obj, stChi)
            len   = obj.mwbm_config.stvLen;
            ndof  = obj.mwbm_model.ndof;

            if iscolumn(stChi)
                if (size(stChi,1) ~= len)
                   error('WBM::getMixedVelocities: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                end

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
                nspc = msl1 - 9 + 4; % length('link_name') = 9

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

        function result = checkInitStateDimensions(obj, stInit) % make this public?
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
