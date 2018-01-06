classdef iCubWBM < WBM.Interfaces.IWBM
    properties(Dependent)
        % public properties for fast get/set methods:
        robot_name@char  = 'iCub';
        robot_model@char = '';
        robot_manuf@char = 'Istituto Italiano di Tecnologia (IIT) - Genoa, Italy.';
        robot_params@WBM.wbmRobotParams
        sim_config@WBM.wbmSimConfig
        base_link@char
        base_tform@double matrix
        foot_conf@struct
        hand_conf@struct
        gravity@double    vector
        jlimits@struct
        ndof@uint16       scalar
        hwbm@WBM.WBM
    end

    properties(Access = protected)
        mwbm_icub@WBM.WBM
        mrobot_name@char  = '';
        mrobot_model@char = '';
        mrobot_manuf@char = '';
        msim_config@WBM.wbmSimConfig
        mbase_tform@double matrix = eye(4,4);
        mtool_tform@cell   vector = {};
        mfoot_conf@struct
        mhand_conf@struct
    end

    methods
        % Constructor:
        function obj = iCubWBM(robot_model, robot_config, wf2fixlnk)
            switch nargin
                % initialize the mex-WholeBodyModel for the iCub robot:
                case 3
                    obj.mwbm_icub = WBM.WBM(robot_model, robot_config, wf2fixlnk);
                case 2
                    obj.mwbm_icub = WBM.WBM(robot_model, robot_config);
                otherwise
                    error('iCubWBM::iCubWBM: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            setToolTform(obj);
        end

        function initRobot(obj, robot_wbm)
            if ~isa(robot_wbm, 'WBM.WBM')
                error('iCubWBM::initRobot: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.mwbm_icub = copy(robot_wbm);
            setToolTform(obj);
        end

        function initRobotFcn(obj, fhInitRobotWBM, wf2fixlnk)
            if ~isa(fhInitRobotWBM, 'function_handle')
                error('iCubWBM::initRobotFcn: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE)
            end

            if ~exist('wf2fixlnk', 'var')
                wf2fixlnk = true;
            end
            obj.mwbm_icub = fhInitRobotWBM(wf2fixlnk);
            setToolTform(obj);
        end

        function initRobotParams(obj, robot_params)
            if ~isa(robot_params, 'WBM.wbmRobotParams')
                error('iCubWBM::initRobotParams: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            rob_params    = copy(robot_params); % copy the robot parameter data ...
            obj.mwbm_icub = WBM.WBM(rob_params.model, rob_params.config, rob_params.wf2fixlnk);
            setToolTform(obj);
        end

        function newObj = copy(obj)
            newObj = WBM.utilities.copyObj(obj);
        end

        function delete(obj)
            delete(obj);
        end

        function [vqT_b, q_j, v_b, dq_j] = getState(obj)
            [vqT_b, q_j, v_b, dq_j] = getState(obj.mwbm_icub);
        end

        function stFltb = getBaseState(obj)
            stFltb = getFloatingBaseState(obj.mwbm_icub);
        end

        function foot_conf = footConfig(obj, cstate, varargin)
            foot_conf = footConfigState(obj.mwbm_icub, cstate, varargin{:});
        end

        function hand_conf = handConfig(obj, cstate, cmode, varargin)
            hand_conf = handConfigState(obj.mwbm_icub, cstate, cmode, varargin{:});
        end

        function ddq_j = jointAcc(obj, tau, q_j, dq_j, stFltb)
            if (nargin == 4)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            ddq_j = jointAccelerations(obj.mwbm_icub, tau, stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function ddq_j = jointAccEF(obj, tau, fe_h, ac_h, ac_f, q_j, dq_j, stFltb) % EF ... External Forces at the hands (no pose corrections)
            if (nargin == 7)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            ddq_j = jointAccelerationsEF(obj.mwbm_icub, obj.mfoot_conf, obj.mhand_conf, tau, fe_h, ac_h, ac_f, ...
                                         stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function ddq_j = jointAccPL(obj, tau, fhTotCWrench, f_cp, ac_f, q_j, dq_j, stFltb) % PL ... PayLoad at the hands (no pose corrections)
            if (nargin == 7)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            ddq_j = jointAccelerationsPL(obj.mwbm_icub, obj.mfoot_conf, obj.mhand_conf, tau, fhTotCWrench, f_cp, ac_f, ...
                                         stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function ddq_j = jointAccFPC(obj, tau, ac_f, q_j, dq_j, stFltb) % FPC ... Foot Pose Corrections (no external forces)
            if (nargin == 5)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            ddq_j = jointAccelerationsFPC(obj.mwbm_icub, obj.mfoot_conf, tau, ac_f, stFltb.wf_R_b, ...
                                          stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function ddq_j = jointAccFPCEF(obj, tau, fe_h, ac_h, ac_f, q_j, dq_j, stFltb) % FPCEF ... Foot Pose Corrections with External Forces (at the hands)
            if (nargin == 7)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            ddq_j = jointAccelerationsFPCEF(obj.mwbm_icub, obj.mfoot_conf, obj.mhand_conf, tau, fe_h, ac_h, ac_f, ...
                                            stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function ddq_j = jointAccFPCPL(obj, tau, fhTotCWrench, f_cp, ac_f, q_j, dq_j, stFltb) % FPCPL ... Foot Pose Corrections with PayLoad (at the hands)
            if (nargin == 7)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            ddq_j = jointAccelerationsFPCPL(obj.mwbm_icub, obj.mfoot_conf, obj.mhand_conf, tau, fhTotCWrench, f_cp, ac_f, ...
                                            stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function ac_h = handAcc(obj, tau, q_j, dq_j, stFltb)
            if (nargin == 4)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            nu   = vertcat(stFltb.v_b, dq_j); % mixed generalized velocity
            ac_0 = zeroCtcAcc(obj.mwbm_icub, obj.mfoot_conf);
            ac_h = handAccelerations(obj.mwbm_icub, obj.mfoot_conf, obj.mhand_conf, tau, ac_0, ...
                                     stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b, nu);
        end

        function vc_h = handVel(obj, q_j, dq_j, stFltb)
            if (nargin == 3)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            vc_h = handVelocities(obj.mwbm_icub, obj.mhand_conf, stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function c_qv = corForces(obj, q_j, dq_j, stFltb)
            if (nargin == 3)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            c_qv = coriolisBiasForces(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function tau_fr = frictForces(obj, dq_j)
            tau_fr = frictionForces(obj.mwbm_icub, dq_j);
        end

        function c_qv = genBiasForces(obj, q_j, dq_j, stFltb)
            if (nargin == 3)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            c_qv = generalizedBiasForces(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function tau_gen = genForces(obj, Je_t, f_e, q_j, dq_j, stFltb)
            if (nargin == 5)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            tau_gen = generalizedForces(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b, Je_t, f_e);
        end

        function g_q = gravForces(obj, q_j, stFltb)
            if (nargin == 2)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            g_q = gravityForces(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j);
        end

        function tau_j = invDyn(obj, q_j, dq_j, ddq_j, dv_b, stFltb)
            if (nargin == 5)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            tau_j = inverseDynamics(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, ...
                                    stFltb.v_b, ddq_j, dv_b);
        end

        function tau_j = invHybridDyn(obj, q_j, dq_j, ddq_j, stFltb)
            if (nargin == 4)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            tau_j = inverseDynamics(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, ...
                                    stFltb.v_b, ddq_j);
        end

        function [t, stmChi] = fwdDyn(obj, tspan, stvChi_0, fhTrqControl, ode_opt, varargin)
            f_cfg = ~isempty(obj.mfoot_conf);
            h_cfg = ~isempty(obj.mhand_conf);

            if (f_cfg && h_cfg)
                % feet and hands:
                % nargin = 11: pc_type = varargin{4}
                if isa(varargin{1,1}, 'function_handle')
                    % with payload:
                    % fhTotCWrench = varargin{1}
                    % f_cp         = varargin{2}
                    % ac_f         = varargin{3}
                    [t, stmChi] = intForwardDynamics(obj.mwbm_icub, tspan, stvChi_0, fhTrqControl, ode_opt, varargin{1,1}, ...
                                                     obj.mfoot_conf, obj.mhand_conf, varargin{2:end});
                else
                    % with external forces:
                    % fe_c = varargin{1}
                    % ac   = varargin{2}
                    % ac_f = varargin{3}
                    [t, stmChi] = intForwardDynamics(obj.mwbm_icub, tspan, stvChi_0, fhTrqControl, ode_opt, obj.mfoot_conf, ...
                                                     obj.mhand_conf, varargin{1:end});
                end
            elseif h_cfg
                % only hands:
                % fe_h      = varargin{1}
                % ac_h      = varargin{2}
                % pc_type   = varargin{3}
                [t, stmChi] = intForwardDynamics(obj.mwbm_icub, tspan, stvChi_0, fhTrqControl, ode_opt, ...
                                                 obj.mhand_conf, varargin{1:3});
            elseif f_cfg
                % only feet:
                % ac_f      = varargin{1}
                % pc_type   = varargin{2}
                [t, stmChi] = intForwardDynamics(obj.mwbm_icub, tspan, stvChi_0, fhTrqControl, ode_opt, ...
                                                 obj.mfoot_conf, varargin{1,1}, varargin{1,2});
            else
                switch nargin
                    case 5
                        % forward dynamics without any pose corrections
                        % and contact link configurations (*):
                        [t, stmChi] = intForwardDynamics(obj.mwbm_icub, tspan, stvChi_0, fhTrqControl, ode_opt);
                    case 6
                        % (*):
                        % pc_type = varargin{1}
                        [t, stmChi] = intForwardDynamics(obj.mwbm_icub, tspan, stvChi_0, fhTrqControl, ode_opt, varargin{1,1});
                    otherwise % nargin > 6:
                        % mixed forward dynamics:
                        % the forward dynamics model is not constant and
                        % changes during the simulation time (e.g. grabbing
                        % an object in between) ...
                        [t, stmChi] = intForwardDynamics(obj.mwbm_icub, tspan, stvChi_0, fhTrqControl, ode_opt, varargin{:});
                end
            end
        end

        function visFwdDyn(obj, stmChi, sim_tstep, vis_ctrl)
            stmPos = getPositionsData(obj.mwbm_icub, stmChi);
            switch nargin
                case 4
                    visualizeForwardDynamics(obj.mwbm_icub, stmPos, obj.msim_config, sim_tstep, vis_ctrl);
                case 3
                    % use the default visualization control values ...
                    visualizeForwardDynamics(obj.mwbm_icub, stmPos, obj.msim_config, sim_tstep);
                otherwise
                    error('iCubWBM::visFwdDyn: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function wf_H_lnk = fwdKin(obj, lnk_name, q_j, stFltb)
            if (nargin == 3)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            vqT_lnk  = forwardKinematics(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, lnk_name);
            wf_H_lnk = WBM.utilities.tfms.frame2tform(vqT_lnk);
        end

        function wf_H_lnk = linkFrame(obj, lnk_name, q_j, stFltb) % link transformation matrix
            if (nargin == 3)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            wf_H_lnk = transformationMatrix(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, lnk_name);
        end

        function wf_H_tt = toolFrame(obj, t_idx, q_j, stFltb) % tool-tip transformation matrix
            if (nargin == 3)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            wf_H_tt = toolFrame(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, t_idx);
        end

        function wf_H_cm = ploadFrame(obj, pl_idx, q_j, stFltb) % transformation matrix of the payload's CoM
            if (nargin == 3)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            wf_H_cm = payloadFrame(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, pl_idx);
        end

        function M = massMatrix(obj, q_j, stFltb)
            if (nargin == 2)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            M = massMatrix(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j);
        end

        function h_c = centMoment(obj, q_j, dq_j, stFltb)
            if (nargin == 3)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            h_c = centroidalMomentum(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function [M, c_qv, h_c] = wholeBodyDyn(obj, q_j, dq_j, stFltb)
            if (nargin == 3)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            [M, c_qv, h_c] = wholeBodyDynamics(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function wf_J_lnk = jacob(obj, lnk_name, q_j, stFltb)
            if (nargin == 3)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            wf_J_lnk = jacobian(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, lnk_name);
        end

        function djdq_lnk = jacobDot(obj, lnk_name, q_j, dq_j, stFltb)
            if (nargin == 4)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            djdq_lnk = dJdq(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b, lnk_name);
        end

        function wf_J_tt = jacobTool(obj, t_idx, q_j, stFltb) % Jacobian matrix in tool-frame
            if (nargin == 3)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            % compute the Jacobian of the tool-tip:
            wf_J_tt = jacobianTool(obj.mwbm_icub, stFltb.wf_R_b, stFltb.wf_p_b, q_j, t_idx);
        end

        function payload(obj, pl_lnk_data)
            setPayloadLinks(obj.mwbm_icub, pl_lnk_data);
        end

        function f_pl = ploadForces(obj, fhTotCWrench, f_cp, tau, q_j, dq_j, stFltb)
            if (nargin == 6)
                stFltb = getFloatingBaseState(obj.mwbm_icub);
            end
            nu    = vertcat(stFltb.v_b, dq_j); % mixed generalized velocity
            acf_0 = zeroCtcAcc(obj.mwbm_icub, obj.mfoot_conf);
            [ac_h, a_prms] = handAccelerations(obj.mwbm_icub, obj.mfoot_conf, obj.mhand_conf, tau, acf_0, ...
                                               stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b, nu);
            v_pl = a_prms.Jc_h * dq_j;
            a_pl = ac_h;
            % calculate the payload forces of the hands:
            f_pl = handPayloadForces(obj.mwbm_icub, obj.mhand_conf, fhTotCWrench, f_cp, v_pl, a_pl);
        end

        function resv = isJntLimit(obj, q_j)
            resv = isJointLimit(obj.mwbm_icub, q_j);
        end

        function dispParams(obj, prec)
            if (nargin == 2)
               dispModel(obj.mwbm_icub, prec);
               dispConfig(obj.mwbm_icub, prec);
               return
            end
            % else, display the values with the default precision ...
            dispModel(obj.mwbm_icub);
            dispConfig(obj.mwbm_icub);
        end

        function setToolTform(obj, ee_H_tt, t_idx)
            if (nargin == 1)
                % initialize the tool-tip transformation matrices:
                [tool_lnks, nTools] = getToolLinks(obj.mwbm_icub);
                if (nTools > 0)
                    % get the tool-tip transformations from the WBM-object:
                    % note: the first element is always the default tool link.
                    obj.mtool_tform = cell(nTools,1);
                    for t_idx = 1:nTools
                        ee_H_tt = WBM.utilities.tfms.frame2tform(tool_lnks(t_idx,1).ee_vqT_tt);
                        obj.mtool_tform{t_idx,1} = ee_H_tt;
                    end
                else
                    n = obj.mwbm_icub.MAX_NUM_TOOLS;
                    obj.mtool_tform = cell(n,1);

                    H_id = eye(4,4);
                    for t_idx = 1:n
                        obj.mtool_tform{t_idx,1} = H_id;
                    end
                end
                return
            end
            % else, update the transformation matrix of the default tool-tip ...
            if ~WBM.utilities.isHomog(ee_H_tt)
                error('iCubWBM::setToolTform: %s', WBM.wbmErrorMsg.NOT_HOMOG_MAT);
            end
            ee_vqT_tt = WBM.utilities.tfms.tform2frame(ee_H_tt);
            updateToolFrame(obj.mwbm_icub, ee_vqT_tt, t_idx);
            obj.mtool_tform{t_idx,1} = ee_H_tt;
        end

        function ee_H_tt = getToolTform(obj, t_idx)
            if (t_idx > obj.mwbm_icub.MAX_NUM_TOOLS)
                error('iCubWBM::getToolTform: %s', WBM.wbmErrorMsg.MAX_NUM_LIMIT);
            end
            ee_H_tt = obj.mtool_tform{t_idx,1};
        end

        function set.robot_name(obj, robot_name)
            obj.mrobot_name = robot_name;
        end

        function robot_name = get.robot_name(obj)
            robot_name = obj.mrobot_name;
        end

        function set.robot_model(obj, model_name)
            if (nargin == 1)
                if ~isempty(obj.mwbm_icub)
                    % get the model name from WBM-object of the iCub ...
                    [~,model_name, ext] = fileparts(obj.mwbm_icub.robot_model.urdfRobot);
                    obj.mrobot_model = strcat(model_name, ext);
                    return
                end
            end
            % else ...
            obj.mrobot_model = model_name;
        end

        function model_name = get.robot_model(obj)
            if isempty(obj.mrobot_model)
                model_name = 'unknown'; return
            end
            % else ...
            model_name = obj.mrobot_model;
        end

        function set.robot_manuf(obj, manuf)
            obj.mrobot_manuf = manuf;
        end

        function robot_manuf = get.robot_manuf(obj)
            if isempty(obj.mrobot_manuf)
                robot_manuf = 'unknown'; return
            end
            % else ...
            robot_manuf = obj.mrobot_manuf;
        end

        function robot_params = get.robot_params(obj)
            robot_params = obj.mwbm_icub.robot_params;
        end

        function set.sim_config(obj, sim_config)
            obj.msim_config = setupSimulation(obj.mwbm_icub, sim_config);
        end

        function sim_config = get.sim_config(obj)
            sim_config = obj.msim_config;
        end

        function set.base_link(obj, rlnk_name)
            if isempty(rlnk_name)
                error('iCubWBM::set.base_link: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end
            [~,q_j,~,~] = getState(obj.mwbm_icub);
            % update the fixed link ...
            obj.mwbm_icub.fixed_link = rlnk_name;
            % calculate the the position and orientation of the new base link
            % (fixed link) to the world frame (WF) and set the WF at this new base:
            [wf_p_b, wf_R_b] = getWorldFrameFromFixLnk(obj.mwbm_icub, rlnk_name, q_j);
            setWorldFrame(obj.mwbm_icub, wf_R_b, wf_p_b);
        end

        function rlnk_name = get.base_link(obj)
            rlnk_name = obj.mwbm_icub.fixed_link;
        end

        function set.base_tform(obj, wf_H_b)
            if (nargin == 1)
                % get the initial base transformation matrix
                % from the WBM-object of the iCub ...
                wf_R_b = obj.mwbm_icub.init_wf_R_b;
                wf_p_b = obj.mwbm_icub.init_wf_p_b;
                obj.mbase_tform = WBM.utilities.tfms.posRotm2tform(wf_p_b, wf_R_b);
                return
            end
            % else, update the base transformation ...
            if ~WBM.utilities.isHomog(wf_H_b)
                error('iCubWBM::set.base_tform: %s', WBM.wbmErrorMsg.NOT_HOMOG_MAT);
            end
            [wf_p_b, wf_R_b] = WBM.utilities.tfms.tform2posRotm(wf_H_b);
            setInitWorldFrame(obj.mwbm_icub, wf_R_b, wf_p_b);
            obj.mbase_tform = wf_H_b;
        end

        function wf_H_b = get.base_tform(obj)
            wf_H_b = obj.mbase_tform;
        end

        function set.foot_conf(obj, foot_conf)
            obj.mfoot_conf = foot_conf;
        end

        function foot_conf = get.foot_conf(obj)
            foot_conf = obj.mfoot_conf;
        end

        function set.hand_conf(obj, hand_conf)
            obj.mhand_conf = hand_conf;
        end

        function hand_conf = get.hand_conf(obj)
            hand_conf = obj.mhand_conf;
        end

        function set.gravity(obj, g_wf)
            obj.mwbm_icub.g_wf = g_wf;
            setInitWorldFrame(obj.mwbm_icub);
        end

        function g_wf = get.gravity(obj)
            g_wf = obj.mwbm_icub.g_wf;
        end

        function jlmts = get.jlimits(obj)
            jlmts = obj.mwbm_icub.joint_limits;
        end

        function set.ndof(obj, ndof)
            obj.mwbm_icub.ndof = ndof;
        end

        function ndof = get.ndof(obj)
            if isempty(obj.mwbm_icub)
                ndof = 0; return
            end
            ndof = obj.mwbm_icub.ndof;
        end

        function hwbm = get.hwbm(obj)
            hwbm = obj.mwbm_icub;
        end

    end
end
