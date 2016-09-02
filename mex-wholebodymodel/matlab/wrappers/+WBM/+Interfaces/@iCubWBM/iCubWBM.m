classdef iCubWBM < WBM.Interfaces.IWBM
    properties(Dependent)
        % public properties for fast get/set methods:
        robot_name@char
        robot_model@char
        robot_manuf@char
        robot_params@WBM.wbmBaseRobotParams
        sim_config@WBM.absSimConfig
        base_tform@double matrix
        tool_tform@double matrix
        foot_conf@struct
        gravity@double    vector
        jlimits@struct
        ndof@uint16       scalar
    end

    properties(Access = protected)
        mwbm_icub@WBM.WBM
        mrobot_name@char
        mrobot_model@char
        mrobot_manuf@char
        msim_config@WBM.absSimConfig
        mbase_tform@double matrix
        mtool_tform@double matrix
        mfoot_conf@struct
    end

    methods
        % Constructor:
        function obj = iCubWBM(robot_model, robot_config, wf2fixLnk)
            obj.mrobot_name  = 'iCub';
            obj.mrobot_manuf = 'Istituto Italiano di Tecnologia (IIT) - Genoa, Italy.';

            obj.mbase_tform  = eye(4,4);
            obj.mtool_tform  = obj.mbase_tform;

            switch nargin
                % init the mex-WholeBodyModel for the iCub-Robot:
                case 3
                    obj.mwbm_icub = WBM.WBM(robot_model, robot_config, wf2fixLnk);
                case 2
                    obj.mwbm_icub = WBM.WBM(robot_model, robot_config);
            end
            % else, do nothing ...
        end

        function initRobot(obj, robot_wbm)
            if ~isa(robot_wbm, 'WBM.WBM')
                error('iCubWBM::initRobot: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.mwbm_icub = copy(robot_wbm);
        end

        function initRobotFcn(obj, fhInitRobotWBM, wf2fixLnk)
            if ~isa(fhInitRobotWBM, 'function_handle')
                error('iCubWBM::initRobotFcn: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE)
            end

            if ~exist('wf2fixLnk', 'var')
                wf2fixLnk = true;
            end
            obj.mwbm_icub = fhInitRobotWBM(wf2fixLnk);
        end

        function initBaseRobotParams(obj, robot_params)
            if ~isa(robot_params, 'WBM.wbmBaseRobotParams')
                error('iCubWBM::initBaseRobotParams: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.mwbm_icub = WBM.WBM(robot_params.model, robot_params.config, robot_params.wf2fixLnk);
        end

        function delete(obj)
            delete(obj);
        end

        function [vqT_b, q_j, v_b, dq_j] = getState(obj)
            [vqT_b, q_j, v_b, dq_j] = obj.mwbm_icub.getState();
        end

        function stFltb = getFloatingBaseState(obj)
            stFltb = obj.mwbm_icub.getFloatingBaseState();
        end

        function ddq_j = jointAccelerations(obj, q_j, dq_j, tau, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            ddq_j = obj.mwmbm_icub.jointAcceleration(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b, tau);
        end

        function ddq_j = jointAccelerationsExt(obj, q_j, dq_j, tau, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            ddq_j = obj.mwmbm_icub.jointAccelerationsExt(stFltb.wf_R_b, stFltb.wf_p_b, q_j, ...
                                                         dq_j, stFltb.v_b, tau, obj.mfoot_conf);
        end

        function tau_c = coriolisForces(obj, q_j, dq_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            tau_c = obj.mwbm_icub.coriolisCentrifugalForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function tau_fr = frictionForces(obj, dq_j)
            tau_fr = obj.mwbm_icub.frictionForces(dq_j);
        end

        function C_qv = generalizedBiasForces(obj, q_j, dq_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            C_qv = obj.mwbm_icub.generalizedBiasForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function tau_gen = generalizedForces(obj, q_j, dq_j, f_c, Jc_t, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            tau_gen = obj.mwbm_icub.generalizedForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b, f_c, Jc_t);
        end

        function tau_g = gravityForces(obj, q_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            tau_g = obj.mwbm_icub.gravityForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
        end

        function tau_j = inverseDyn(obj, q_j, dq_j, ddq_j, dv_b, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            tau_j = obj.mwbm_icub.inverseDynamics(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b, ddq_j, dv_b);
        end

        function tau_j = inverseHybridDyn(obj, q_j, dq_j, ddq_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            tau_j = obj.mwbm_icub.inverseDynamics(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b, ddq_j);
        end

        function [t, stmChi] = forwardDyn(obj, tspan, fhTrqControl, stvChi_0, ode_opt, foot_conf)
            if exist('foot_conf', 'var')
                % use the extended functions for the ODE-solver ...
                [t, stmChi] = obj.mwbm_icub.intForwardDynamics(fhTrqControl, tspan, stvChi_0, ode_opt, foot_conf);
                return
            end
            % else, use the normal functions ...
            [t, stmChi] = obj.mwbm_icub.intForwardDynamics(fhTrqControl, tspan, stvChi_0, ode_opt);
        end

        function visualizeForwardDyn(obj, stmChi, sim_tstep, vis_ctrl)
            pos_out = obj.mwbm_icub.getPositionsData(stmChi);
            obj.mwbm_icub.visualizeForwardDynamics(pos_out, obj.msim_config, sim_tstep, vis_ctrl);
        end

        function wf_H_lnk = forwardKin(obj, lnk_name, q_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            vqT_lnk  = obj.mwbm_icub.forwardKinematics(stFltb.wf_R_b, stFltb.wf_p_b, q_j, lnk_name);
            wf_H_lnk = WBM.utilities.frame2tform(vqT_lnk);
        end

        function wf_H_lnk = linkFrame(obj, lnk_name, q_j, stFltb) % link transformation matrix
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            wf_H_lnk = obj.mwbm_icub.transformationMatrix(stFltb.wf_R_b, stFltb.wf_p_b, q_j, lnk_name);
        end

        function wf_H_tt = toolFrame(obj, t_idx, q_j, stFltb) % tool-tip transformation matrix
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            wf_H_tt = obj.mwbm_icub.toolFrame(stFltb.wf_R_b, stFltb.wf_p_b, q_j, t_idx);
        end

        function M = massMatrix(obj, q_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            M = obj.mwbm_icub.massMatrix(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
        end

        function h_c = centroidalMomentum(obj, q_j, dq_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            h_c = obj.mwbm_icub.centroidalMomentum(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function [M, C_qv, h_c] = wholeBodyDyn(obj, q_j, dq_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            [M, C_qv, h_c] = obj.mwbm_icub.wholeBodyDynamics(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b);
        end

        function J = jacobian(obj, lnk_name, q_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            J = obj.mwbm_icub.jacobian(stFltb.wf_R_b, stFltb.wf_p_b, q_j, lnk_name);
        end

        function dJ = jacobianDot(obj, lnk_name, q_j, dq_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            dJ = obj.mwbm_icub.dJdq(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b, lnk_name);
        end

        function J_tt = jacobianTool(obj, q_j, stFltb) % Jacobian matrix in tool-frame
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            % compute the jacobian of the tool-tip:
            % use the default tool (1st element of the tool list)
            J_tt = obj.mwbm_icub.jacobianTool(stFltb.wf_R_b, stFltb.wf_p_b, q_j, 1);
        end

        function payload(obj, pt_mass, pos, link_names)
            if ( ~iscolumn(pt_mass) || ~ismatrix(pos) || (size(pt_mass,1) ~= size(pos,1)) )
                error('iCubWBM::payload: %s', WBM.utilities.DIM_MISMATCH);
            end
            pl_data = horzcat(pt_mass, pos);
            obj.mwbm_icub.setLinkPayloads(link_names, pl_data);
        end

        % function tau_pl = payloadForces(obj, q_j, dq_j, stFltb)

        % end

        function resv = islimit(obj, q_j)
            resv = obj.mwbm_icub.isJointLimit(q_j);
        end

        function dispParams(obj, prec)
            if exist('prec', 'var')
               obj.mwbm_icub.dispModel(prec);
               obj.mwbm_icub.dispConfig(prec);
               return
            end
            % else, display the values with the default precision ...
            obj.mwbm_icub.dispModel();
            obj.mwbm_icub.dispConfig();
        end

        function set.robot_name(obj, robot_name)
            obj.mrobot_name = robot_name;
        end

        function robot_name = get.robot_name(obj)
            robot_name = sprintf('%s, model: %s', obj.mrobot_name, obj.robot_model);
        end

        function set.robot_model(obj, model_name)
            if ~exist('model_name', 'var')
                if ~isempty(obj.mwbm_icub)
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
            obj.msim_config = obj.mwbm_icub.setupSimulation(sim_config);
        end

        function sim_config = get.sim_config(obj)
            sim_config = obj.msim_config;
        end

        function set.base_tform(obj, tform)
            if ~exist('tform', 'var')
                R_b = obj.mwbm_icub.wf_R_b;
                p_b = obj.mwbm_icub.wf_p_b;
                obj.mbase_tform = WBM.utilities.posRotm2tform(p_b, R_b);
                return
            end
            % else, update the base transformation ...
            if ~WBM.utilities.isHomog(tform)
                error('iCubWBM::set.base_tform: %s', WBM.wbmErrorMsg.NOT_HOMOG_MAT);
            end

            [p_b, R_b] = WBM.utilities.tform2posRotm(tform);
            obj.mwbm_icub.setInitWorldFrame(R_b, p_b);
            obj.mbase_tform = tform;
        end

        function tform = get.base_tform(obj)
            tform = obj.mbase_tform;
        end

        function set.tool_tform(obj, tform)
            if ~exist('tform', 'var')
                [tool_lnks, nTools] = obj.mwbm_icub.getToolLinks();
                if (nTools > 0)
                    % use the default tool (is always the first element of the list)
                    obj.mtool_tform = WBM.utilities.frame2tform(tool_lnks(1,1).ee_vqT_tt);
                else
                    obj.mtool_tform = eye(4,4);
                end
                return
            end
            % else, update the transformation matrix of the default tool-tip ...
            if ~WBM.utilities.isHomog(tform)
                error('iCubWBM::set.tool_tform: %s', WBM.wbmErrorMsg.NOT_HOMOG_MAT);
            end

            ee_vqT_tt = WBM.utilities.tform2frame(tform);
            obj.mwbm_icub.updateToolFrame(1, ee_vqT_tt);
            obj.mtool_tform = tform;
        end

        function tform = get.tool_tform(obj)
            tform = obj.mtool_tform;
        end

        function set.foot_conf(obj, foot_conf)
            obj.mfoot_conf = foot_conf;
        end

        function foot_conf = get.foot_conf(obj)
            foot_conf = obj.mfoot_conf;
        end

        function set.gravity(obj, g_wf)
            obj.mwbm_icub.g_wf = g_wf;
            obj.mwbm_icub.setInitWorldFrame();
        end

        function g_wf = get.gravity(obj)
            g_wf = obj.mwbm_icub.g_wf;
        end

        function jl = get.jlimits(obj)
            jl = obj.mwbm_icub.joint_limits;
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

    end
end
