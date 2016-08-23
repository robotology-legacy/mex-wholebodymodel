classdef MultChainTree < WBM.Interfaces.IMultChainTree
    properties(Dependent)
        name@char       % the name of the robot.
        manuf@char      % the name of the manufacturer (annotation)
        comment@char    % general comment (annotation)
        wbm_info@struct % general information about whole body model of the robot.
        wbm_params@WBM.wbmBaseRobotParams
        %model3d
        plotopt3d@WBM.absSimConfig
        ctrl_link@char        % current kinematic link of the robot that is controlled by the system.
        gravity@double vector % gravity vector (direction of the gravity)
        base@double    matrix % base transform of the robot (pose of the robot)
        tool@double    matrix % tool transform (from the end-effector to the tool-tip)
        qlim@double    matrix % joint limits, [qmin qmax] (Nx2)
        n@uint16       scalar % number of joints (equivalent to number of DoFs)

        %interface % interface to a real robot platform
    end

    properties(Access = protected)
        mwbm@IWBM
        mwbm_info = struct( 'robot_name',  '', ...
                            'robot_manuf', '', ...
                            'comment',     '' );
        mctrl_link@char
    end

    methods
        function obj = MultChainTree(robot_wbm, ctrl_link, comment)
            if ~isa(robot_wbm, 'IWBM')
                error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if isempty(ctrl_link)
                error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end

            obj.mwbm = robot_wbm;
            obj.mctrl_link = ctrl_link;

            % get some informations about the WBM of the robot ...
            obj.mwbm_info.robot_name  = robot_wbm.robot_name;
            obj.mwbm_info.robot_manuf = robot_wbm.robot_manuf;
            if exist('comment', 'var')
                obj.mwbm_info.comment = comment;
            end
        end

        function delete(obj)
            delete(obj);
        end

        function ddq_j = accel(obj, q_j, dq_j, tau)
            ddq_j = obj.mwbm.jointAccelerations(q_j, dq_j, tau);
        end

        function tau_c = corolis(obj, q_j, dq_j)
            tau_c = obj.mwbm.coriolisForces(q_j, dq_j);
        end

        function tau_fr = friction(obj, dq_j)
            tau_fr = obj.mwbm.frictionForces(dq_j);
        end

        function tau_g = gravload(obj, q_j)
            tau_g = obj.mwbm.gravityForces(q_j);
        end

        function tau_ctrl = invdyn(obj, q_j, dq_j, ddq_j)
            tau_ctrl = obj.mwbm.inverseDyn(q_j, dq_j, ddq_j);
        end

        function [t, stmChi] = fdyn(obj, fhCtrlTrqs, tspan, stvChi_0, ode_opt)
            [t, stmChi] = obj.mwbm.forwardDyn(fhCtrlTrqs, tspan, stvChi_0, ode_opt);
        end

        function wf_H_lnk = fkine(obj, q_j)
            wf_H_lnk = obj.mwbm.forwardKin(obj.mctrl_link, q_j);
        end

        function wf_H_lnk = A(obj, jnt_idx, q_j)
            wf_H_lnk = obj.mwbm.linkFrame(jnt_idx, q_j);
        end

        function dJ = jacob_dot(obj, q_j, dq_j)
            dJ = obj.mwbm.jacobianDot(q_j, obj.mctrl_link, dq_j);
        end

        function J_0 = jacob0(obj, q_j, varargin)
            opt.rpy   = false;
            opt.eul   = false;
            opt.trans = false;
            opt.rot   = false;
            opt = tb_optparse(opt, varargin);

            J_0 = obj.mwbm.jacobian(q_j, obj.mctrl_link);

            if opt.rpy % why do we make there a block-diagonale?
                wf_H_lnk = obj.mwbm.forwardKin(obj.mctrl_link, q_j);
                B_inv = WBM.utilities.tform2angRateTF(wf_H_lnk, 'ZYX'); % use the RPY (ZYX) euler-angles

                J_0 = blkdiag(eye(3,3), B_inv) * J_0;
            elseif opt.eul
                wf_H_lnk = obj.mwbm.forwardKin(obj.mctrl_link, q_j);
                B_inv = WBM.utilities.tform2angRateTF(wf_H_lnk, 'ZYZ'); % use the ZYZ euler-angles

                J_0 = blkdiag(eye(3,3), B_inv) * J_0;
            end

            if opt.trans
                J_0 = J_0(1:3,1:6);
            elseif opt.rot
                J_0 = J_0(4:6,1:6);
            end
        end

        % function J_n = jacobn(obj, q_j) % Jacobian in the ee-frame. Makes sense for a robot arm. But for a humanoid?
        %     J_n = mwbm.jacobianTool(obj, q_j);
        % end

        function M = inertia(obj, q_j)
            M = obj.mwbm.massMatrix(q_j);
        end

        % function gravjac()

        % end

        function resv = islimit(obj, q_j)
            resv = obj.mwbm.islimit(q_j);
        end

        function plot3d(obj, x_out, sim_tstep, vis_ctrl)
            obj.mwbm.visualizeFDyn(x_out, sim_tstep, vis_ctrl);
        end

        function set.name(obj, robot_name)
            obj.mwbm_info.robot_name = robot_name;
        end

        function robot_name = get.name(obj)
            robot_name = obj.mwbm_info.robot_name;
        end

        function set.manuf(obj, robot_manuf)
            obj.mwbm_info.robot_manuf = robot_manuf;
        end

        function robot_manuf = get.manuf(obj)
            robot_manuf = obj.mrobot_manuf;
        end

        function set.comment(obj, comment)
            obj.mwbm_info.comment = comment;
        end

        function comment = get.comment(obj)
            comment = obj.mwbm_info.comment;
        end

        function wbm_info = get.wbm_info(obj)
            wbm_info = obj.mwbm_info;
        end

        function wbm_params = get.wbm_params(obj)
            wbm_params = obj.mwbm.robot_params;
        end

        function set.plotopt3d(obj, sim_config)
            obj.mwbm.sim_config = sim_config;
        end

        function sim_config = get.plotopt3d(obj)
            sim_config = obj.mwbm.sim_config;
        end

        function set.ctrl_link(obj, link_name)
            if isempty(link_name)
                error('MultChainTree::set.ctrl_link: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end
            obj.mctrl_link = link_name;
        end

        function link_name = get.ctrl_link(obj)
            link_name = obj.mctrl_link;
        end

        function set.gravity(obj, g_wf)
            obj.mwbm.gravity = g_wf;
        end

        function g_wf = get.gravity(obj)
            g_wf = obj.mwbm.g_wf;
        end

        function set.base(obj, tform)
            obj.mwbm.base_tform = tform;
        end

        function tform = get.base(obj)
            tform = obj.mwbm.base_tform;
        end

        function set.tool(obj, tform)
            obj.mwbm.tool_tform = tform;
        end

        function tform = get.tool(obj)
            tform = obj.mwbm.tool_tform;
        end

        function jlmts = get.qlim(obj)
            jl    = obj.mwbm.jlimits;
            jlmts = horzcat(jl.lwr, jl.upr);
        end

        function ndof = get.n(obj)
            ndof = obj.mwbm.ndof;
        end

    end
end
