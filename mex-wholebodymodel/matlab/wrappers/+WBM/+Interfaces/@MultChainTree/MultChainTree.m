classdef MultChainTree < WBM.Interfaces.IMultChainTree
    properties(Dependent)
        name@char       % the name of the robot.
        manuf@char      % the name of the manufacturer (annotation)
        comment@char    % general comment (annotation)
        wbm_info@struct % general information about whole body model of the robot.
        wbm_params@WBM.wbmBaseRobotParams
        %model3d
        plotopt3d@WBM.absSimConfig
        link_name@char % current kinematic link of the robot that is controlled by the system.
        gravity@double vector  % gravity vector (direction of the gravity)
        base@double    matrix  % base transform of the robot (pose of the robot)
        tool@double    matrix  % tool transform (from the end-effector to the tool-tip)
        qlim@double    matrix  % joint limits, [qmin qmax] (Nx2)
        n@uint16       scalar  % number of joints (equivalent to number of DoFs)

        %interface % interface to a real robot platform
    end

    properties(Access = protected)
        mwbm@IWBM
        mwbm_info = struct( 'robot_name',  '', ...
                            'robot_manuf', '', ...
                            'comment',     '' );
        mlink_name@char
    end

    methods
        function obj = MultChainTree(robot_wbm, link_name, comment)
            if ~isa(robot_wbm, 'IWBM')
                error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if isempty(link_name)
                error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end

            obj.mwbm = robot_wbm;
            obj.mlink_name = link_name;

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
            ddq_j = mwbm.acceleration(obj, q_j, dq_j, tau);
        end

        function tau_c = corolis(obj, q_j, dq_j)
            tau_c = mwbm.coriolisForces(obj, q_j, dq_j);
        end

        function tau_fr = friction(obj, dq_j)
            tau_fr = mwbm.frictionForces(obj, dq_j);
        end

        function tau_g = gravload(obj, q_j)
            tau_g = mwbm.gravityForces(obj, q_j);
        end

        function tau_gen = invdyn(obj, q_j, dq_j, ddq_j)
            tau_gen = mwbm.invDyn(obj, q_j, dq_j, ddq_j, obj.mlink_name);
        end

        function [t, stmChi] = fdyn(obj, fhCtrlTrqs, tspan, stvChi_0, ode_opt)
            [t, stmChi] = mwbm.forwardDyn(obj, fhCtrlTrqs, tspan, stvChi_0, ode_opt);
        end

        function w_H_rlnk = fkine(obj, q_j)
            w_H_rlnk = mwbm.forwardKin(obj, q_j, obj.mlink_name);
        end

        function w_H_rlnk = A(obj, joint_idx, q_j) % link transformation matrix (useful for the iCub?)
            w_H_rlnk = mwbm.linkFrame(obj, joint_idx, q_j);
        end

        function dJ = jacob_dot(obj, q_j, dq_j)
            dJ = mwbm.jacobianDot(obj, q_j, dq_j, obj.mlink_name);
        end

        function J_0 = jacob0(obj, q_j, varargin)
            opt.rpy   = false;
            opt.eul   = false;
            opt.trans = false;
            opt.rot   = false;
            opt = tb_optparse(opt, varargin);

            J_0 = mwbm.jacobian(obj, q_j, obj.mlink_name);

            if opt.rpy % why do we make there a block-diagonale?
                wf_H_rlnk = mwbm.fkine(obj, q_j, obj.mlink_name);
                B_inv = WBM.utilities.tform2angRateTF(wf_H_rlnk, 'ZYX'); % use the RPY (ZYX) euler-angles

                J_0 = blkdiag(eye(3,3), B_inv) * J_0;
            elseif opt.eul
                wf_H_rlnk = mwbm.fkine(obj, q_j, obj.mlink_name);
                B_inv = WBM.utilities.tform2angRateTF(wf_H_rlnk, 'ZYZ'); % use the ZYZ euler-angles

                J_0 = blkdiag(eye(3,3), B_inv) * J_0;
            end

            if opt.trans
                J_0 = J_0(1:3,1:6);
            elseif opt.rot
                J_0 = J_0(4:6,1:6);
            end
        end

        function J_n = jacobn(obj, q_j)
            J_n = mwbm.jacobianTool(obj, q_j);
        end

        function M = inertia(obj, q_j)
            M = mwbm.inertia(obj, q_j);
        end

        function cinertia() % ??

        end

        function payload(obj, pt_mass, pos, link_names)
            mwbm.payload(obj, pt_mass, pos, link_names);
        end

        function gravjac() % ??

        end

        function paycap() % ??

        end

        function tau_pl = pay(obj, )
            tau_pl = mwbm.payloadForces(obj, );
        end

        function resv = islimit(obj, q_j)
            resv = mwbm.islimit(obj, q_j);
        end

        function plot3d(obj, x_out, sim_tstep, vis_ctrl)
            mwbm.visualizeFDyn(obj, x_out, sim_tstep, vis_ctrl);
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

        function set.link_name(obj, link_name)
            if isempty(link_name)
                error('MultChainTree::set.link: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end
            obj.mlink_name = link_name;
        end

        function link_name = get.link_name(obj)
            link_name = obj.mlink_name;
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

        function jl = get.qlim(obj)
            jl = obj.mwbm.jlimits;
            jl = horzcat(jl.lwr, jl.upr);
        end

        function ndof = get.n(obj)
            ndof = obj.mwbm.ndof;
        end

    end
end
