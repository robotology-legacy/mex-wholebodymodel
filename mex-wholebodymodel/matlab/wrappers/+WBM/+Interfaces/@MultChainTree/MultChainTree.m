classdef MultChainTree < WBM.Interfaces.IMultChainTree
    properties(Dependent)
        name@char       % the name of the robot.
        manuf@char      % the name of the manufacturer (annotation)
        comment@char    % general comment (annotation)
        wbm_info@struct % general information about whole body model of the robot.
        wbm_params@WBM.wbmBaseRobotParams
        plotopt3d@WBM.absSimConfig
        ctrl_link@char        % current kinematic link of the robot that is controlled by the system.
        ee_link@char          % kinematic link of the current end-effector that is controlled by the system.
        gravity@double vector % gravity vector (direction of the gravity)
        base@double    matrix % base transform of the robot (pose of the robot)
        tool@double    matrix % tool transform (from the end-effector to the tool-tip)
        qlim@double    matrix % joint limits, [qmin qmax] (Nx2)
        n@uint16       scalar % number of joints (equivalent to number of DoFs)

        %interface % interface to a real robot platform
    end

    properties(Access = protected)
        mwbm@WBM.Interfaces.IWBM
        mwbm_info = struct( 'robot_name',  '', ...
                            'robot_manuf', '', ...
                            'comment',     '' );
        mlink_ctrl@char
        mlink_ee@char
    end

    methods
        function bot = MultChainTree(robot_wbm, ctrl_link, comment)
            if ~isa(robot_wbm, 'WBM.Interfaces.IWBM')
                error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if isempty(ctrl_link)
                error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end

            bot.mwbm = robot_wbm;
            bot.mlink_ctrl = ctrl_link;

            % get some informations about the WBM of the robot ...
            bot.mwbm_info.robot_name  = robot_wbm.robot_name;
            bot.mwbm_info.robot_manuf = robot_wbm.robot_manuf;
            if exist('comment', 'var')
                bot.mwbm_info.comment = comment;
            end
        end

        function delete(bot)
            delete(bot);
        end

        function [q_j, dq_j, wf_H_b, v_b] = getstate(bot)
            switch nargout
                case 2
                    [~,q_j,~,dq_j] = bot.mwbm.getState();
                case 4
                    [vqT_b, q_j, v_b, dq_j] = bot.mwbm.getState();
                    wf_H_b = WBM.utilities.tfms.frame2tform(vqT_b);
                otherwise
                    error('MultChainTree::getstate: %s', WBM.wbmErrorMsg.WRONG_NARGOUT);
            end
        end

        function ddq_j = accel(bot, q_j, dq_j, tau)
            ddq_j = bot.mwbm.jointAccelerations(tau, q_j, dq_j);
        end

        function c_qv = coriolis(bot, q_j, dq_j)
            c_qv = bot.mwbm.coriolisForces(q_j, dq_j);
        end

        function tau_fr = friction(bot, dq_j)
            tau_fr = bot.mwbm.frictionForces(dq_j);
        end

        function g_q = gravload(bot, q_j)
            g_q = bot.mwbm.gravityForces(q_j);
        end

        function tau_j = rne(bot, q_j, dq_j, ddq_j)
            tau_j = bot.mwbm.inverseHybridDyn(q_j, dq_j, ddq_j);
        end

        function [t, stmChi] = fdyn(bot, tspan, stvChi_0, fhCtrlTrqs, ode_opt, varargin)
            [t, stmChi] = bot.mwbm.forwardDyn(tspan, stvChi_0, fhCtrlTrqs, ode_opt, varargin{:});
        end

        function wf_H_lnk = fkine(bot, q_j)
            wf_H_lnk = bot.mwbm.forwardKin(bot.mlink_ctrl, q_j);
        end

        function wf_H_lnk = A(bot, lnk_name, q_j)
            wf_H_lnk = bot.mwbm.linkFrame(lnk_name, q_j);
        end

        function wf_H_ee = T0_n(bot, q_j) % computes the forward kinematics of the current end-effector.
            vqT_ee = bot.mwbm.forwardKin(bot.mlink_ee, q_j);
            wf_H_ee = WBM.utilities.tfms.frame2tform(vqT_ee);
        end

        function djdq_lnk = jacob_dot(bot, q_j, dq_j)
            djdq_lnk = bot.mwbm.jacobianDot(bot.mlink_ctrl, q_j, dq_j);
        end

        function wf_J_lnk = jacob0(bot, q_j, varargin)
            % options:
            opt.rpy   = false;
            opt.eul   = false;
            opt.trans = false;
            opt.rot   = false;
            opt = tb_optparse(opt, varargin); % function from the RTB of Peter Corke.

            wf_J_lnk = bot.mwbm.jacobian(bot.mlink_ctrl, q_j);

            if opt.rpy
                % compute the analytical Jacobian with the Euler rotation rate in ZYX (RPY) order:
                wf_H_lnk = bot.mwbm.forwardKin(bot.mlink_ctrl, q_j);
                Er_inv   = WBM.utilities.tfms.tform2angRateTF(wf_H_lnk, 'eul', 'ZYX');
                if (rcond(Er_inv) < eps)
                    error('MultChainTree::jacob0: %s', WBM.wbmErrorMsg.SINGULAR_MAT);
                end
                wf_rX_lnk = eye(6,6);
                wf_rX_lnk(4:6,4:6) = Er_inv;

                wf_J_lnk = wf_rX_lnk * wf_J_lnk;
            elseif opt.eul
                % compute the analytical Jacobian with the Euler rotation rate in ZYZ order:
                wf_H_lnk = bot.mwbm.forwardKin(bot.mlink_ctrl, q_j);
                Er_inv   = WBM.utilities.tfms.tform2angRateTF(wf_H_lnk, 'eul', 'ZYZ');
                if (rcond(Er_inv) < eps)
                    error('MultChainTree::jacob0: %s', WBM.wbmErrorMsg.SINGULAR_MAT);
                end
                wf_rX_lnk = eye(6,6);
                wf_rX_lnk(4:6,4:6) = Er_inv;

                wf_J_lnk = wf_rX_lnk * wf_J_lnk;
            end

            if opt.trans
                % return the translational sub-matrix of the Jacobian:
                wf_J_lnk = wf_J_lnk(1:3,1:6);
            elseif opt.rot
                % return the rotational sub-matrix of the Jacobian:
                wf_J_lnk = wf_J_lnk(4:6,1:6);
            end
        end

        function wf_J_ee = jacobn(bot, q_j, varargin) % Jacobian of the current ee-frame.
            % options:
            opt.trans = false;
            opt.rot   = false;
            opt = tb_optparse(opt, varargin);

            % Jacobian of the end-effector controlled by the system:
            wf_J_ee = bot.mwbm.jacobian(bot.mlink_ee, q_j);

            if opt.trans
                % translational sub-matrix:
                wf_J_ee = wf_J_ee(1:3,1:6);
            elseif opt.rot
                % rotational sub-matrix:
                wf_J_ee = wf_J_ee(4:6,1:6);
            end
        end

        function M = inertia(bot, q_j)
            M = bot.mwbm.massMatrix(q_j);
        end

        function Mx = cinertia(bot, q_j)
            M = bot.mwbm.massMatrix(q_j);
            wf_J_lnk = bot.mwbm.jacobian(bot.mlink_ctrl, q_j);

            % calculate the Cartesian mass matrix (pseudo-kinetic energy matrix)
            %   Mx = (J * M^(-1) * J^T)^(-1)
            % in operational space:
            [Mx,~,~] = WBM.utilities.tfms.cartmass(wf_J_lnk, M);
        end

        function payload(bot, pl_data)
            bot.mwbm.payload(pl_data);
        end

        function f_pl = pay(bot, fhTotCWrench, f_cp, tau, q_j, dq_j)
           f_pl = bot.mwbm.payloadForces(fhTotCWrench, f_cp, tau, q_j, dq_j);
        end

        function resv = islimit(bot, q_j)
            resv = bot.mwbm.islimit(q_j);
        end

        function plot3d(bot, x_out, sim_tstep, vis_ctrl)
            bot.mwbm.visualizeFDyn(x_out, sim_tstep, vis_ctrl);
        end

        function set.name(bot, robot_name)
            bot.mwbm_info.robot_name = robot_name;
        end

        function robot_name = get.name(bot)
            robot_name = bot.mwbm_info.robot_name;
        end

        function set.manuf(bot, robot_manuf)
            bot.mwbm_info.robot_manuf = robot_manuf;
        end

        function robot_manuf = get.manuf(bot)
            robot_manuf = bot.mrobot_manuf;
        end

        function set.comment(bot, comment)
            bot.mwbm_info.comment = comment;
        end

        function comment = get.comment(bot)
            comment = bot.mwbm_info.comment;
        end

        function wbm_info = get.wbm_info(bot)
            wbm_info = bot.mwbm_info;
        end

        function wbm_params = get.wbm_params(bot)
            wbm_params = bot.mwbm.robot_params;
        end

        function set.plotopt3d(bot, sim_config)
            bot.mwbm.sim_config = sim_config;
        end

        function sim_config = get.plotopt3d(bot)
            sim_config = bot.mwbm.sim_config;
        end

        function set.ctrl_link(bot, lnk_name)
            if isempty(lnk_name)
                error('MultChainTree::set.ctrl_link: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end
            bot.mlink_ctrl = lnk_name;
        end

        function lnk_name = get.ctrl_link(bot)
            lnk_name = bot.mlink_ctrl;
        end

        function set.ee_link(bot, lnk_name)
            if isempty(lnk_name)
                error('MultChainTree::set.ee_link: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end
            bot.mlink_ee = lnk_name;
        end

        function lnk_name = get.ee_link(bot)
            lnk_name = bot.mlink_ee;
        end

        function set.gravity(bot, g_wf)
            bot.mwbm.gravity = g_wf;
        end

        function g_wf = get.gravity(bot)
            g_wf = bot.mwbm.g_wf;
        end

        function set.base(bot, tform)
            bot.mwbm.base_tform = tform;
        end

        function tform = get.base(bot)
            tform = bot.mwbm.base_tform;
        end

        function set.tool(bot, tform)
            bot.mwbm.tool_tform = tform;
        end

        function tform = get.tool(bot)
            tform = bot.mwbm.tool_tform;
        end

        function jlmts = get.qlim(bot)
            jlim  = bot.mwbm.jlimits;
            jlmts = horzcat(jlim.lwr, jlim.upr);
        end

        function ndof = get.n(bot)
            ndof = bot.mwbm.ndof;
        end

    end
end
