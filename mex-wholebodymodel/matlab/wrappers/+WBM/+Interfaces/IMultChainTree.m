classdef (Abstract) IMultChainTree < handle
    properties(Abstract, Dependent)
        % public properties for fast get/set methods:
        name@char       % the name of the robot.
        manuf@char      % the name of the manufacturer (annotation)
        comment@char    % general comment (annotation)
        wbm_info@struct % general information about whole body model of the robot.
        wbm_params@WBM.wbmBaseRobotParams % base model and configuration parameters of the robot
        plotopt3d@WBM.absSimConfig
        base_link@char        % floating base link (fixed reference link) of the robot.
        ctrl_link@char        % current kinematic link of the robot that is controlled by the system.
        ee_links              % kinematic links of the end-effectors (hands) that are controlled by the system.
        gravity@double vector % gravity vector (direction of the gravity)
        base@double    matrix % base transform of the robot (pose of the robot)
        tool@double    matrix % tool transform (from the end-effector to the tool-tip)
        qlim@double    matrix % joint limits, [qmin qmax] (Nx2)
        n@uint16       scalar % number of joints (equivalent to number of DoFs)

        %interface % interface to a real robot platform
    end

    methods(Abstract)
        [q_j, dq_j, wf_H_b, v_b] = getstate(bot);

        ddq_j = accel(bot, q_j, dq_j, tau)

        c_qv = coriolis(bot, q_j, dq_j)

        tau_fr = friction(bot, dq_j)

        g_q = gravload(bot, q_j)

        [g_q, wf_J_b] = gravjac(bot, q_j)

        tau_j = rne(bot, q_j, dq_j, ddq_j)

        [t, stmChi] = fdyn(bot, tspan, stvChi_0, fhCtrlTrqs, ode_opt, varargin)

        wf_H_lnk = fkine(bot, q_j)

        wf_H_lnk = A(bot, lnk_name, q_j)

        wf_H_ee = T0_n(bot, q_j) % computes the forward kinematics of the end-effectors (hands).

        djdq_lnk = jacob_dot(bot, q_j, dq_j)

        wf_J_lnk = jacob0(bot, q_j, varargin)

        wf_J_ee = jacobn(bot, q_j, varargin) % Jacobians of the ee-frames.

        M = inertia(bot, q_j)

        Mx = cinertia(bot, q_j)

        payload(bot, pl_data);

        f_pl = pay(bot, fhTotCWrench, f_cp, tau, q_j, dq_j);

        resv = islimit(bot, q_j)

        plot3d(bot, x_out, sim_tstep, vis_ctrl)

    end
end
