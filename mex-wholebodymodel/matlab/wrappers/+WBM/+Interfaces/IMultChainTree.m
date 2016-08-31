classdef (Abstract) IMultChainTree < handle
    properties(Abstract, Dependent)
        % public properties for fast get/set methods:
        name@char       % the name of the robot.
        manuf@char      % the name of the manufacturer (annotation)
        comment@char    % general comment (annotation)
        wbm_info@struct % general information about whole body model of the robot.
        wbm_params@WBM.wbmBaseRobotParams % base model and configuration parameters of the robot
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

    methods(Abstract)
        ddq_j = accel(obj, q_j, dq_j, tau)

        tau_c = corolis(obj, q_j, dq_j)

        tau_fr = friction(obj, dq_j)

        tau_g = gravload(obj, q_j)

        tau_ctrl = invdyn(obj, q_j, dq_j, ddq_j)

        [t, stmChi] = fdyn(obj, tspan, fhCtrlTrqs, stvChi_0, ode_opt)

        wf_H_lnk = fkine(obj, q_j)

        wf_H_lnk = A(obj, jnt_idx, q_j)

        wf_H_ee = T0_n(obj, q_j) % computes the forward kinematics of the current end-effector.

        dJ = jacob_dot(obj, q_j, dq_j)

        J_0 = jacob0(obj, q_j, varargin)

        J_ee = jacobn(obj, q_j) % Jacobian of the current ee-frame.

        M = inertia(obj, q_j)

        resv = islimit(obj, q_j)

        plot3d(obj, x_out, sim_tstep, vis_ctrl)

    end
end
