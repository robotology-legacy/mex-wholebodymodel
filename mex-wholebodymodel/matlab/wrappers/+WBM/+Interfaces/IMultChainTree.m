classdef (Abstract) IMultChainTree < handle
    properties(Abstract, Dependent)
        % public properties for fast get/set methods:
        name@char       % the name of the robot.
        manuf@char      % the name of the manufacturer (annotation)
        comment@char    % general comment (annotation)
        wbm_info@struct % general information about whole body model of the robot.
        wbm_params@WBM.wbmBaseRobotParams % base model and configuration parameters of the robot
        plotopt3d@WBM.absSimConfig
        %links
        link_name@char % current kinematic link of the robot that is controlled by the system.
        gravity@double vector  % gravity vector (direction of the gravity)
        base@double    matrix  % base transform of the robot (pose of the robot)
        tool@double    matrix  % tool transform (from the end-effector to the tool-tip)
        qlim@double    matrix  % joint limits, [qmin qmax] (Nx2)
        n@uint16       scalar  % number of joints (equivalent to number of DoFs)

        %interface % interface to a real robot platform
    end

    methods(Abstract)
        delete(obj)

        ddq_j = accel(obj, q_j, dq_j, tau)

        tau_c = corolis(obj, q_j, dq_j)

        tau_fr = friction(obj, dq_j)

        tau_g = gravload(obj, q_j)

        tau_gen = invdyn(obj, q_j, dq_j, ddq_j)

        [t, stmChi] = fdyn(obj, fhCtrlTrqs, tspan, stvChi_0, ode_opt)

        w_H_rlnk = fkine(obj, q_j)

        wf_H_rlnk = A(obj, joint_idx, q_j)

        %wf_H_rlnk = T0_n(obj, q_j, lnk_name) % ?? computes the forward kinematics for the end-effector?

        dJ = jacob_dot(obj, q_j, dq_j)

        J_0 = jacob0(obj, q_j, varargin)

        J_n = jacobn(obj, q_j)

        M = inertia(obj, q_j)

        M_c = cinertia(obj, q_j) % ?? cartesian inertia, useful for the iCub?

        payload(obj, pt_mass, pos, link_names)

        gravjac() % ??

        paycap() % ??

        tau_pl = pay(obj, )

        resv = islimit(obj, q_j)

        plot3d(obj, x_out, sim_tstep, vis_ctrl)

    end
end
