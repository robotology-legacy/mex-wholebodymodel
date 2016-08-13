classdef (Abstract) IWBM < handle
    properties(Abstract, Dependent)
        % public properties for fast get/set methods:
        model_name@char
        robot_name@char
        robot_manuf@char
        robot_params@WBM.wbmBaseRobotParams
        sim_config@WBM.absSimConfig
        base_tform@double matrix
        tool_tform@double matrix
        gravity@double    vector
        jlimits@struct
        ndof@uint16       scalar
    end

    % are they useful for the iCub?
    % offset     kinematic joint coordinate offsets (Nx1)
    % theta       kinematic: joint angles (1xN)
    % d           kinematic: link offsets (1xN)
    % a           kinematic: link lengths (1xN)
    % alpha       kinematic: link twists (1xN)

    methods(Abstract)
        initRobot(robot_wbm)

        initRobotFcn(obj, fhInitRobotWBM, wf2fixLnk)

        initBaseRobotParams(obj, base_params)

        delete(obj)

        [vqT_b, q_j, v_b, dq_j] = getState(obj)

        stFltb = getFltgBase(obj)

        I_acc = Iqdd(obj, q_j, dq_j, tau, stFltb) % ?? inertial forces? (tau = generalized bias forces?)

        ddq_j = acceleration(obj, q_j, dq_j, tau, stFltb) % joint acceleration

        tau_c = coriolisForces(obj, q_j, dq_j, stFltb)

        tau_fr = frictionForces(obj, dq_j)

        C_qv = genBiasForces(obj, q_j, dq_j, stFltb)

        tau_g = gravityForces(obj, q_j, stFltb)

        tau_gen = invDyn(obj, q_j, dq_j, ddq_j, lnk_name, stFltb)

        [t, stmChi] = forwardDyn(obj, fhCtrlTrqs, tspan, stvChi_0, ode_opt)

        visForwardDyn(obj, x_out, sim_tstep, vis_ctrl)

        w_H_rlnk = forwardKin(obj, q_j, link_name, stFltb)

        wf_H_rlnk = T0_n(obj, q_j, lnk_name, stFltb) % ?? computes the forward kinematics for the end-effector?

        wf_H_rlnk = linkFrame(obj, joint_idx, q_j) % link transformation matrix
        %lnk_tform = linkFrame(obj, joint_idx, q_j) % link transformation matrix

        M = inertia(obj, q_j, stFltb)

        M_c = cartesianInertia(obj, q_j, stFltb) % ?? useful for the iCub?

        h_c = centroidalMomentum(obj, q_j, dq_j, stFltb)

        [M, C_qv, h_c] = wholeBodyDyn(obj, q_j, dq_j, stFltb)

        J = jacobian(obj, q_j, lnk_name, stFltb) % jacobian in the wolrd frame.

        dJ = jacobianDot(obj, q_j, dq_j, lnk_name, stFltb)

        J_tp = jacobianTool(obj, q_j, stFltb) % jacobian in the tool frame.

        payload(obj, pt_mass, pos, link_names)

        tau_pl = payloadForces(obj, ,stFltb)

        paycap() % ??

        gravjac() % ??

        resv = islimit(obj, q_j)

        dispParams(obj, prec)

        % set.offset(obj, v) % useful for the iCub?

        % v = get.offset(obj)

    end
end
