classdef (Abstract) IWBM < handle
    properties(Abstract, Dependent)
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

    methods(Abstract)
        initRobot(obj, robot_wbm)

        initRobotFcn(obj, fhInitRobotWBM, wf2fixLnk)

        initBaseRobotParams(obj, robot_params)

        [vqT_b, q_j, v_b, dq_j] = getState(obj)

        stFltb = getFloatingBaseState(obj)

        ddq_j = jointAccelerations(obj, q_j, dq_j, tau, stFltb)

        ddq_j = jointAccelerationsExt(obj, q_j, dq_j, tau, stFltb)

        tau_c = coriolisForces(obj, q_j, dq_j, stFltb)

        tau_fr = frictionForces(obj, dq_j)

        C_qv = generalizedBiasForces(obj, q_j, dq_j, stFltb)

        tau_gen = generalizedForces(obj, q_j, dq_j, f_c, Jc_t, stFltb)

        tau_g = gravityForces(obj, q_j, stFltb)

        tau_j = inverseDyn(obj, q_j, dq_j, ddq_j, dv_b, stFltb)

        tau_j = inverseHybridDyn(obj, q_j, dq_j, ddq_j, stFltb)

        [t, stmChi] = forwardDyn(obj, tspan, fhTrqControl, stvChi_0, ode_opt, foot_conf)

        visualizeForwardDyn(obj, stmChi, sim_tstep, vis_ctrl)

        wf_H_lnk = forwardKin(obj, lnk_name, q_j, stFltb)

        wf_H_lnk = linkFrame(obj, lnk_name, q_j, stFltb) % link transformation matrix

        wf_H_tt = toolFrame(obj, t_idx, q_j, stFltb) % tool-tip transformation matrix

        M = massMatrix(obj, q_j, stFltb)

        h_c = centroidalMomentum(obj, q_j, dq_j, stFltb)

        [M, C_qv, h_c] = wholeBodyDyn(obj, q_j, dq_j, stFltb)

        J = jacobian(obj, lnk_name, q_j, stFltb)

        dJ = jacobianDot(obj, lnk_name, q_j, dq_j, stFltb)

        J_tt = jacobianTool(obj, t_idx, q_j, stFltb) % Jacobian matrix in tool-frame

        payload(obj, pt_mass, pos, link_names)

        % tau_pl = payloadForces(obj, q_j, dq_j, stFltb)

        resv = islimit(obj, q_j)

        dispParams(obj, prec)

    end
end
