% Copyright (C) 2015-2018, by Martin Neururer
% Author: Martin Neururer
% E-mail: martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
% Date:   January, 2018
%
% Departments:
%   Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia and
%   Automation and Control Institute - TU Wien.
%
% This file is part of the Whole-Body Model Library for Matlab (WBML).
%
% The development of the WBM-Library was made in the context of the master
% thesis "Learning Task Behaviors for Humanoid Robots" and is an extension
% for the Matlab MEX whole-body model interface, which was supported by the
% FP7 EU project CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and
% Robotics (b)), <http://www.codyco.eu>.
%
% Permission is granted to copy, distribute, and/or modify the WBM-Library
% under the terms of the GNU Lesser General Public License, Version 2.1
% or any later version published by the Free Software Foundation.
%
% The WBM-Library is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU Lesser General Public License for more details.
%
% A copy of the GNU Lesser General Public License can be found along
% with the WBML. If not, see <http://www.gnu.org/licenses/>.

classdef WBM < WBM.WBMBase
    properties(Dependent)
        stvLen@uint16        scalar
        vqT_base@double      vector
        init_vqT_base@double vector
        init_stvChi@double   vector
        init_state@WBM.wbmStateParams
        robot_body@WBM.wbmBody
        robot_config@WBM.wbmRobotConfig
        robot_params@WBM.wbmRobotParams
    end

    properties(Constant)
        DF_STIFFNESS  = 2.5; % default control gain for the position correction.
        MAX_NUM_TOOLS = 2;

        MAX_JNT_SPEED = 250; % max. joint speed in [ksps] (kilosample(s) per second). (*)
        MAX_JNT_ACC   = 1e6; % max. joint acceleration with 1 [Ms/s^2] (Megasample(s) per second squared). (*)
        MAX_JNT_TRQ   = 1e5; % max. joint torque in [ksps] (kilosample(s) per second).
        % (*) Source: <http://wiki.icub.org/brain/velControlThread_8cpp.html>

        % zero-vectors for contact accelerations/velocities
        % and for external force vectors:
        ZERO_CVEC_12 = zeros(12,1);
        ZERO_CVEC_6  = zeros(6,1);
    end

    properties(Access = protected)
        mwbm_config@WBM.wbmRobotConfig
        mwf2fixlnk@logical scalar
    end

    methods
        % Constructor:
        function obj = WBM(robot_model, robot_config, wf2fixlnk)
            % call the constructor of the superclass ...
            obj = obj@WBM.WBMBase(robot_model);

            switch nargin
                case 3
                    obj.mwf2fixlnk = wf2fixlnk;
                case 2
                    % set default value ...
                    obj.mwf2fixlnk = false;
                otherwise
                    error('WBM::WBM: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end

            initConfig(obj, robot_config);
            if obj.mwf2fixlnk
                if ~isempty(obj.mwbm_model.urdf_fixed_link)
                    % use the previously set fixed link of the robot model ...
                    updateWorldFrameFromFixLnk(obj);
                elseif (obj.mwbm_config.nCstrs > 0)
                    % set the world frame (WF) at the initial VQ-transformation of the chosen
                    % fixed link. In this case the first entry of the contact constraint list:
                    setWorldFrameAtFixLnk(obj, obj.mwbm_config.ccstr_link_names{1,1});
                else
                    error('WBM::WBM: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
                end
            end
            % retrieve and update the initial VQ-transformation of the robot base (world frame) ...
            updateInitVQTransformation(obj);
        end

        % Copy-function:
        function newObj = copy(obj)
            newObj = copy@WBM.WBMBase(obj);
        end

        % Destructor:
        function delete(obj)
            delete@WBM.WBMBase(obj);
        end

        function setWorldFrameAtFixLnk(obj, urdf_fixed_link, q_j, dq_j, v_b, g_wf)
            if (nargin < 6)
                switch nargin
                    case 5
                        % use the default gravity vector ...
                        g_wf = obj.mwbm_model.g_wf;
                    case 2
                        % use the initial state values (possibly changed from outside) ...
                        v_b  = vertcat(obj.mwbm_config.init_state_params.dx_b, obj.mwbm_config.init_state_params.omega_b);
                        q_j  = obj.mwbm_config.init_state_params.q_j;
                        dq_j = obj.mwbm_config.init_state_params.dq_j;
                        g_wf = obj.mwbm_model.g_wf;
                    otherwise
                        error('WBM::setWorldFrameAtFixLnk: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
                end
            end
            obj.fixed_link = urdf_fixed_link; % replace the old fixed link with the new one ...

            setState(obj, q_j, dq_j, v_b); % update the robot state (important for initializations) ...
            [wf_p_b, wf_R_b] = getWorldFrameFromFixLnk(obj, urdf_fixed_link); % use optimized mode
            setWorldFrame(obj, wf_R_b, wf_p_b, g_wf);
        end

        function updateWorldFrameFromFixLnk(obj, q_j, dq_j, v_b, g_wf)
            if (nargin < 5)
                switch nargin
                    case 4
                        % use the default gravity values ...
                        g_wf = obj.mwbm_model.g_wf;
                    case 1
                        % use the initial state values (possibly changed from outside) ...
                        v_b  = vertcat(obj.mwbm_config.init_state_params.dx_b, obj.mwbm_config.init_state_params.omega_b);
                        q_j  = obj.mwbm_config.init_state_params.q_j;
                        dq_j = obj.mwbm_config.init_state_params.dq_j;
                        g_wf = obj.mwbm_model.g_wf;
                    otherwise
                        error('WBM::updateWorldFrameFromFixLnk: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
                end
            end
            setState(obj, q_j, dq_j, v_b); % update state ...
            [wf_p_b, wf_R_b] = getWorldFrameFromDfltFixLnk(obj); % optimized mode
            setWorldFrame(obj, wf_R_b, wf_p_b, g_wf); % update the world frame with the new values ...
        end

        function updateInitVQTransformation(obj)
            vqT_init = obj.vqT_base; % get the vector-quaternion transf. of the current state ...
            obj.mwbm_config.init_state_params.x_b  = vqT_init(1:3,1); % translation/position
            obj.mwbm_config.init_state_params.qt_b = vqT_init(4:7,1); % orientation (quaternion)
        end

        function vqT_lnk = fkinVQTransformation(obj, urdf_link_name, q_j, vqT_b, g_wf)
            % computes the forward kinematic vector-quaternion transf. of a specified link frame:
            % set the world frame at the given base-to-world transformation (base frame) ...
            switch nargin
                case 5
                    [wf_p_b, wf_R_b] = WBM.utilities.tfms.frame2posRotm(vqT_b); % pos. & orientation from the base frame ...
                    setWorldFrame(obj, wf_R_b, wf_p_b, g_wf);
                case 4
                    [wf_p_b, wf_R_b] = WBM.utilities.tfms.frame2posRotm(vqT_b);
                    setWorldFrame(obj, wf_R_b, wf_p_b); % use the default gravity vector ...
                otherwise
                    error('WBM::fkinVQTransformation: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % compute the forward kinematics of the given link frame ...
            vqT_lnk = forwardKinematics(obj, wf_R_b, wf_p_b, q_j, urdf_link_name);
        end

        [Jc, djcdq] = contactJacobians(obj, varargin)

        function [f_c, tau_gen] = contactForces(obj, tau, Jc, djcdq, M, c_qv, dq_j)
            switch nargin
                case 7
                    % generalized forces with friction:
                    tau_fr  = frictionForces(obj, dq_j);         % friction torques (negated torque values)
                    tau_gen = vertcat(zeros(6,1), tau + tau_fr); % generalized forces tau_gen = S_j*(tau + (-tau_fr)),
                                                                 % S_j = [0_(6xn); I_(nxn)] ... joint selection matrix
                case 6
                    % general case:
                    tau_gen = vertcat(zeros(6,1), tau);
                otherwise
                    error('WBM::contactForces: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % Calculation of the contact (constraint) force vector:
            % For further details about the formula see,
            %   [1] Control Strategies for Robots in Contact, J. Park, PhD-Thesis, Artificial Intelligence Laboratory, Stanford University, 2006,
            %       <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>, Chapter 5, pp. 106-110, eq. (5.5)-(5.14).
            %   [2] A Mathematical Introduction to Robotic Manipulation, Murray & Li & Sastry, CRC Press, 1994, pp. 269-270, eq. (6.5) & (6.6).
            Jc_t      = Jc.';
            JcMinv    = Jc / M; % = Jc * M^(-1)
            Upsilon_c = JcMinv * Jc_t; % inverse mass matrix Upsilon_c = Lambda^(-1) = Jc * M^(-1) * Jc^T in contact space {c},
                                       % Lambda^(-1) ... inverse pseudo-kinetic energy matrix.
            % contact constraint forces f_c (generated by the environment):
            f_c = Upsilon_c \ (JcMinv*(c_qv - tau_gen) - djcdq);
            % (this calculation method is numerically more accurate and robust than the calculation variant with the cartmass-function.)
        end

        [f_c, tau_gen] = contactForcesEF(obj, tau, fe_c, ac, Jc, djcdq, M, c_qv, dq_j) % EF ... External Forces at the contact links (without pose corrections)

        [f_c, tau_gen] = contactForcesCLPCEF(obj, clnk_conf, tau, fe_c, ac, Jc, djcdq, M, c_qv, varargin) % CLPCEF ... Contact Link Pose Corrections with External Forces
                                                                                                          %            (at the contact links)

        function [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCS(obj, clnk_conf, varargin) % in dependency of the contact state (CS)
            % wf_R_b_arr = varargin{1}
            % wf_p_b     = varargin{2}
            % q_j        = varargin{3}
            % dq_j       = varargin{4}
            % v_b        = varargin{5}

            % check which link is in contact with the ground/object and calculate the
            % multibody dynamics and the corresponding the contact Jacobians:
            clink_idx = getContactIdx(obj, clnk_conf);
            if ~clink_idx
                % both links have no contact to the ground/object ...
                n = obj.mwbm_model.ndof + 6;
                [M, c_qv] = wholeBodyDyn(obj, varargin{:});
                Jc    = zeros(12,n);
                djcdq = obj.ZERO_CVEC_12;
                return
            end
            n = size(varargin,2);
            varargin{1,n+1} = clink_idx; % = idx_list

            [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCC(obj, varargin{:});
        end

        function [Jc, djcdq] = contactJacobiansCS(obj, clnk_conf, varargin) % in dependency of the contact state (CS)
            % wf_R_b_arr = varargin{1}
            % wf_p_b     = varargin{2}
            % q_j        = varargin{3}
            % dq_j       = varargin{4}
            % v_b        = varargin{5}

            clink_idx = getContactIdx(obj, clnk_conf);
            if ~clink_idx
                % both links have no contact to the ground/object ...
                n = obj.mwbm_model.ndof + 6;
                Jc    = zeros(12,n);
                djcdq = obj.ZERO_CVEC_12;
                return
            end
            % compute the contact Jacobians ...
            n = size(varargin,2);
            varargin{1,n+1} = clink_idx; % = idx_list

            [Jc, djcdq] = contactJacobians(obj, varargin{:});
        end

        function [ddq_j, fd_prms] = jointAccelerations(obj, tau, varargin)
            switch nargin
                case 7 % normal modes:
                    % generalized forces with friction:
                    % wf_R_b = varargin{1}
                    % wf_p_b = varargin{2}
                    % q_j    = varargin{3}
                    % v_b    = varargin{5}
                    dq_j = varargin{1,4};

                    % compute the whole body dynamics and for every contact constraint
                    % the Jacobian and the derivative Jacobian ...
                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCC(obj, wf_R_b_arr, varargin{1,2}, ...
                                                               varargin{1,3}, dq_j, varargin{1,5});
                    % get the contact forces and the corresponding generalized forces ...
                    [f_c, tau_gen] = contactForces(obj, tau, Jc, djcdq, M, c_qv, dq_j);
                case 6
                    % without friction:
                    % wf_R_b = varargin{1}
                    % wf_p_b = varargin{2}
                    % q_j    = varargin{3}
                    nu = varargin{1,4};

                    len  = obj.mwbm_model.ndof + 6;
                    dq_j = nu(7:len,1);
                    v_b  = nu(1:6,1);

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCC(obj, wf_R_b_arr, varargin{1,2}, ...
                                                               varargin{1,3}, dq_j, v_b);
                    [f_c, tau_gen] = contactForces(obj, tau, Jc, djcdq, M, c_qv);
                case 3 % optimized modes:
                    % with friction:
                    % dq_j = varargin{1}
                    [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCC(obj);
                    [f_c, tau_gen] = contactForces(obj, tau, Jc, djcdq, M, c_qv, varargin{1,1});
                case 2
                    % without friction:
                    [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCC(obj);
                    [f_c, tau_gen] = contactForces(obj, tau, Jc, djcdq, M, c_qv);
                otherwise
                    error('WBM::jointAccelerations: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end

            % Joint Acceleration q_ddot (derived from the state-space equation):
            % For further details see:
            %   [1] Efficient Dynamic Simulation of Robotic Mechanisms, K. Lilly, Springer, 1992, p. 82, eq. (5.2).
            Jc_t  = Jc.';
            ddq_j = M \ (tau_gen + Jc_t*f_c - c_qv); % ddq_j = M^(-1) * (tau - c_qv - Jc.'*(-f_c))

            if (nargout == 2)
                % set the forward dynamics parameters ...
                fd_prms = struct('tau_gen', tau_gen, 'f_c', f_c);
            end
        end

        [ddq_j, fd_prms] = jointAccelerationsEF(obj, foot_conf, clnk_conf, tau, fe_c, ac, varargin) % EF ... External Forces at the contact links (no pose corrections)

        [ddq_j, fd_prms] = jointAccelerationsPL(obj, foot_conf, hand_conf, tau, fhTotCWrench, f_cp, varargin) % PL ... PayLoad at the hands (no pose corrections)

        [ddq_j, fd_prms] = jointAccelerationsNFB(obj, tau, varargin) % NFB ... No Floating Base (no pose corrections)

        [ddq_j, fd_prms] = jointAccelerationsNFBEF(obj, clnk_conf, tau, fe_c, ac, varargin) % NFBEF ... No Floating Base, with External Forces at the contact links
                                                                                            %           (no pose corrections)

        [ddq_j, fd_prms] = jointAccelerationsNFBPL(obj, hand_conf, tau, fhTotCWrench, f_cp, varargin) % NFBPL ... No Floating Base, with PayLoad at the hands
                                                                                                      %           (no pose corrections)

        [ddq_j, fd_prms] = jointAccelerationsCLPCEF(obj, clnk_conf, tau, fe_c, ac, varargin) % CLPCEF ... Contact Link Pose Corrections with External Forces
                                                                                             %            (at the contact links)

        function [ddq_j, fd_prms] = jointAccelerationsFPC(obj, foot_conf, tau, ac_f, varargin) % FPC ... Foot Pose Corrections (no external forces)
            fe_0 = zeroExtForces(obj, foot_conf);
            if (nargout == 2)
                [ddq_j, fd_prms] = jointAccelerationsCLPCEF(obj, foot_conf, tau, fe_0, ac_f, varargin{:});
                return
            end
            % else ...
            ddq_j = jointAccelerationsCLPCEF(obj, foot_conf, tau, fe_0, ac_f, varargin{:});
        end

        [ddq_j, fd_prms] = jointAccelerationsFPCEF(obj, foot_conf, clnk_conf, tau, fe_c, ac, varargin) % FPCEF ... Foot Pose Corrections with External Forces (at the contact links)

        [ddq_j, fd_prms] = jointAccelerationsFPCPL(obj, foot_conf, hand_conf, tau, fhTotCWrench, f_cp, varargin) % FPCPL ... Foot Pose Corrections with PayLoad (at the hands)

        function [ddq_j, fd_prms] = jointAccelerationsHPCEF(obj, hand_conf, tau, fe_h, ac_h, varargin) % HPCEF ... Hand Pose Corrections with External Forces (at the hands)
            if (nargout == 2)
                [ddq_j, fd_prms] = jointAccelerationsCLPCEF(obj, hand_conf, tau, fe_h, ac_h, varargin{:});
                return
            end
            % else ...
            ddq_j = jointAccelerationsCLPCEF(obj, hand_conf, tau, fe_h, ac_h, varargin{:});
        end

        [ddq_j, fd_prms] = jointAccelerationsFHPCEF(obj, foot_conf, hand_conf, tau, fe_h, ac_h, varargin) % FHPCEF ... Foot & Hand Pose Corrections with External Forces (at the hands)

        [ddq_j, fd_prms] = jointAccelerationsFHPCPL(obj, foot_conf, hand_conf, tau, fhTotCWrench, f_cp, varargin) % FHPCPL ... Foot & Hand Pose Corrections with PayLoad (at the hands)

        [ac_h, a_prms] = handAccelerations(obj, foot_conf, hand_conf, tau, varargin)

        [vc_h, v_prms] = handVelocities(obj, hand_conf, varargin)

        dstvChi = forwardDynamics(obj, t, stvChi, fhTrqControl)

        dstvChi = forwardDynamicsEF(obj, t, stvChi, fhTrqControl, foot_conf, clnk_conf, fe_c, ac, ac_f)

        dstvChi = forwardDynamicsPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, f_cp, ac_f)

        dstvChi = forwardDynamicsNFB(obj, t, stvChi, fhTrqControl)

        dstvChi = forwardDynamicsNFBEF(obj, t, stvChi, fhTrqControl, clnk_conf, fe_c, ac)

        dstvChi = forwardDynamicsNFBPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, hand_conf, f_cp)

        dstvChi = forwardDynamicsFPC(obj, t, stvChi, fhTrqControl, foot_conf, ac_f)

        dstvChi = forwardDynamicsFPCEF(obj, t, stvChi, fhTrqControl, foot_conf, clnk_conf, fe_c, ac, ac_f)

        dstvChi = forwardDynamicsFPCPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, f_cp, ac_f)

        dstvChi = forwardDynamicsHPCEF(obj, t, stvChi, fhTrqControl, hand_conf, fe_h, ac_h)

        dstvChi = forwardDynamicsFHPCEF(obj, t, stvChi, fhTrqControl, foot_conf, hand_conf, fe_h, ac_h, ac_f)

        dstvChi = forwardDynamicsFHPCPL(obj, t, stvChi, fhTrqControl, fhTotCWrench, foot_conf, hand_conf, f_cp, ac_f)

        [t, stmChi] = intForwardDynamics(obj, tspan, stvChi_0, fhTrqControl, ode_opt, varargin)

        function ac_0 = zeroCtcAcc(obj, clnk_conf)
            nctc = uint8(clnk_conf.contact.left) + uint8(clnk_conf.contact.right);
            switch nctc
                case 1
                    ac_0 = obj.ZERO_CVEC_6;
                otherwise % if nctc = 0 or nctc = 2:
                    % either both contact links have contact with the ground/object, or
                    % both contact links have no contact to the ground/object ...
                    ac_0 = obj.ZERO_CVEC_12;
            end
        end

        function fe_0 = zeroExtForces(obj, clnk_conf)
            nctc = uint8(clnk_conf.contact.left) + uint8(clnk_conf.contact.right);
            switch nctc
                case 1
                    fe_0 = obj.ZERO_CVEC_6;
                otherwise % if nctc = 0 or nctc = 2:
                    fe_0 = obj.ZERO_CVEC_12;
            end
        end

        clnk_conf = clnkConfigState(obj, varargin)

        function foot_conf = footConfigState(obj, cstate, varargin)
            foot_conf = createConfigStateCL(obj, cstate, 'l_sole', 'r_sole', varargin{:});
        end

        function hand_conf = handConfigState(obj, cstate, cmode, varargin)
            switch cmode
                case 'hand'
                    % hand palms (hand link frame):
                    clnk_l = 'l_hand';
                    clnk_r = 'r_hand';
                case 'gripper'
                    % finger tips (hand_dh_frame):
                    clnk_l = 'l_gripper';
                    clnk_r = 'r_gripper';
                otherwise
                    error('WBM::handConfigState: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
            end
            hand_conf = createConfigStateCL(obj, cstate, clnk_l, clnk_r, varargin{:});
        end

        vis_data = getFDynVisData(obj, stmChi, fhTrqControl, varargin)

        function vis_data = getFDynVisDataSect(obj, stmChi, fhTrqControl, start_idx, end_idx, varargin)
            if ( (start_idx < 1)   || (end_idx < 1)   || ...
                 (start_idx > len) || (end_idx > len) || (start_idx > end_idx) )
                error('WBM::getFDynVisDataSect: %s', WBM.wbmErrorMsg.VAL_OUT_OF_BOUNDS);
            end
            vis_data = getFDynVisData(obj, stmChi(start_idx:end_idx,:), fhTrqControl, varargin{:});
        end

        sim_config = setupSimulation(~, sim_config, rot3d)

        [] = visualizeForwardDynamics(obj, stmPos, sim_config, sim_tstep, vis_ctrl)

        function simulateForwardDynamics(obj, stmPos, sim_config, sim_tstep, nRpts, vis_ctrl)
            if ( sim_config.mkvideo && (nRpts > 1) ), nRpts = 1; end
            if ~exist('vis_ctrl', 'var')
                % use the default vis-ctrl values ...
                for i = 1:nRpts
                    visualizeForwardDynamics(obj, stmPos, sim_config, sim_tstep);
                end
                return
            end
            % else ...
            for i = 1:nRpts
                visualizeForwardDynamics(obj, stmPos, sim_config, sim_tstep, vis_ctrl);
            end
        end

        lnk_traj = setTrajectoriesData(obj, lnk_traj, stmPos, start_idx, end_idx)

        function plotCoMTrajectory(obj, stmPos, prop)
            if ~exist('prop', 'var')
                % use the default plot properties ...
                prop.fwnd_title   = 'iCub - CoM-trajectory:';
                prop.title        = '';
                prop.title_fnt_sz = 15;
                prop.line_color   = 'blue';
                prop.spt_marker   = '*';
                prop.spt_color    = 'red';
                prop.label_fnt_sz = 15;
            end

            [m, n] = size(stmPos);
            if (n == 3)
                x_b = stmPos;
            elseif (n == (obj.mwbm_model.ndof + 7))
                % extract all base position values ...
                x_b = stmPos(1:m,1:3);
            else
                error('WBM::plotCoMTrajectory: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            figure('Name', prop.fwnd_title, 'NumberTitle', 'off');

            % draw the trajectory-line:
            %         x-axis      y-axis      z-axis
            plot3(x_b(1:m,1), x_b(1:m,2), x_b(1:m,3), 'Color', prop.line_color);
            hold on;
            % mark the start point ...
            hsp = plot3(x_b(1,1), x_b(1,2), x_b(1,3), 'LineStyle', 'none', ...
                        'Marker', prop.spt_marker, 'MarkerEdgeColor', prop.spt_color);
            axis square;
            grid on;

            % add title and axis-lables ...
            if ~isempty(prop.title)
                title(prop.title, 'Interpreter', 'latex', 'FontSize', prop.title_fnt_sz);
            end
            xlabel('$x_{\mathbf{x_b}}$', 'Interpreter', 'latex', 'FontSize', prop.label_fnt_sz);
            ylabel('$y_{\mathbf{x_b}}$', 'Interpreter', 'latex', 'FontSize', prop.label_fnt_sz);
            zlabel('$z_{\mathbf{x_b}}$', 'Interpreter', 'latex', 'FontSize', prop.label_fnt_sz);

            legend(hsp, 'init. position', 'Location', 'northeast');
        end

        function setPayloadLinks(obj, pl_lnk_data)
            % verify the input data ...
            if isempty(pl_lnk_data)
                error('WBM::setPayloadLinks: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end
            if isstruct(pl_lnk_data)
                pl_lnk_data = {pl_lnk_data};
            elseif ( iscell(pl_lnk_data) && isstruct(pl_lnk_data{1,1}) )
                pl_lnk_data = pl_lnk_data(:);
            else
                error('WBM::setPayloadLinks: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % Note: Index 1 and 2 are always reserved for the left and right hand of the
            % humanoid robot. The default index is always 1. All further indices can be
            % used for other links, on which additionally special payloads are mounted
            % (e.g. battery pack or knapsack at torso, tools, etc.).
            n = size(pl_lnk_data,1);
            obj.mwbm_config.nPlds = n; % number of payloads ...

            lnk_names_lh = {'l_hand', 'l_gripper', 'l_wrist_1'};
            lnk_names_rh = {'r_hand', 'r_gripper', 'r_wrist_1'};
            lh = ~isempty(find(ismember(pl_lnk_data{1,1}.name, lnk_names_lh), 1));
            if (n > 1)
                rh = ~isempty(find(ismember(pl_lnk_data{2,1}.name, lnk_names_rh), 1));
            else % if n = 1:
                rh = ~isempty(find(ismember(pl_lnk_data{1,1}.name, lnk_names_rh), 1));
            end

            if (lh && rh)
                len = n;
                obj.mwbm_config.payload_links(1,1:n) = WBM.wbmPayloadLink;
                setPayloadLinkData(obj, 1, pl_lnk_data{1,1});
                setPayloadLinkData(obj, 2, pl_lnk_data{2,1});
            elseif lh
                len = n + 1;
                obj.mwbm_config.payload_links(1,1:len) = WBM.wbmPayloadLink;
                setPayloadLinkData(obj, 1, pl_lnk_data{1,1});
            elseif rh
                len = n + 1;
                obj.mwbm_config.payload_links(1,1:len) = WBM.wbmPayloadLink;
                setPayloadLinkData(obj, 2, pl_lnk_data{1,1});
            else
                len = n + 2;
                obj.mwbm_config.payload_links(1,1:len) = WBM.wbmPayloadLink;
            end

            if (len > 2)
                for i = 3:len
                    pl_lnk = pl_lnk_data{i,1};
                    setPayloadLinkData(obj, i, pl_lnk);
                end
            end
        end

        function [pl_links, nPlds] = getPayloadLinks(obj)
            pl_links = obj.mwbm_config.payload_links;
            nPlds    = obj.mwbm_config.nPlds;
        end

        function pl_tbl = getPayloadTable(obj)
            nPlds = obj.mwbm_config.nPlds;
            if (nPlds == 0)
                pl_tbl = table(); % empty table ...
                return
            end

            pl_links   = obj.mwbm_config.payload_links;
            clnk_names = cell(nPlds,1);
            cpos       = clnk_names;
            tidx       = zeros(nPlds,1);
            vbidx      = tidx;
            mass       = tidx;
            cinert     = clnk_names;

            for i = 1:nPlds
                clnk_names{i,1} = pl_links(1,i).urdf_link_name;
                cpos{i,1}       = pl_links(1,i).lnk_p_cm;
                tidx(i,1)       = pl_links(1,i).t_idx;
                vbidx(i,1)      = pl_links(1,i).vb_idx;
                mass(i,1)       = pl_links(1,i).m_rb;
                cinert{i,1}     = pl_links(1,i).I_cm;
            end
            cplds  = horzcat(clnk_names, cpos, num2cell(tidx), num2cell(vbidx), num2cell(mass), cinert);
            pl_tbl = cell2table(cplds, 'VariableNames', {'link_name', 'lnk_p_cm', 't_idx', 'vb_idx', 'mass', 'inertia'});
        end

        function wf_H_cm = payloadFrame(obj, varargin)
            if (obj.mwbm_config.nPlds == 0)
                error('WBM::payloadFrame: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            switch nargin
                case 5 % normal modes:
                    pl_idx      = varargin{1,4};
                    pl_lnk_name = obj.mwbm_config.payload_links(1,pl_idx).urdf_link_name;
                    lnk_p_cm    = obj.mwbm_config.payload_links(1,pl_idx).lnk_p_cm;

                    WBM.utilities.chkfun.checkLinkName(pl_lnk_name, 'WBM::payloadFrame');

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_lnk = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, pl_lnk_name);
                case 4
                    % use the values of the default payload-link ...
                    lnk_p_cm = obj.mwbm_config.payload_links(1,1).lnk_p_cm;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_lnk = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, ...
                                                 obj.mwbm_config.payload_links(1,1).urdf_link_name);
                case 2 % optimized modes:
                    pl_idx      = varargin{1,1};
                    pl_lnk_name = obj.mwbm_config.payload_links(1,pl_idx).urdf_link_name;
                    lnk_p_cm    = obj.mwbm_config.payload_links(1,pl_idx).lnk_p_cm;

                    WBM.utilities.chkfun.checkLinkName(pl_lnk_name, 'WBM::payloadFrame');

                    wf_H_lnk = mexWholeBodyModel('transformation-matrix', pl_lnk_name);
                case 1
                    lnk_p_cm = obj.mwbm_config.payload_links(1,1).lnk_p_cm;
                    wf_H_lnk = mexWholeBodyModel('transformation-matrix', obj.mwbm_config.payload_links(1,1).urdf_link_name);
                otherwise
                    error('WBM::payloadFrame: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % Transformation: We assume that the orientation of the payload-frame {pl} has the same
            %                 orientation as the frame {lnk} of a given link, i.e. the link of an
            %                 end-effector (hand, finger, etc.) or an arbitrary link (torso, leg, etc.)
            %                 where the payload is mounted on that body part.
            %
            % get the homog. transformation of the payload-frame centered at the CoM (relative to the link-frame):
            lnk_H_cm = eye(4,4);
            lnk_H_cm(1:3,4) = lnk_p_cm; % position from the payload's CoM to the frame {lnk}.

            wf_H_cm = wf_H_lnk * lnk_H_cm; % payload transformation matrix
        end

        f_pl = handPayloadForces(obj, hand_conf, fhTotCWrench, f_cp, wf_v_lnk, wf_a_lnk)

        function f_pl = dynPayloadForce(obj, pl_idx, wf_v_lnk, wf_a_lnk, w_e)
            if (nargin == 4)
                w_e = zeros(6,1);
            end
            pl_lnk_name = obj.mwbm_config.payload_links(1,pl_idx).urdf_link_name;
            WBM.utilities.chkfun.checkLinkName(pl_lnk_name, 'WBM::dynPayloadForce');

            wf_H_lnk     = transformationMatrix(obj, pl_lnk_name); % optimized mode
            [~,wf_R_lnk] = WBM.utilities.tfms.tform2posRotm(wf_H_lnk);

            % mixed velocity transformation:
            lnk_R_wf = wf_R_lnk.';
            lnk_X_wf = WBM.utilities.tfms.mixveltfm(lnk_R_wf, lnk_R_wf);

            % get the payload's acceleration & velocity from the link frame:
            a_pl = lnk_X_wf * wf_a_lnk; % = a_lnk = I_6 * cm_a_lnk
            v_pl = lnk_X_wf * wf_v_lnk; % = v_lnk = I_6 * cm_v_lnk

            % calculate the affecting force of the payload object
            % at the current contact point p_c:
            M_pl = generalizedInertiaPL(obj, pl_idx); % = lnk_M_cm
            f_pl = WBM.utilities.mbd.payloadWrenchRB(M_pl, v_pl, a_pl, w_e); % = f_lnk
        end

        function M_pl = generalizedInertiaPL(obj, pl_idx)
            m_rb     = obj.mwbm_config.payload_links(1,pl_idx).m_rb;
            lnk_p_cm = obj.mwbm_config.payload_links(1,pl_idx).lnk_p_cm;

            % Apply a simplified position/orientation estimation for the payload's CoM:
            %
            %       lnk_R_cm = lnk_R_c * c_R_cm   and
            %       lnk_p_cm = lnk_R_c * c_p_cm,
            %
            % where lnk_R_c = c_R_cm = I_3 (identity matrix) and lnk_p_cm = c_p_cm.
            % The contact frame {c} and the payload frame {cm} (centered at CoM)
            % have the same orientation as the reference link frame {lnk} of the
            % given end-effector (hand). The contact point p_c is set at the
            % origin o_lnk of the link frame {lnk}.
            %
            % Note: The given position for the object's CoM must be from the
            %       same reference frame {lnk}.
            if (m_rb == 0)
                M_pl = zeros(6,6);
            else
                % generalized inertia lnk_M_cm of the payload object at
                % the contact point p_c = lnk_p_cm with lnk_R_cm = I_3:
                I_cm = obj.mwbm_config.payload_links(1,pl_idx).I_cm;
                M_pl = WBM.utilities.rb.generalizedInertia(m_rb, I_cm, lnk_p_cm);
            end
        end

        function setToolLinks(obj, ee_lnk_names, ee_vqT_tt, pl_idx)
            % verify the input types ...
            if ( ~iscellstr(ee_lnk_names) || ~ismatrix(ee_vqT_tt) )
                error('WBM::setToolLinks: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % check dimensions ...
            [m, n] = size(ee_vqT_tt);
            if (m ~= 7)
                error('WBM::setToolLinks: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            if (n > obj.MAX_NUM_TOOLS)
                error('WBM::setToolLinks: %s', WBM.wbmErrorMsg.MAX_NUM_LIMIT);
            end
            WBM.utilities.chkfun.checkRVecDim(ee_lnk_names, n, 'WBM::setToolLinks');

            % set the end-effector (ee) link names and the tool-tip (tt) frames:
            % if some grabbed payload objects are also tools, link the tool links
            % with the corresponding payload objects.
            obj.mwbm_config.nTools = n; % number of tools ...
            obj.mwbm_config.tool_links(1:n,1) = WBM.wbmToolLink;

            if (nargin == 4)
                % the grabbed payload objects are also tools (at least one) ...
                WBM.utilities.chkfun.checkRVecDim(pl_idx, n, 'WBM::setToolLinks');
            else
                % no payload object is defined as a tool ...
                pl_idx = zeros(1,n);
            end

            for i = 1:n
                obj.mwbm_config.tool_links(i,1).urdf_link_name = ee_lnk_names{1,i};
                obj.mwbm_config.tool_links(i,1).ee_vqT_tt      = ee_vqT_tt(1:7,i);
                obj.mwbm_config.tool_links(i,1).pl_idx         = pl_idx(1,i); % payload link index
            end
        end

        function [tool_links, nTools] = getToolLinks(obj)
            tool_links = obj.mwbm_config.tool_links;
            nTools     = obj.mwbm_config.nTools;
        end

        function tool_tbl = getToolTable(obj)
            nTools = obj.mwbm_config.nTools;
            if (nTools == 0)
                tool_tbl = table(); % empty table ...
                return
            end

            tool_links = obj.mwbm_config.tool_links;
            clnk_names = cell(nTools,1);
            cfrms      = clnk_names;
            cpidx      = clnk_names;

            for i = 1:nTools
                clnk_names{i,1} = tool_links(i,1).urdf_link_name;
                cfrms{i,1}      = tool_links(i,1).ee_vqT_tt;
                cpidx{i,1}      = tool_links(i,1).pl_idx;
            end
            ctools = horzcat(clnk_names, cfrms, cpidx);

            tool_tbl = cell2table(ctools, 'VariableNames', {'link_name', 'ee_vqT_tt', 'pl_idx'});
        end

        function updateToolFrame(obj, ee_vqT_tt, t_idx)
            if (obj.mwbm_config.nTools == 0)
                error('WBM::updateToolFrame: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end
            if (t_idx > obj.MAX_NUM_TOOLS)
                error('WBM::updateToolFrame: %s', WBM.wbmErrorMsg.MAX_NUM_LIMIT);
            end
            if (size(ee_vqT_tt,1) ~= 7)
                error('WBM::updateToolFrame: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            % update the tool-frame (VQ-transformation) of the selected tool with the new frame ...
            obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt = ee_vqT_tt;
        end

        function wf_H_tt = toolFrame(obj, varargin)
            if (obj.mwbm_config.nTools == 0)
                error('WBM::toolFrame: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end

            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            switch nargin
                case 5 % normal modes:
                    t_idx       = varargin{1,4};
                    ee_lnk_name = obj.mwbm_config.tool_links(t_idx,1).urdf_link_name;
                    ee_vqT_tt   = obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_ee = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, ee_lnk_name);
                case 4
                    % use the values of the default tool link (1st element of the list) ...
                    ee_vqT_tt = obj.mwbm_config.tool_links(1,1).ee_vqT_tt;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_ee = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, ...
                                                obj.mwbm_config.tool_links(1,1).urdf_link_name);
                case 2 % optimized modes:
                    t_idx       = varargin{1,1};
                    ee_lnk_name = obj.mwbm_config.tool_links(t_idx,1).urdf_link_name;
                    ee_vqT_tt   = obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt;

                    wf_H_ee = mexWholeBodyModel('transformation-matrix', ee_lnk_name);
                case 1
                    % use the default tool link ...
                    ee_vqT_tt = obj.mwbm_config.tool_links(1,1).ee_vqT_tt;
                    wf_H_ee   = mexWholeBodyModel('transformation-matrix', obj.mwbm_config.tool_links(1,1).urdf_link_name);
                otherwise
                    error('WBM::toolFrame: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % Transformation: We assume the general case, that the orientation of the tool-frame (tt)
            %                 does not have the same orientation as the frame of the end-effector (ee),
            %                 i.e. the frame of a hand (or of a finger).
            %
            % get the homog. transformation of the tool-frame (relative to the ee-frame):
            ee_H_tt = WBM.utilities.tfms.frame2tform(ee_vqT_tt);
            wf_H_tt = wf_H_ee * ee_H_tt; % tool transformation matrix
        end

        function wf_J_tt = jacobianTool(obj, varargin) % Jacobian matrix in tool-frame
            if (obj.mwbm_config.nTools == 0)
                error('WBM::jacobianTool: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end

            % wf_R_b = varargin{1}
            switch nargin
                case 5 % normal modes:
                    wf_p_b = varargin{1,2};
                    q_j    = varargin{1,3};
                    t_idx  = varargin{1,4};

                    ee_lnk_name = obj.mwbm_config.tool_links(t_idx,1).urdf_link_name;
                    ee_vqT_tt   = obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_ee = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, wf_p_b, q_j, ee_lnk_name);
                    wf_J_ee = mexWholeBodyModel('jacobian', wf_R_b_arr, wf_p_b, q_j, ee_lnk_name);
                case 4
                    % use the values of the default tool-link (1st element of the list) ...
                    wf_p_b = varargin{1,2};
                    q_j    = varargin{1,3};

                    ee_lnk_name = obj.mwbm_config.tool_links(1,1).urdf_link_name;
                    ee_vqT_tt   = obj.mwbm_config.tool_links(1,1).ee_vqT_tt;

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    wf_H_ee = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, wf_p_b, q_j, ee_lnk_name);
                    wf_J_ee = mexWholeBodyModel('jacobian', wf_R_b_arr, wf_p_b, q_j, ee_lnk_name);
                case 2 % optimized modes:
                    t_idx = varargin{1,1};

                    ee_lnk_name = obj.mwbm_config.tool_links(t_idx,1).urdf_link_name;
                    ee_vqT_tt   = obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt;

                    wf_H_ee = mexWholeBodyModel('transformation-matrix', ee_lnk_name);
                    wf_J_ee = mexWholeBodyModel('jacobian', ee_lnk_name);
                case 1
                    % use the default tool-link ...
                    ee_lnk_name = obj.mwbm_config.tool_links(1,1).urdf_link_name;
                    ee_vqT_tt   = obj.mwbm_config.tool_links(1,1).ee_vqT_tt;

                    wf_H_ee = mexWholeBodyModel('transformation-matrix', ee_lnk_name);
                    wf_J_ee = mexWholeBodyModel('jacobian', ee_lnk_name);
                otherwise
                    error('WBM::jacobianTool: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end

            %% Velocity transformation matrix:
            %   The transformation matrix X maps the velocities of the geometric Jacobian
            %   in frame {ee} to velocities in frame {tt} and is defined as
            %
            %                        | I    -S(wf_R_ee * ee_p_tt)*I |
            %      tt[wf]_X_ee[wf] = |                              |,
            %                        | 0                 I          |
            %
            %   s.t. wf_J_tt = tt[wf]_X_ee[wf] * ee[wf]_J_ee. The notations tt[wf] and ee[wf]
            %   are denoting frames with origin o_tt and o_ee with orientation [wf].
            %
            % For further details see:
            %   [1] Multibody Dynamics Notation, S. Traversaro & A. Saccon, Eindhoven University of Technology,
            %       Department of Mechanical Engineering, 2016, <http://repository.tue.nl/849895>, p. 6, eq. (27).
            %   [2] Robotics: Modelling, Planning and Control, B. Siciliano & L. Sciavicco & L. Villani & G. Oriolo,
            %       Springer, 2010, p. 150, eq. (3.112).
            %   [3] Introduction to Robotics: Mechanics and Control, John J. Craig, 3rd Edition, Pearson/Prentice Hall, 2005,
            %       p. 158, eq. (5.103).
            [~, wf_R_ee]    = WBM.utilities.tfms.tform2posRotm(wf_H_ee);   % orientation of the end-effector (ee).
            [p_tt, ee_R_tt] = WBM.utilities.tfms.frame2posRotm(ee_vqT_tt); % position & orientation of the tool-tip (tt).

            % calculate the position from the tool-tip (tt) to the world-frame (wf) ...
            wf_p_tt = wf_R_ee * (ee_R_tt * p_tt); % = wf_R_ee * ee_p_tt
            % get the velocity transformation matrix ...
            tt_X_wf = WBM.utilities.tfms.adjoint(wf_p_tt);

            % compute wf_J_tt by performing velocity addition ...
            wf_J_tt = tt_X_wf * wf_J_ee; % = tt[wf]_X_ee[wf] * ee[wf]_J_ee
        end

        function [chn_q, chn_dq] = getStateJntChains(obj, chain_names, q_j, dq_j)
            switch nargin
                case {2, 4}
                    if isempty(chain_names)
                        error('WBM::getJntChainsState: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
                    end
                    % check if the body components are defined ...
                    if isempty(obj.mwbm_config.body)
                        error('WBM::getJntChainsState: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
                    end

                    if (nargin == 2)
                        % get the current state values ...
                        [~,q_j,~,dq_j] = mexWholeBodyModel('get-state');
                    end

                    len = length(chain_names);
                    if (len > obj.mwbm_config.body.nChns)
                        error('WBM::getJntChainsState: %s', WBM.wbmErrorMsg.WRONG_ARR_SIZE);
                    end

                    % get the joint angles and velocities of each chain ...
                    ridx = find(ismember(obj.mwbm_config.body.chains(:,1), chain_names));
                    if ( isempty(ridx) || (length(ridx) ~= len) )
                        error('WBM::getJntChainsState: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
                    end
                    chn_q  = cell(len,1); % chains ...
                    chn_dq = chn_q;

                    for i = 1:len
                        idx = ridx(i); % for each idx of row-idx ...
                        start_idx = obj.mwbm_config.body.chains{idx,2};
                        end_idx   = obj.mwbm_config.body.chains{idx,3};

                        chn_q{i,1}  = q_j(start_idx:end_idx,1);  % joint angles
                        chn_dq{i,1} = dq_j(start_idx:end_idx,1); % joint velocities
                    end
                otherwise
                    error('WBM::getJntChainsState: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function [jnt_q, jnt_dq] = getStateJointNames(obj, joint_names, q_j, dq_j)
            switch nargin
                case {2, 4}
                    if isempty(joint_names)
                        error('WBM::getStateJointNames: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
                    end
                    % check if the body parts are defined ...
                    if isempty(obj.mwbm_config.body)
                        error('WBM::getStateJointNames: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
                    end

                    if (nargin == 2)
                        % get the state values ...
                        [~,q_j,~,dq_j] = mexWholeBodyModel('get-state');
                    end
                    len = length(joint_names);

                    % get the row indices ...
                    ridx = find(ismember(obj.mwbm_config.body.joints(:,1), joint_names));
                    if ( isempty(ridx) || (length(ridx) ~= len) )
                        error('WBM::getStateJointNames: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
                    end
                    % get the angles and velocities ...
                    [jnt_q, jnt_dq] = getJointValues(obj, q_j, dq_j, ridx, len);
                otherwise
                    error('WBM::getStateJointNames: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function [jnt_q, jnt_dq] = getStateJointIdx(obj, jnt_idx, q_j, dq_j)
            switch nargin
                case {2, 4}
                    % check the index list ...
                    if isempty(jnt_idx)
                        error('WBM::getStateJointIdx: %s', WBM.wbmErrorMsg.EMPTY_VECTOR);
                    end
                    if ( ~isvector(jnt_idx) && ~isinteger(jnt_idx) )
                        error('WBM::getStateJointIdx: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                    end

                    if (nargin == 2)
                        % get the values ...
                        [~,q_j,~,dq_j] = mexWholeBodyModel('get-state');
                    end
                    len = length(jnt_idx);

                    % get the angle and velocity of each joint ...
                    [jnt_q, jnt_dq] = getJointValues(obj, q_j, dq_j, jnt_idx, len);
                otherwise
                    error('WBM::getStateJointIdx: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function stParams = getStateParams(obj, stChi)
            len      = obj.mwbm_config.stvLen;
            ndof     = obj.mwbm_model.ndof;
            stParams = WBM.wbmStateParams;

            if iscolumn(stChi)
                WBM.utilities.chkfun.checkCVecDim(stChi, len, 'WBM::getStateParams');
                % get the base/joint positions and the base orientation ...
                stParams.x_b  = stChi(1:3,1);
                stParams.qt_b = stChi(4:7,1);
                stParams.q_j  = stChi(8:ndof+7,1);
                % the corresponding velocities ...
                stParams.dx_b    = stChi(ndof+8:ndof+10,1);
                stParams.omega_b = stChi(ndof+11:ndof+13,1);
                stParams.dq_j    = stChi(ndof+14:len,1);
                return
            elseif ismatrix(stChi)
                [m, n] = size(stChi);
                if (n ~= len)
                    error('WBM::getStateParams: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end
                % extract all values ...
                stParams.x_b  = stChi(1:m,1:3);
                stParams.qt_b = stChi(1:m,4:7);
                stParams.q_j  = stChi(1:m,8:ndof+7);

                stParams.dx_b    = stChi(1:m,ndof+8:ndof+10);
                stParams.omega_b = stChi(1:m,ndof+11:ndof+13);
                stParams.dq_j    = stChi(1:m,ndof+14:len);
                return
            end
            % else ...
            error('WBM::getStateParams: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end

        function [vqT_b, q_j] = getPositions(obj, stChi)
            len  = obj.mwbm_config.stvLen;
            cutp = obj.mwbm_model.ndof + 7; % 3 + 4 + ndof

            if iscolumn(stChi)
                WBM.utilities.chkfun.checkCVecDim(stChi, len, 'WBM::getPositions');
                % extract the base VQS-Transformation (without S)
                % and the joint positions ...
                vqT_b = stChi(1:7,1); % [x_b; qt_b]
                q_j   = stChi(8:cutp,1);
                return
            elseif ismatrix(stChi)
                [m, n] = size(stChi);
                if (n ~= len)
                    error('WBM::getPositions: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end
                vqT_b = stChi(1:m,1:7);    % m -by- [x_b, qt_b]
                q_j   = stChi(1:m,8:cutp); % m -by- q_j
                return
            end
            % else ...
            error('WBM::getPositions: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end

        function stmPos = getPositionsData(obj, stmChi)
            [m, n] = size(stmChi);
            if (n ~= obj.mwbm_config.stvLen)
                error('WBM::getPositionsData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            cutp   = obj.mwbm_model.ndof + 7; % 3 + 4 + ndof
            stmPos = stmChi(1:m,1:cutp);      % m -by- [x_b, qt_b, q_j]
        end

        function [v_b, dq_j] = getMixedVelocities(obj, stChi)
            len   = obj.mwbm_config.stvLen;
            ndof  = obj.mwbm_model.ndof;

            if iscolumn(stChi)
                WBM.utilities.chkfun.checkCVecDim(stChi, len, 'WBM::getMixedVelocities');
                % extract the velocities ...
                v_b  = stChi(ndof+8:ndof+13,1); % [dx_b; omega_b]
                dq_j = stChi(ndof+14:len,1);
                return
            elseif ismatrix(stChi)
                [m, n] = size(stChi);
                if (n ~= len)
                    error('WBM::getMixedVelocities: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end
                v_b  = stChi(1:m,ndof+8:ndof+13); % m -by- [dx_b; omega_b]
                dq_j = stChi(1:m,ndof+14:len,1);  % m -by- dq_j
                return
            end
            % else ...
            error('WBM::getMixedVelocities: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end

        function v_b = getBaseVelocities(obj, stChi)
            len   = obj.mwbm_config.stvLen;
            ndof  = obj.mwbm_model.ndof;

            if iscolumn(stChi)
                WBM.utilities.chkfun.checkCVecDim(stChi, len, 'WBM::getBaseVelocities');

                v_b = stChi(ndof+8:ndof+13,1); % [dx_b; omega_b]
                return
            elseif ismatrix(stChi)
                [m, n] = size(stChi);
                if (n ~= len)
                    error('WBM::getBaseVelocities: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                end
                v_b = stChi(1:m,ndof+8:ndof+13); % m -by- [dx_b; omega_b]
                return
            end
            % else ...
            error('WBM::getBaseVelocities: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end

        function len = get.stvLen(obj)
            len = obj.mwbm_config.stvLen;
        end

        function vqT_b = get.vqT_base(~)
            [vqT_b,~,~,~] = mexWholeBodyModel('get-state');
        end

        function vqT_b = get.init_vqT_base(obj)
            stp_init = obj.mwbm_config.init_state_params;
            vqT_b    = vertcat(stp_init.x_b, stp_init.qt_b);
        end

        function stvChi = get.init_stvChi(obj)
            stp_init = obj.mwbm_config.init_state_params;
            stvChi   = vertcat(stp_init.x_b, stp_init.qt_b, stp_init.q_j, ...
                               stp_init.dx_b, stp_init.omega_b, stp_init.dq_j);
        end

        function set.init_state(obj, stp_init)
            if ~checkInitStateDimensions(obj, stp_init)
                error('WBM::set.init_state: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            obj.mwbm_config.init_state_params = stp_init;
        end

        function stp_init = get.init_state(obj)
            stp_init = obj.mwbm_config.init_state_params;
        end

        function robot_body = get.robot_body(obj)
            robot_body = obj.mwbm_config.body;
        end

        function robot_config = get.robot_config(obj)
            robot_config = obj.mwbm_config;
        end

        function robot_params = get.robot_params(obj)
            robot_params = WBM.wbmRobotParams;
            robot_params.model     = obj.mwbm_model;
            robot_params.config    = obj.mwbm_config;
            robot_params.wf2fixlnk = obj.mwf2fixlnk;
        end

        function dispConfig(obj, prec)
            if ~exist('prec', 'var')
                prec = 2;
            end
            nPlds    = obj.mwbm_config.nPlds;
            nCstrs   = obj.mwbm_config.nCstrs;
            stp_init = obj.mwbm_config.init_state_params;

            clnk_names    = vertcat(num2cell(1:nCstrs), obj.mwbm_config.ccstr_link_names);
            strLnkNameLst = sprintf('  %d  %s\n', clnk_names{1:2,1:nCstrs});

            cinit_stp = cell(6,1);
            cinit_stp{1,1} = sprintf('  q_j:      %s', mat2str(stp_init.q_j, prec));
            cinit_stp{2,1} = sprintf('  dq_j:     %s', mat2str(stp_init.dq_j, prec));
            cinit_stp{3,1} = sprintf('  x_b:      %s', mat2str(stp_init.x_b, prec));
            cinit_stp{4,1} = sprintf('  qt_b:     %s', mat2str(stp_init.qt_b, prec));
            cinit_stp{5,1} = sprintf('  dx_b:     %s', mat2str(stp_init.dx_b, prec));
            cinit_stp{6,1} = sprintf('  omega_b:  %s', mat2str(stp_init.omega_b, prec));
            strInitState   = sprintf('%s\n%s\n%s\n%s\n%s\n%s', cinit_stp{1,1}, cinit_stp{2,1}, ...
                                     cinit_stp{3,1}, cinit_stp{4,1}, cinit_stp{5,1}, cinit_stp{6,1});

            strPldTbl = sprintf('  none\n');
            if (nPlds > 0)
                % print the payload data in table form:
                pl_lnks = obj.mwbm_config.payload_links;

                clnk_names = cell(nPlds,1);
                cpos       = clnk_names;
                cmass      = clnk_names;
                cinertia   = clnk_names;
                % put the data in cell-arrays ...
                for i = 1:nPlds
                    clnk_names{i,1} = pl_lnks(1,i).urdf_link_name;
                    cpos{i,1}       = mat2str(pl_lnks(1,i).lnk_p_cm, prec);
                    cmass{i,1}      = num2str(pl_lnks(1,i).m_rb, prec);
                    cinertia{i,1}   = mat2str(pl_lnks(1,i).I_cm, prec);
                end
                % get the string lengths and the max. string lengths ...
                slen1 = cellfun('length', clnk_names);
                slen2 = cellfun('length', cpos);
                slen3 = cellfun('length', cmass);
                msl1  = max(slen1);
                msl2  = max(slen2);
                msl3  = max(slen3);
                % compute the number of spaces ...
                nspc = msl1 - 9 + 6; % length('link_name') = 9

                % create the formatted table in string form ...
                strPldTbl = sprintf('  idx   link_name%spos%smass%sinertia\\n', blanks(nspc), blanks(msl2), blanks(msl3-1));
                for i = 1:nPlds
                    nspc_1 = msl1 - slen1(i,1) + 6;
                    nspc_2 = msl2 - slen2(i,1) + 3;
                    nspc_3 = msl3 - slen3(i,1) + 3;
                    str = sprintf('   %d    %s%s%s%s%s%s%s\\n', i, clnk_names{i,1}, blanks(nspc_1), cpos{i,1}, blanks(nspc_2), ...
                                                                   cmass{i,1}, blanks(nspc_3), cinertia{i,1});
                    strPldTbl = strcat(strPldTbl, str);
                end
                strPldTbl = sprintf(strPldTbl);
            end

            strConfig = sprintf(['Robot Configuration:\n\n' ...
                                 ' #constraints: %d\n' ...
                                 ' constraint link names:\n%s\n' ...
                                 ' initial state:\n%s\n\n' ...
                                 ' #payloads: %d\n' ...
                                 ' link payloads:\n%s'], ...
                                obj.mwbm_config.nCstrs, strLnkNameLst, ...
                                strInitState, nPlds, strPldTbl);
            fprintf('%s\n', strConfig);
        end

    end

    methods(Access = private)
        function initConfig(obj, robot_config)
            % check if robot_config is an instance of a class that
            % is derived from "wbmRobotConfig" ...
            if ~isa(robot_config, 'WBM.wbmRobotConfig')
                error('WBM::initConfig: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % further error checks ...
            nCstrs = robot_config.nCstrs; % by default 0, when value is not given ...
            if (nCstrs > 0)
                if (nCstrs ~= size(robot_config.ccstr_link_names,2))
                    % the list is not a row vector or the sizes are different ...
                    error('WBM::initConfig: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
                end
            else
                % the length is not given, try to get it ...
                nCstrs = size(robot_config.ccstr_link_names,2);
            end

            if isempty(robot_config.init_state_params)
                error('WBM::initConfig: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end

            obj.mwbm_config = WBM.wbmRobotConfig;
            obj.mwbm_config.nCstrs           = nCstrs;
            obj.mwbm_config.ccstr_link_names = robot_config.ccstr_link_names;

            if ~isempty(robot_config.body)
                obj.mwbm_config.body = robot_config.body;
            end

            if ~WBM.utilities.isStateEmpty(robot_config.init_state_params)
                if (obj.mwbm_model.ndof > 0)
                    obj.mwbm_config.stvLen = 2*obj.mwbm_model.ndof + 13;
                else
                    % the DoF is unknown or is not set --> use the vector length ...
                    vlen = size(robot_config.init_state_params.q_j,1);
                    obj.mwbm_config.stvLen = 2*vlen + 13;
                end

                % check all parameter dimensions in "init_state_params", summed size
                % is either: 0 (= empty), 'stvLen' or 'stvLen-7' ...
                if ~checkInitStateDimensions(obj, robot_config.init_state_params)
                    error('WBM::initConfig: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
                end
                % check the number of joints ...
                if (size(robot_config.init_state_params.q_j,1) > obj.MAX_NUM_JOINTS)
                    error('WBM::initConfig: %s', WBM.wbmErrorMsg.MAX_JOINT_LIMIT);
                end
            end
            obj.mwbm_config.init_state_params = robot_config.init_state_params;
        end

        function [jnt_q, jnt_dq] = getJointValues(obj, q_j, dq_j, jnt_idx, len)
            if (len > obj.mwbm_config.body.nJnts)
                error('WBM::getJointValues: %s', WBM.wbmErrorMsg.WRONG_VEC_LEN);
            end
            % get the joint values of the index list ...
            jnt_q(1:len,1)  = q_j(jnt_idx,1);  % angle
            jnt_dq(1:len,1) = dq_j(jnt_idx,1); % velocity
        end

        function result = checkInitStateDimensions(obj, stp_init)
            len = size(stp_init.x_b,1) + size(stp_init.qt_b,1) + size(stp_init.q_j,1) + ...
                  size(stp_init.dx_b,1) + size(stp_init.omega_b,1) + size(stp_init.dq_j,1);

            if (len ~= obj.mwbm_config.stvLen) % allowed length: 'stvLen' or 'stvLen-7'
                if (len ~= (obj.mwbm_config.stvLen - 7)) % length without x_b & qt_b (they will be updated afterwards)
                    result = false;
                    return
                end
            end
            result = true;
        end

        function [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCC(obj, varargin) % in dependency of (specific) contact constraints (CC)
            switch nargin
                case 7 % normal modes:
                    % use only specific contacts:
                    % wf_R_b_arr = varargin{1}
                    % wf_p_b     = varargin{2}
                    % q_j        = varargin{3}
                    % dq_j       = varargin{4}
                    % v_b        = varargin{5}
                    % idx_list   = varargin{6}
                    [M, c_qv]   = wholeBodyDyn(obj, varargin{1:5});
                    [Jc, djcdq] = contactJacobians(obj, varargin{1:6});
                case 6
                    % use all contacts:
                    [M, c_qv]   = wholeBodyDyn(obj, varargin{1:5});
                    [Jc, djcdq] = contactJacobians(obj, varargin{1:5});
                case 2 % optimized modes:
                    % specific contacts:
                    % idx_list = varargin{1}
                    [M, c_qv]   = wholeBodyDyn(obj);
                    [Jc, djcdq] = contactJacobians(obj, varargin{1,1});
                case 1
                    % all contacts:
                    [M, c_qv]   = wholeBodyDyn(obj);
                    [Jc, djcdq] = contactJacobians(obj);
                otherwise
                    error('WBM::wholeBodyDynamicsCC: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function [M, c_qv] = wholeBodyDyn(~, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b)
            switch nargin
                case 6 % normal mode:
                    M    = mexWholeBodyModel('mass-matrix', wf_R_b_arr, wf_p_b, q_j);
                    c_qv = mexWholeBodyModel('generalized-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                case 1 % optimized mode:
                    M    = mexWholeBodyModel('mass-matrix');
                    c_qv = mexWholeBodyModel('generalized-forces');
                otherwise
                    error('WBM::wholeBodyDyn: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function nu = fdynNewMixedVelocities(~, qt_b, dx_b, wf_omega_b, dq_j)
            % get the rotation matrix of the current VQ-transformation (base-to-world):
            [vqT_b,~,~,~] = mexWholeBodyModel('get-state');
            [~,wf_R_b] = WBM.utilities.tfms.frame2posRotm(vqT_b);

            % We need to apply the world-to-base rotation b_R_wf to the spatial angular
            % velocity wf_omega_b to obtain the angular velocity b_omega_wf in the base
            % body frame. This is then used in the quaternion derivative computation:
            b_R_wf = wf_R_b.';
            b_omega_wf = b_R_wf * wf_omega_b;
            dqt_b      = WBM.utilities.tfms.dquat(qt_b, b_omega_wf);

            % new mixed generalized velocity vector ...
            nu = vertcat(dx_b, dqt_b, dq_j);
        end

        function clink_idx = getContactIdx(~, clnk_conf)
            ctc_l = clnk_conf.contact.left;  % CS-left
            ctc_r = clnk_conf.contact.right; % CS-right

            % check the contact state (CS) of the contact links:
            if (ctc_l && ctc_r)
                % both links have contact to the ground/object ...
                clink_idx = horzcat(clnk_conf.lnk_idx_l, clnk_conf.lnk_idx_r);
            elseif ctc_l
                % only the left link has contact ...
                clink_idx = clnk_conf.lnk_idx_l;
            elseif ctc_r
                % only the right link has contact ...
                clink_idx = clnk_conf.lnk_idx_r;
            else
                % no contacts ...
                clink_idx = 0;
            end
        end

        function [fc_f, tau_gen] = footContactForces(obj, foot_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j)
            fe_0 = zeroExtForces(obj, foot_conf);
            [fc_f, tau_gen] = contactForcesEF(obj, tau, fe_0, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j); % with friction
        end

        function [fc_f, tau_gen] = footContactForcesPC(obj, foot_conf, tau, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j, nu) % PC ... Pose Corrections
            fe_0 = zeroExtForces(obj, foot_conf);
            [fc_f, tau_gen] = contactForcesCLPCEF(obj, foot_conf, tau, fe_0, ac_f, Jc_f, djcdq_f, M, c_qv, dq_j, nu); % with friction, optimized mode
        end

        function [M, c_qv, Jc_f, djcdq_f, Jc, djcdq] = fullWholeBodyDynCS(obj, foot_conf, clnk_conf, wf_R_b, wf_p_b, q_j, dq_j, v_b) % in dependency of the contact state (CS)
            % compute whole body dynamics and the contact Jacobians of all contact constraints (incl. feet):
            switch nargin
                case 8
                    % normal mode:
                    wf_R_b_arr = reshape(wf_R_b, 9, 1);
                    [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, foot_conf, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                    [Jc, djcdq] = contactJacobiansCS(obj, clnk_conf, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                case 3
                    % optimized mode:
                    [M, c_qv, Jc_f, djcdq_f] = wholeBodyDynamicsCS(obj, foot_conf);
                    [Jc, djcdq] = contactJacobiansCS(obj, clnk_conf);
                otherwise
                    error('WBM::fullWholeBodyDynCS: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function [M, c_qv, Jc_f] = getWBDynFeet(~, a_prms)
            M    = a_prms.M;
            c_qv = a_prms.c_qv;
            Jc_f = a_prms.Jc_f;
        end

        function clnk_conf = createConfigStateCL(obj, cstate, clink_l, clink_r, varargin)
            clnk_idx_l = find(strcmp(obj.mwbm_config.ccstr_link_names, clink_l));
            clnk_idx_r = find(strcmp(obj.mwbm_config.ccstr_link_names, clink_r));

            if ( isempty(clnk_idx_l) || isempty(clnk_idx_r) )
                error('WBM::createConfigStateCL: %s', WBM.wbmErrorMsg.LNK_NOT_IN_LIST);
            end
            clink_idx = horzcat(clnk_idx_l, clnk_idx_r);

            n = size(varargin,2);
            if ~ischar(varargin{1,n}) % n = 3, 2 or 1
                varargin{1,n+1} = 'eul'; % default rotation type
            end

            clnk_conf = clnkConfigState(obj, cstate, clink_idx, varargin{:});
        end

        [] = visualizeSimRobot(obj, stmPos, sim_config, sim_tstep, vis_ctrl)

        function setPayloadLinkData(obj, pl_idx, pl_lnk)
            obj.mwbm_config.payload_links(1,pl_idx).urdf_link_name = pl_lnk.name;
            obj.mwbm_config.payload_links(1,pl_idx).lnk_p_cm       = pl_lnk.lnk_p_cm;

            if isfield(pl_lnk, 't_idx') % tool index:
                obj.mwbm_config.payload_links(1,pl_idx).t_idx = pl_lnk.t_idx;
            else
                % the payload is not a tool ...
                obj.mwbm_config.payload_links(1,pl_idx).t_idx = 0;
            end

            if isfield(pl_lnk, 'vb_idx') % volume body index:
                obj.mwbm_config.payload_links(1,pl_idx).vb_idx = pl_lnk.vb_idx;
            else
                % the payload is not linked with a volume body object ...
                obj.mwbm_config.payload_links(1,pl_idx).vb_idx = 0;
            end

            if isfield(pl_lnk, 'm_rb') % mass of rigid body:
                obj.mwbm_config.payload_links(1,pl_idx).m_rb = pl_lnk.m_rb;
                obj.mwbm_config.payload_links(1,pl_idx).I_cm = pl_lnk.I_cm;
            else
                % mass and inertia are undefined ...
                obj.mwbm_config.payload_links(1,pl_idx).m_rb = 0;
                obj.mwbm_config.payload_links(1,pl_idx).I_cm = [];
            end
        end

        function lnk_traj = setTrajectoryDPts(obj, lnk_traj, vqT_b, q_j, nSteps)
            % calculate the positions of the given link for n time steps and
            % add them to the position array:
            lnk_name         = lnk_traj.urdf_link_name;
            lnk_traj.lnk_pos = zeros(nSteps,3);
            for i = 1:nSteps
                q   = q_j(:,i);
                vqT = squeeze(vqT_b(i,1:7)).';
                [wf_p_b, wf_R_b] = WBM.utilities.tfms.frame2posRotm(vqT);

                wf_H_lnk = transformationMatrix(obj, wf_R_b, wf_p_b, q, lnk_name);
                lnk_traj.lnk_pos(i,1:3) = wf_H_lnk(1:3,4).';
            end
        end

    end
end
