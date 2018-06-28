% Copyright (C) 2015-2018, by Martin Neururer
% Author: Martin Neururer
% E-mail: martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
% Date:   January-May, 2018
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
% FP7 EU-project CoDyCo (No. 600716, ICT-2011.2.1 Cognitive Systems and
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
    % The class :class:`!WBM` is the *main class* of the *whole-body model* (WBM)
    % for YARP-based floating-base robots. It is a derived class from the class
    % :class:`~WBM.WBMBase` and offers additionally *higher-level methods* for the
    % robot as extension, such as *visualization, forward dynamics, contact forces,
    % payloads, tools*, etc.
    %
    % Attributes:
    %   stvLen        (uint16, scalar): Length of the state parameter vector, i.e. the
    %                                   length of all state parameters concatenated in
    %                                   in one vector. The length can be either
    %                                   :math:`2 n_{dof} + 13` (default),
    %                                   :math:`2 n_{dof} + 6` (without
    %                                   :attr:`~WBM.wbmStateParams.x_b` and
    %                                   :attr:`~WBM.wbmStateParams.qt_b`) or *zero*,
    %                                   if :attr:`init_state` is empty [#f7]_
    %                                   (*read only*).
    %   vqT_base      (double, vector): Base VQ-transformation frame (of the
    %                                   floating base) at the current state of
    %                                   the robot system (*read only*).
    %   init_vqT_base (double, vector): Initial VQ-transformation frame of the
    %                                   robot's floating base (*read only*).
    %   init_stvChi   (double, vector): Initial state vector :math:`\chi` of the
    %                                   robot for the initial integration of the
    %                                   forward dynamics in state-space form [#f10]_
    %                                   (*read only*).
    %
    %                                   **Note:** The state variable :math:`\chi` is
    %                                   a vector to express the forward dynamics in
    %                                   *state-space form* (:cite:`Featherstone2008`,
    %                                   p. 42, eq. (3.8)) and is defined as,
    %                                   :math:`\chi = [x_b, qt_b, q_j, \dot{x}_b, \omega_b, \dot{q}_j]^T`
    %                                   with:
    %
    %                                      - :math:`x_b` -- Cartesian position of the floating
    %                                        base *b* in Euclidean space :math:`\mathbb{R}^3`.
    %                                      - :math:`qt_b` -- Orientation of the base *b* in
    %                                        *quaternions* (global parametrization of
    %                                        :math:`\mathbf{SO}^3`).
    %                                      - :math:`q_j` -- Joint positions (angles) of dimension
    %                                        :math:`\mathbb{R}^{n_{dof}}` in :math:`[\si{\radian}]`.
    %                                      - :math:`\dot{x}_b` -- Cartesian velocity of the base
    %                                        *b* in Euclidean space :math:`\mathbb{R}^3`.
    %                                      - :math:`\omega_b` -- Angular velocity describing the
    %                                        *rotational velocity* of the base *b* in
    %                                        :math:`\mathbf{SO}^3`.
    %
    %   init_state   (:class:`~WBM.wbmStateParams`): Data object to define the initial state
    %                                                parameters of the given floating-base robot.
    %   robot_body   (:class:`~WBM.wbmBody`): Data object that specifies the body components of
    %                                         the given floating-base robot (*read only*).
    %   robot_config (:class:`~WBM.wbmRobotConfig`): Configuration object with the configuration
    %                                                settings of the given floating-base robot
    %                                                (*read only*).
    %   robot_params (:class:`~WBM.wbmRobotParams`): Data object with the current settings of the
    %                                                model and configuration parameters of the
    %                                                given floating-base robot (*read only*).
    %
    %                                                **Note:** This property is useful to exchange
    %                                                the parameter settings of the robot between
    %                                                interfaces.
    %   DF_STIFFNESS  (int, scalar): Default stiffness control gain for the
    %                                position correction: 2.5 (*constant*).
    %   MAX_NUM_TOOLS (int, scalar): Maximum number of tools that a humanoid
    %                                robot can use simultaneously: 2 (*constant*).
    %   MAX_JNT_SPEED (int, scalar): Maximum joint speed for execution in :math:`[\mathrm{ksps}]`
    %                                (kilosample(s) per second) [#f11]_: 250 (*constant*).
    %   MAX_JNT_ACC   (int, scalar): Maximum joint acceleration with :math:`\SI{1}{\mathrm{Ms/{s^2}}}`
    %                                (Megasample(s) per second squared) that any joint
    %                                is allowed to attempt [#f11]_ (*constant*).
    %   MAX_JNT_TRQ   (int, scalar): Maximum joint torque in :math:`[\mathrm{ksps}]`
    %                                that any joint is allowed to attempt: :math:`\num{1e5}`
    %                                (*constant*).
    %   ZERO_CVEC_12  (int, scalar): :math:`(12 \times 1)` column-vector of zeros
    %                                (for velocities, accelerations, forces, etc.)
    %                                -- *constant*.
    %   ZERO_CVEC_6   (int, scalar): :math:`(6 \times 1)` column-vector of zeros
    %                                (for velocities, accelerations, etc.) --
    %                                *constant*.
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

        MAX_JNT_SPEED = 250; % max. joint speed in [ksps] (kilosample(s) per second). [*]
        MAX_JNT_ACC   = 1e6; % max. joint acceleration with 1 [Ms/s^2] (Megasample(s) per second squared). [*]
        MAX_JNT_TRQ   = 1e5; % max. joint torque in [ksps] (kilosample(s) per second).
        % [*] Source: <http://wiki.icub.org/brain/velControlThread_8cpp.html>

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
        function obj = WBM(robot_model, robot_config, wf2fixlnk)
            % Constructor.
            %
            % The constructor initializes the given model and configuration settings
            % of the floating-base robot and sets the world frame (wf) to the initial
            % position and orientation with a specified gravitation.
            %
            % Arguments:
            %   robot_model   (:class:`~WBM.wbmRobotModel`): Model object with the model parameters
            %                                                of the given floating-base robot.
            %   robot_config (:class:`~WBM.wbmRobotConfig`): Configuration object with the configuration
            %                                                settings of the given floating-base robot.
            %   wf2fixlnk (logical, scalar): Boolean flag to indicate if the world frame (wf)
            %                                will be set to a fixed reference link frame
            %                                (*optional*). Default: *false*.
            %
            %                                **Note:** If the fixed reference link is not
            %                                specified in the given robot model, then the
            %                                constructor uses as *default fixed link* the
            %                                *first entry* of the given *contact constraint
            %                                list*.
            % Returns:
            %   obj: An instance of the :class:`!WBM` class.

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
                    % set the world frame (wf) at the computed position and orientation relative
                    % from the given fixed link (base reference frame). In this case, the default
                    % fixed link is used (first entry of the contact constraint list):
                    setWorldFrameAtFixLnk(obj, obj.mwbm_config.ccstr_link_names{1,1});
                else
                    error('WBM::WBM: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
                end
            end
            % retrieve and update the initial VQ-transformation of the robot base (world frame) ...
            updateInitVQTransformation(obj);
        end

        function newObj = copy(obj)
            % Clones the current object.
            %
            % This copy method replaces the ``matlab.mixin.Copyable.copy()``
            % method and creates a *deep copy* of the current object by using
            % directly the memory.
            %
            % Returns:
            %   newObj: An exact copy (clone) of the current :class:`!WBM` object.
            newObj = copy@WBM.WBMBase(obj);
        end

        function delete(obj)
            % Destructor.
            %
            % Removes all class properties from the workspace (free-up memory).
            delete@WBM.WBMBase(obj);
        end

        function setWorldFrameAtFixLnk(obj, urdf_fixed_link, q_j, dq_j, v_b, g_wf)
            % Sets the world frame (wf) at the computed *position* and *orientation*
            % relative from a specified *fixed reference link* (base reference
            % frame).
            %
            % The *base reference frame*, also called *floating-base frame*, is
            % attached to the *fixed reference link* of the robot. Since the most
            % humanoid robots and other legged robots are not physically attached
            % to the world, the *floating-base framework* provides a more general
            % representation for the robot control. The position and orientation
            % of the world frame, relative from the specified fixed link, is
            % obtained from the *forward kinematics* w.r.t. the frame of the
            % given fixed link. The specified fixed link (*floating-base link*
            % or *base reference link*) can also be a *contact constraint link*.
            %
            % The method can be called in two ways:
            %
            %   - .. py:method:: setWorldFrameAtFixLnk(urdf_fixed_link, q_j, dq_j, v_b[, g_wf])
            %   - .. py:method:: setWorldFrameAtFixLnk(urdf_fixed_link)
            %
            % Arguments:
            %   urdf_fixed_link (char, vector): String matching *URDF name* of the
            %                                   fixed link as reference link (frame).
            %   q_j           (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                                   vector in :math:`[\si{\radian}]` (*optional*).
            %   dq_j          (double, vector): :math:`(n_{dof} \times 1)` joint velocities
            %                                   vector in :math:`[\si{\radian/s}]` (*optional*).
            %   v_b           (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                                   vector (*optional*).
            %   g_wf          (double, vector): :math:`(3 \times 1)` Cartesian gravity vector
            %                                   in the world frame *wf* (*optional*).
            % Note:
            %   If only the fixed link is specified, then by default the method uses
            %   for the position and orientation calculation (from the floating base
            %   to world frame) the predefined *initial state parameters* of the robot
            %   configuration.
            %
            % See Also:
            %   :meth:`WBMBase.getWorldFrameFromFixLnk` and :meth:`WBMBase.setWorldFrame`.
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
            % Updates the *position* and *orientation* of the world frame (wf)
            % relative from the defined *fixed reference link* (base reference
            % frame).
            %
            % The position and orientation of the world frame (relative from the
            % specified fixed link) is obtained from the *forward kinematics*
            % w.r.t. the frame of the predefined fixed link.
            %
            % The method can be called in two ways:
            %
            %   - .. py:method:: updateWorldFrameFromFixLnk(q_j, dq_j, v_b[, g_wf])
            %   - .. py:method:: updateWorldFrameFromFixLnk()
            %
            % Arguments:
            %   q_j  (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                          vector in :math:`[\si{\radian}]` (*optional*).
            %   dq_j (double, vector): :math:`(n_{dof} \times 1)` joint velocities
            %                          vector in :math:`[\si{\radian/s}]` (*optional*).
            %   v_b  (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                          vector (*optional*).
            %   g_wf (double, vector): :math:`(3 \times 1)` Cartesian gravity vector
            %                          in the world frame *wf* (*optional*).
            % Note:
            %   If no arguments are given, then by default the method uses for the
            %   position and orientation calculation (from the floating base to the
            %   world frame) the predefined *initial state parameters* of the robot
            %   configuration.
            %
            % See Also:
            %   :meth:`WBM.setWorldFrameAtFixLnk`, :meth:`WBMBase.getWorldFrameFromDfltFixLnk`
            %   and :meth:`WBMBase.setWorldFrame`.
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
            % Updates the initial state parameters of the *position* and *orientation*
            % of the *floating base* with the base VQ-transformation values of the
            % current state of the robot system.

            vqT_init = obj.vqT_base; % get the VQ-transf. of the current state ...
            obj.mwbm_config.init_state_params.x_b  = vqT_init(1:3,1); % translation/position
            obj.mwbm_config.init_state_params.qt_b = vqT_init(4:7,1); % orientation (quaternion)
        end

        function vqT_lnk = fkinVQTransformation(obj, urdf_link_name, q_j, vqT_b, g_wf)
            % Computes the forward kinematic *VQ-transformation frame* of a specific
            % link of the floating-base robot.
            %
            % The method can be called as follows:
            %
            %   .. py:method:: fkinVQTransformation(urdf_link_name, q_j, vqT_b[, g_wf])
            %
            % Arguments:
            %   urdf_link_name (char, vector): String matching *URDF name* of the link.
            %   q_j          (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                                  vector in :math:`[\si{\radian}]`.
            %   vqT_b        (double, vector): :math:`(7 \times 1)` VQ-transformation
            %                                  frame relative from the base *b* to
            %                                  the world frame *wf* [#f9]_.
            %   g_wf         (double, vector): :math:`(3 \times 1)` Cartesian gravity vector
            %                                  in the world frame *wf* (*optional*).
            % Returns:
            %   vqT_lnk (double, vector): :math:`(7 \times 1)` VQ-transformation frame
            %   relative from the given link frame *lnk* to the world frame *wf*.
            %
            % See Also:
            %   :meth:`WBMBase.forwardKinematics`.

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
            % Computes the *contact forces* :math:`f_c` of the given *contact
            % constraints* of the floating-base robot, that are generated by
            % the environment.
            %
            % The formula for calculating the *contact constraint forces* is derived
            % from the dynamic equations of motion of the robot, such that:
            %
            %   .. math::
            %      :label: contact_forces
            %
            %      f_c = \Lambda_c\cdot (J_c M^{\text{-}1}\cdot (C(q_j, \dot{q}_j) -
            %      \tau_{gen}) - \dot{J}_c\dot{q}_j)\:,
            %
            % where :math:`\Lambda_c = \Upsilon_{c}^{\text{-}1} = (J_c M^{\text{-}1} J_{c}^{T})^{\text{-}1}`
            % denotes the *inertia matrix* (or *pseudo-kinetic energy matrix*
            % :cite:`Khatib1987`) in contact space :math:`\mathrm{C = \{C_1,\ldots,C_k\}}`,
            % :math:`\tau_{gen} = S_j\cdot (\tau - \tau_{fr})` represents the
            % *generalized forces* with the *joint selection matrix*
            % :math:`S_j = [\mathbf{0}_{(6 \times n)}\ \mathbb{I}_n]^T`
            % and the *friction forces* :math:`\tau_{fr}`.
            %
            % The method can be called as follows:
            %
            %   .. py:method:: contactForces(tau, Jc, djcdq, M, c_qv[, dq_j])
            %
            % Arguments:
            %   tau   (double, vector): :math:`(n \times 1)` torque force vector
            %                           of the joints and the base of the robot.
            %   Jc    (double, matrix): :math:`(6 m \times n)` Jacobian of the
            %                           *contact constraints* in contact space
            %                           :math:`\mathrm{C = \{C_1,\ldots,C_k\}}`.
            %   djcdq (double, vector): :math:`(6 m \times 1)` product vector of
            %                           the time derivative of the contact Jacobian
            %                           :math:`\dot{J}_c` and the joint velocity
            %                           :math:`\dot{q}_j`.
            %   M     (double, matrix): :math:`(n \times n)` generalized mass matrix
            %                           of the robot.
            %   c_qv  (double, vector): :math:`(n \times 1)` generalized bias force
            %                           vector of the robot.
            %   dq_j  (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                           vector in :math:`[\si{\radian/s}]` (*optional*).
            %
            %                           **Note:** If the joint velocity is not given,
            %                           then the method assumes that the robot system
            %                           is *frictionless*.
            %
            % The variable :math:`m` denotes the *number of contact constraints* of
            % the robot and :math:`n = n_{dof} + 6`.
            %
            % Returns:
            %   [f_c, tau_gen]: 2-element tuple containing:
            %
            %      - **f_c**     (*double, vector*) -- :math:`(n \times 1)` contact
            %        constraint force vector,
            %      - **tau_gen** (*double, vector*) -- :math:`(n \times 1)` generalized
            %        force vector of the robot,
            %
            %   with :math:`n = n_{dof} + 6`.
            %
            % See Also:
            %   :meth:`WBMBase.frictionForces`.
            %
            % References:
            %   - :cite:`Park2006`, chapter 5, pp. 106-110, eq. (5.5)-(5.14).
            %   - :cite:`Murray1994`, pp. 269-270, eq. (6.5) and (6.6).
            %   - :cite:`Khatib1987`, p. 49-50, eq. (51).

            % References:
            %   [Par06] Park, Jaeheung: Control Strategies for Robots in Contact.
            %           PhD-Thesis, Artificial Intelligence Laboratory, Stanford University,
            %           2006, Chapter 5, pp. 106-110, eq. (5.5)-(5.14),
            %           URL: <http://cs.stanford.edu/group/manips/publications/pdfs/Park_2006_thesis.pdf>.
            %   [MLS94] Murray, R. M.; Li, Z.; Sastry, S. S.: A Mathematical Introduction to Robotic Manipulation.
            %           CRC Press, 1994, pp. 269-270, eq. (6.5) and (6.6).
            %   [Kha87] Khatib, Oussama: A unified approach for motion and force control of robot manipulators: The operational space formulation.
            %           In: IEEE Journal on Robotics and Automation, Volume 3, Issue 1, 1987, p. 49-50, eq. (51),
            %           URL: <https://cs.stanford.edu/group/manips/publications/pdfs/Khatib_1987_RA.pdf>.
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

                                                                                      % in dependency of the Contact State (CS)
        function [M, c_qv, Jc, djcdq] = wholeBodyDynamicsCS(obj, clnk_conf, varargin)
            % Computes the *whole-body dynamics* of a floating-base robot in dependency
            % of the current contact state (CS).
            %
            % The method can be called in two different modes:
            %
            % **Normal mode** -- Computes the whole-body dynamics of the given
            % contact links configuration, specified by the base orientation,
            % the positions and the velocities:
            %
            %   .. py:method:: wholeBodyDynamicsCS(clnk_conf, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b)
            %
            % **Optimized mode** -- Computes the whole-body dynamics of the given
            % contact links configuration at the current state of the robot system:
            %
            %   .. py:method:: wholeBodyDynamicsCS(clnk_conf)
            %
            % Arguments:
            %   clnk_conf  (struct): Configuration structure to specify the *qualitative state*
            %                        of at most two *contact links*.
            %
            %                        The data structure specifies which link is currently in
            %                        contact with the ground or an object. It specifies also
            %                        the *desired poses*, *angular velocities* and *control
            %                        gains* for the position-regulation system of the links.
            %   wf_R_b_arr (double, vector): :math:`(9 \times 1)` rotation matrix reshaped
            %                                in vector form from the base frame *b* to the
            %                                world frame *wf* (*optional*).
            %   wf_p_b     (double, vector): :math:`(3 \times 1)` position vector from the
            %                                base frame *b* to the world frame *wf* (*optional*).
            %   q_j        (double, vector): :math:`(n_{dof} \times 1)` joint positions vector
            %                                in :math:`[\si{\radian}]` (*optional*).
            %   dq_j       (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                                vector in :math:`[\si{\radian/s}]` (*optional*).
            %   v_b        (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                                vector (Cartesian and rotational velocity of
            %                                the base) -- *optional*.
            % Returns:
            %   [M, c_qv, Jc, djcdq]: 4-element tuple containing:
            %
            %      - **M**     (*double, vector*) -- :math:`(n \times n)` generalized
            %        mass matrix of the robot.
            %      - **c_qv**  (*double, vector*) -- :math:`(n \times 1)` generalized
            %        bias force vector of the robot.
            %      - **Jc**    (*double, vector*) -- :math:`(6\cdot m \times n)` Jacobian
            %        of the *contact constraints* in contact space
            %        :math:`\mathrm{C = \{C_1,\ldots,C_k\}}`.
            %      - **djcdq** (*double, vector*) -- :math:`(6\cdot m \times 1)` vector
            %        of the product of the contact Jacobian derivative :math:`\dot{J}_c`
            %        with the joint velocity :math:`\dot{q}_j`.
            %
            % The variable :math:`m` denotes the *number of contact constraints* of
            % the robot and :math:`n = n_{dof} + 6`.
            %
            % Note:
            %   If both contact links that are defined in the given configuration
            %   structure are (currently) not in contact with the ground or an
            %   object, then the arrays of the contact Jacobian :math:`J_c` and
            %   of the product :math:`\dot{J}_c\dot{q}_j` will be set to zero.
            %
            % See Also:
            %   :meth:`WBM.contactJacobiansCS`.

            % wf_R_b_arr = varargin{1}
            % wf_p_b     = varargin{2}
            % q_j        = varargin{3}
            % dq_j       = varargin{4}
            % v_b        = varargin{5}

            % check which link is in contact with the ground/object and calculate
            % the multibody dynamics and the corresponding the contact Jacobians:
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
                                                                            % in dependency of the Contact State (CS)
        function [Jc, djcdq] = contactJacobiansCS(obj, clnk_conf, varargin)
            % Computes the *contact constraint Jacobian* and the *bias acceleration*,
            % i.e. the product of the contact Jacobian derivative with the joint
            % velocities of the floating-base robot, in dependency of the current
            % contact state (CS).
            %
            % The method can be called in two different modes:
            %
            % **Normal mode** -- Computes the contact Jacobian and the bias
            % acceleration of the given contact links configuration, specified
            % by the base orientation, the positions and the velocities:
            %
            %   .. py:method:: contactJacobiansCS(clnk_conf, wf_R_b_arr, wf_p_b, q_j, dq_j, v_b)
            %
            % **Optimized mode** -- Computes the contact Jacobian and the bias
            % acceleration of the given contact links configuration at the
            % current state of the robot system:
            %
            %   .. py:method:: contactJacobiansCS(clnk_conf)
            %
            % Arguments:
            %   clnk_conf  (struct): Configuration structure to specify the *qualitative state*
            %                        of at most two *contact links*.
            %
            %                        The data structure specifies which link is currently in
            %                        contact with the ground or an object. It specifies also
            %                        the *desired poses*, *angular velocities* and *control
            %                        gains* for the position-regulation system of the links.
            %   wf_R_b_arr (double, vector): :math:`(9 \times 1)` rotation matrix reshaped
            %                                in vector form from the base frame *b* to the
            %                                world frame *wf* (*optional*).
            %   wf_p_b     (double, vector): :math:`(3 \times 1)` position vector from the
            %                                base frame *b* to the world frame *wf* (*optional*).
            %   q_j        (double, vector): :math:`(n_{dof} \times 1)` joint positions vector
            %                                in :math:`[\si{\radian}]` (*optional*).
            %   dq_j       (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                                vector in :math:`[\si{\radian/s}]` (*optional*).
            %   v_b        (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                                vector (*optional*).
            % Returns:
            %   [Jc, djcdq]: 2-element tuple containing:
            %
            %      - **Jc**    (*double, vector*) -- :math:`(6\cdot m \times n)` Jacobian
            %        of the *contact constraints* in contact space
            %        :math:`\mathrm{C = \{C_1,\ldots,C_k\}}`.
            %      - **djcdq** (*double, vector*) -- :math:`(6\cdot m \times 1)` bias
            %        acceleration vector :math:`\dot{J}_c\dot{q}_j` in contact space
            %        :math:`\mathrm{C}`.
            %
            %    The variable :math:`m` denotes the *number of contact constraints* of
            %    the robot and :math:`n = n_{dof} + 6`.
            %
            % Note:
            %   If both contact links that are defined in the given configuration
            %   structure are (currently) not in contact with the ground or an
            %   object, then the arrays of the contact Jacobian :math:`J_c` and
            %   of the product :math:`\dot{J}_c\dot{q}_j` will be set to zero.
            %   Furthermore, the bias acceleration :math:`\dot{J}_c\dot{q}_j` is
            %   an acceleration that is not due to a robot acceleration.

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
            % Calculates the *joint angle accelerations* :math:`\ddot{q}_j` of
            % the given floating-base robot.
            %
            % Using the dynamic equations of motion, the calculation of the *joint
            % acceleration vector* :math:`\ddot{q}_j` for the closed chain is
            % solved as follows:
            %
            %   .. math::
            %      :label: joint_accelerations
            %
            %      \ddot{q}_j = M^{\text{-}1}\cdot (\tau_{gen} - C(q_j, \dot{q}_j) -
            %      (J_{c}^{T}\cdot \text{-}f_c))
            %
            % The method can be called in two different modes:
            %
            % **Normal mode** -- Computes the joint accelerations, specified by
            % the base orientation, the positions and the velocities:
            %
            %   - .. py:method:: jointAccelerations(tau, wf_R_b, wf_p_b, q_j, dq_j, v_b)
            %   - .. py:method:: jointAccelerations(tau, wf_R_b, wf_p_b, q_j, nu)
            %
            % **Optimized mode** -- Computes the joint accelerations at the current
            % state of the robot system, in dependency of the given joint velocities:
            %
            %   .. py:method:: jointAccelerations(tau[, dq_j])
            %
            % Arguments:
            %   tau    (double, vector): :math:`(n \times 1)` torque force vector
            %                            for the joints and the base of the robot
            %                            with :math:`n = n_{dof} + 6`.
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf* (*optional*).
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*
            %                            (*optional*).
            %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                            vector in :math:`[\si{\radian}]` (*optional*).
            %   dq_j   (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                            vector in :math:`[\si{\radian/s}]` (*optional*).
            %   v_b    (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                            vector (*optional*).
            %   nu     (double, vector): :math:`((6 + n_{dof}) \times 1)` mixed generalized
            %                            velocity vector (generalized base velocity and
            %                            joint velocity) -- *optional*.
            % Returns:
            %   [ddq_j[, fd_prms]]: 2-element tuple containing:
            %
            %      - **ddq_j** (*double, vector*) -- :math:`(n_{dof} \times 1)` joint angle
            %        acceleration vector in :math:`[\si{\radian/{s^2}}]`.
            %      - **fd_prms**       (*struct*) -- Data structure for the parameter values
            %        of the forward dynamics calculation with the fields ``tau_gen`` and ``f_c``
            %        (*optional*).
            %
            %        **Note:** The second output argument can be used for further calculations
            %        or for data logging.
            %
            % See Also:
            %   :meth:`WBM.jointAccelerationsEF`, :meth:`WBM.jointAccelerationsPL` and
            %   :meth:`WBM.contactForces`.
            %
            % References:
            %   :cite:`Lilly1992`, p. 82, eq. (5.2).

            % References:
            %   [Lil92] Lilly, Kathryn: Efficient Dynamic Simulation of Robotic Mechanisms.
            %           Springer, 1992, p. 82, eq. (5.2).
            switch nargin
                case 7 % normal modes:
                    % generalized forces with friction:
                    % wf_R_b = varargin{1}
                    % wf_p_b = varargin{2}
                    % q_j    = varargin{3}
                    % v_b    = varargin{5}
                    dq_j = varargin{1,4};

                    % compute the whole body dynamics and for every contact constraint
                    % the Jacobian and the product of the Jacobian derivative with the
                    % joint velocities ...
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

            % Joint Acceleration q_ddot (derived from the dyn. equations of motion):
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

                                                                                               % FPC ... Foot Pose Corrections (no external forces)
        function [ddq_j, fd_prms] = jointAccelerationsFPC(obj, foot_conf, tau, ac_f, varargin)
            % Calculates the *joint angle accelerations* :math:`\ddot{q}_j` with
            % *foot pose corrections* (*FPC*), in dependency of the given contact
            % constraints of the floating-base robot.
            %
            % The equation for solving the joint accelerations :math:`\ddot{q}_j`
            % of a closed chain is derived from the dynamic equations of motion
            % (see :eq:`joint_accelerations`). In order to obtain the joint
            % accelerations from the *closed-loop control system* with
            % *position-regulation* (velocity and position correction) of the
            % feet, the method computes at first the contact forces :math:`f_c`
            % of the feet and does not include any external forces that may
            % affecting the system.
            %
            % The method assumes that *only the feet* of the robot may have
            % contact with the *ground* and can be called in three different
            % modes:
            %
            % **Normal mode** -- Computes the joint accelerations, specified by
            % the base orientation, the positions and the velocities:
            %
            %   .. py:method:: jointAccelerationsFPC(foot_conf, tau, ac_f, wf_R_b, wf_p_b, q_j, dq_j, v_b, nu)
            %
            % **Semi-optimized mode:**
            %
            %   .. py:method:: jointAccelerationsFPC(foot_conf, tau, ac_f, wf_R_b, wf_p_b, q_j, nu)
            %
            % **Optimized mode** -- Computes the joint accelerations at the current
            % state of the robot system, in dependency of the given whole-body
            % dynamics and velocities:
            %
            %   - .. py:method:: jointAccelerationsFPC(foot_conf, tau, ac_f, Jc, djcdq, M, c_qv[, dq_j], nu)
            %   - .. py:method:: jointAccelerationsFPC(foot_conf, tau, ac_f, nu)
            %
            % Note:
            %   If the joint velocity vector :math:`\dot{q}_j` (``dq_j``) is not
            %   given as an argument, then the method assumes that the robot system
            %   is *frictionless*.
            %
            % Arguments:
            %   foot_conf         (struct): Configuration structure to specify the *qualitative
            %                               state* of the feet.
            %
            %                               The data structure specifies which foot is currently
            %                               in contact with the ground. It specifies also the
            %                               *desired poses*, *angular velocities* and *control
            %                               gains* for the position-regulation system of the feet.
            %   tau       (double, vector): :math:`(n \times 1)` torque force vector
            %                               for the joints and the base of the robot
            %                               with :math:`n = n_{dof} + 6`.
            %   ac_f      (double, vector): :math:`(k \times 1)` mixed acceleration vector
            %                               for the specified *foot contact points* with
            %                               the size of :math:`k = 6` or :math:`k = 12`.
            %
            %                               **Note:** The given *foot accelerations* are
            %                               either *constant* or *zero*. If :math:`k = 6`,
            %                               then only one foot touches the ground, else
            %                               both feet.
            %   wf_R_b    (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                               (orientation) from the base frame *b*
            %                               to world frame *wf* (*optional*).
            %   wf_p_b    (double, vector): :math:`(3 \times 1)` position vector from
            %                               the base frame *b* to the world frame *wf*
            %                               (*optional*).
            %   q_j       (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                               vector in :math:`[\si{\radian}]` (*optional*).
            %   dq_j      (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                               vector in :math:`[\si{\radian/s}]` (*optional*).
            %   v_b       (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                               vector (*optional*).
            %   nu        (double, vector): :math:`(6 + n_{dof} \times 1)` mixed generalized
            %                               velocity vector (generalized base velocity and
            %                               joint velocity).
            % Other Parameters:
            %   Jc    (double, matrix): :math:`(6 m \times n)` Jacobian of the
            %                           *contact constraints* in contact space
            %                           :math:`\mathrm{C = \{C_1,\ldots,C_k\}}`
            %                           (*optional*).
            %   djcdq (double, vector): :math:`(6 m \times 1)` product vector of
            %                           the time derivative of the contact Jacobian
            %                           :math:`\dot{J}_c` and the joint velocity
            %                           :math:`\dot{q}_j` (*optional*).
            %   M     (double, matrix): :math:`(n \times n)` generalized mass matrix
            %                           of the robot (*optional*).
            %   c_qv  (double, vector): :math:`(n \times 1)` generalized bias force
            %                           vector of the robot (*optional*).
            %
            % The variable :math:`m` denotes the *number of contact constraints* of
            % the robot and :math:`n = n_{dof} + 6`.
            %
            % Returns:
            %   [ddq_j[, fd_prms]]: 2-element tuple containing:
            %
            %      - **ddq_j** (*double, vector*) -- :math:`(n_{dof} \times 1)` joint angle
            %        acceleration vector in :math:`[\si{\radian/{s^2}}]`.
            %      - **fd_prms**       (*struct*) -- Data structure for the parameter values
            %        of the forward dynamics calculation with the fields ``tau_gen``, ``f_c``,
            %        ``a_c`` and ``f_e`` (*optional*).
            %
            %        **Note:** The second output argument can be used for further calculations
            %        or for data logging.
            %
            % See Also:
            %   :meth:`WBM.jointAccelerationsFPCEF`, :meth:`WBM.jointAccelerationsFPCPL` and
            %   :meth:`WBM.jointAccelerationsCLPCEF`.
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

                                                                                                       % HPCEF ... Hand Pose Corrections with External Forces (at the hands)
        function [ddq_j, fd_prms] = jointAccelerationsHPCEF(obj, hand_conf, tau, fe_h, ac_h, varargin)
            % Calculates the *joint angle accelerations* :math:`\ddot{q}_j` with
            % *hand pose corrections* and additionally *external forces* (*HPCEF*)
            % that are acting on the hands of the humanoid robot.
            %
            % The joint accelerations :math:`\ddot{q}_j` will be calculated as
            % defined in equation :eq:`joint_accelerations`. For a *closed-loop
            % control system* with *velocity* and *position regulation*, the
            % calculated joint acceleration vector depends on the given *contact
            % constraints* and the additional *external forces* that are acting
            % on the hands of the floating-base robot.
            %
            % The method assumes that *only the hands* of the robot may have
            % contact with the *ground/object/wall* and can be called in three
            % different modes:
            %
            % **Normal mode** -- Computes the joint accelerations, specified by
            % the base orientation, the positions and the velocities:
            %
            %   .. py:method:: jointAccelerationsHPCEF(hand_conf, tau, fe_h, ac_h, wf_R_b, wf_p_b, q_j, dq_j, v_b, nu)
            %
            % **Semi-optimized mode:**
            %
            %   .. py:method:: jointAccelerationsHPCEF(hand_conf, tau, fe_h, ac_h, wf_R_b, wf_p_b, q_j, nu)
            %
            % **Optimized mode** -- Computes the joint accelerations at the current
            % state of the robot system, in dependency of the given whole-body
            % dynamics and velocities:
            %
            %   - .. py:method:: jointAccelerationsHPCEF(hand_conf, tau, fe_h, ac_h, Jc, djcdq, M, c_qv[, dq_j], nu)
            %   - .. py:method:: jointAccelerationsHPCEF(hand_conf, tau, fe_h, ac_h, nu)
            %
            % Arguments:
            %   hand_conf         (struct): Configuration structure to specify the *qualitative
            %                               state* of the hands.
            %
            %                               The data structure specifies which hand is currently
            %                               in contact with the ground, or an object, or a wall.
            %                               It specifies also the *desired poses*, *angular
            %                               velocities* and *control gains* for the
            %                               position-regulation system of the hands.
            %   tau       (double, vector): :math:`(n \times 1)` torque force vector
            %                               for the joints and the base of the robot
            %                               with :math:`n = n_{dof} + 6`.
            %   fe_h      (double, vector): :math:`(k \times 1)` vector of external forces
            %                               (in contact space) that are acting on the
            %                               specified *contact points* of the hands.
            %
            %                               **Note:** The *external forces* are either
            %                               *constant* or *zero*.
            %   ac_h      (double, vector): :math:`(k \times 1)` mixed acceleration vector
            %                               for the specified *hand contact points*.
            %
            %                               **Note:** If the hand accelerations are very small,
            %                               then the vector can also be *constant* or *zero*.
            %   wf_R_b    (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                               (orientation) from the base frame *b*
            %                               to world frame *wf* (*optional*).
            %   wf_p_b    (double, vector): :math:`(3 \times 1)` position vector from
            %                               the base frame *b* to the world frame *wf*
            %                               (*optional*).
            %   q_j       (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                               vector in :math:`[\si{\radian}]` (*optional*).
            %   dq_j      (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                               vector in :math:`[\si{\radian/s}]` (*optional*).
            %   v_b       (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                               vector (*optional*).
            %   nu        (double, vector): :math:`((6 + n_{dof}) \times 1)` mixed generalized
            %                               velocity vector (generalized base velocity and
            %                               joint velocity).
            %
            % The variable :math:`k` indicates the *size* of the given *force* and
            % *acceleration vectors* in dependency of the specified hands:
            %
            %   - :math:`k = 6`  -- only one hand is defined.
            %   - :math:`k = 12` -- both hands are defined.
            %
            % If the joint velocity vector :math:`\dot{q}_j` (``dq_j``) is not
            % given as an argument, then the method assumes that the robot system
            % is *frictionless*.
            %
            % Other Parameters:
            %   Jc    (double, matrix): :math:`(6 m \times n)` Jacobian of the
            %                           *contact constraints* in contact space
            %                           :math:`\mathrm{C = \{C_1,\ldots,C_k\}}`
            %                           (*optional*).
            %   djcdq (double, vector): :math:`(6 m \times 1)` product vector of
            %                           the time derivative of the contact Jacobian
            %                           :math:`\dot{J}_c` and the joint velocity
            %                           :math:`\dot{q}_j` (*optional*).
            %   M     (double, matrix): :math:`(n \times n)` generalized mass matrix
            %                           of the robot (*optional*).
            %   c_qv  (double, vector): :math:`(n \times 1)` generalized bias force
            %                           vector of the robot (*optional*).
            %
            % The variable :math:`m` denotes the *number of contact constraints* of
            % the robot and :math:`n = n_{dof} + 6`.
            %
            % Returns:
            %   [ddq_j[, fd_prms]]: 2-element tuple containing:
            %
            %      - **ddq_j** (*double, vector*) -- :math:`(n_{dof} \times 1)` joint angle
            %        acceleration vector in :math:`[\si{\radian/{s^2}}]`.
            %      - **fd_prms**       (*struct*) -- Data structure for the parameter values
            %        of the forward dynamics calculation with the fields ``tau_gen``, ``f_c``,
            %        ``a_c`` and ``f_e`` (*optional*).
            %
            %        **Note:** The second output argument can be used for further calculations
            %        or for data logging.
            %
            % See Also:
            %   :meth:`WBM.jointAccelerationsCLPCEF`.
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
            % Returns a *zero contact acceleration vector* in dependency of the
            % *contact state* of the given contact links configuration.
            %
            % Arguments:
            %   clnk_conf (struct): Configuration structure to specify the *qualitative
            %                       state* of at most two *contact links*.
            %
            %                       The data structure specifies which link is currently
            %                       in contact with the ground or an object. It specifies
            %                       also the *desired poses*, *angular velocities* and
            %                       *control gains* for the position-regulation system
            %                       of the links.
            % Returns:
            %   ac_0 (double, vector): :math:`(k \times 1)` zero contact acceleration vector
            %   with the size of :math:`k = 6` or :math:`k = 12`.
            %
            % Note:
            %   If both contact links have contact with the ground, object or wall,
            %   (or both links have no contact at all) then the size of the vector
            %   is 12, otherwise the vector size is 6.
            nctc = uint8(clnk_conf.contact.left) + uint8(clnk_conf.contact.right);
            switch nctc
                case 1
                    ac_0 = obj.ZERO_CVEC_6;
                otherwise % if nctc = 0 or nctc = 2:
                    % either both contact links have contact or both links have
                    % no contact to the ground/object ...
                    ac_0 = obj.ZERO_CVEC_12;
            end
        end

        function fe_0 = zeroExtForces(obj, clnk_conf)
            % Returns a *zero external force vector* in dependency of the *contact
            % state* of the given contact links configuration.
            %
            % Arguments:
            %   clnk_conf (struct): Configuration structure to specify the *qualitative
            %                       state* of at most two *contact links*.
            %
            %                       The data structure specifies which link is currently
            %                       in contact with the ground or an object. It specifies
            %                       also the *desired poses*, *angular velocities* and
            %                       *control gains* for the position-regulation system
            %                       of the links.
            % Returns:
            %   fe_0 (double, vector): :math:`(k \times 1)` zero external force vector
            %   with the size of :math:`k = 6` or :math:`k = 12`.
            %
            % Note:
            %   If both contact links have contact with the ground, object or wall,
            %   (or both links have no contact at all) then the size of the vector
            %   is 12, otherwise the vector size is 6.
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
            % Creates the *foot links configuration* in dependency of the given
            % *contact state* of the feet.
            %
            % The method creates a data structure that specifies which foot of
            % the robot is currently in contact with the ground. If required,
            % the structure specifies also the *desired poses*, *angular
            % velocities* and the corresponding *control gains* for the
            % position-regulation system of the feet.
            %
            % The method can be called as follows:
            %
            %   - .. py:method:: footConfigState(cstate, vqT_lnk, k_p, k_v[, rtype])
            %   - .. py:method:: footConfigState(cstate, veT_lnk, k_p, k_v[, rtype])
            %   - .. py:method:: footConfigState(cstate, q_j, k_p[, rtype])
            %   - .. py:method:: footConfigState(cstate[, q_j[, rtype]])
            %
            % Arguments:
            %   cstate (logical, vector): :math:`(1 \times 2)` boolean vector pair
            %                             to indicate which foot is in contact with
            %                             the ground.
            %
            %                             First element:
            %                                Contact state of the *left foot*.
            %
            %                             Second element:
            %                                Contact state of the *right foot*.
            %
            %                             A foot touches the ground if the corresponding
            %                             value is set to *true*, otherwise *false*.
            %   vqT_lnk (double, vector): :math:`(7 \times 1)` VQ-transformation frame
            %                             (vector-quaternion frame) relative from the
            %                             given link frame *lnk* to the world frame *wf*
            %                             (*optional*).
            %   veT_lnk (double, vector): :math:`(6 \times 1)` VE-transformation frame
            %                             (vector-euler frame) relative from the given
            %                             link frame *lnk* to the world frame *wf*
            %                             (*optional*).
            %   k_p     (double, scalar): Stiffness control gain for the closed-loop system
            %                             (*optional*).
            %   k_v     (double, scalar): Damping control gain for the closed-loop system
            %                             (*optional*).
            %
            %                             **Note:** If the control gain is not defined, then
            %                             by default the method computes the gain value for
            %                             *critical damping* with :math:`k_v = 2\cdot \sqrt{k_p}`.
            %   rtype     (char, vector): Specifies the *rotation representation type* for the
            %                             desired poses of the links as reference. The rotation
            %                             can be represented either in quaternions ``quat`` or
            %                             in Euler-angles ``eul`` (default type: ``eul``).
            % Returns:
            %   foot_conf (struct): Configuration structure that specifies the current
            %   *qualitative state* of the feet.
            %
            % See Also:
            %   :meth:`WBM.clnkConfigState`.
            foot_conf = createConfigStateCL(obj, cstate, 'l_sole', 'r_sole', varargin{:});
        end

        function hand_conf = handConfigState(obj, cstate, cmode, varargin)
            % Creates the *hand links configuration* in dependency of the given
            % *contact state* of the hands.
            %
            % The method creates a data structure that specifies which hand of
            % the robot is currently in contact with the ground, or an object,
            % or a wall. If required, the structure specifies also the *desired
            % poses*, the *angular velocities* and the corresponding *control
            % gains* for the position-regulation system of the hands.
            %
            % The method can be called as follows:
            %
            %   - .. py:method:: footConfigState(cstate, cmode, vqT_lnk, k_p, k_v[, rtype])
            %   - .. py:method:: footConfigState(cstate, cmode, veT_lnk, k_p, k_v[, rtype])
            %   - .. py:method:: footConfigState(cstate, cmode, q_j, k_p[, rtype])
            %   - .. py:method:: footConfigState(cstate, cmode[, q_j[, rtype]])
            %
            % Arguments:
            %   cstate (logical, vector): :math:`(1 \times 2)` boolean vector pair
            %                             to indicate which hand is in contact with
            %                             the ground, or an object, or a wall.
            %
            %                             First element:
            %                                Contact state of the *left hand*.
            %
            %                             Second element:
            %                                Contact state of the *right hand*.
            %
            %                             A hand touches the ground/object/wall if
            %                             the corresponding value is set to *true*,
            %                             otherwise *false*.
            %   cmode     (char, vector): Defines the *contact mode* of the hands either with
            %                             the hand palms ``hand`` (*hand link frame*) or with
            %                             the finger tips ``gripper`` (*hand_dh_frame*).
            %   vqT_lnk (double, vector): :math:`(7 \times 1)` VQ-transformation frame
            %                             (vector-quaternion frame) relative from the
            %                             given link frame *lnk* to the world frame *wf*
            %                             (*optional*).
            %   veT_lnk (double, vector): :math:`(6 \times 1)` VE-transformation frame
            %                             (vector-euler frame) relative from the given
            %                             link frame *lnk* to the world frame *wf*
            %                             (*optional*).
            %   k_p     (double, scalar): Stiffness control gain for the closed-loop system
            %                             (*optional*).
            %   k_v     (double, scalar): Damping control gain for the closed-loop system
            %                             (*optional*).
            %
            %                             **Note:** If the control gain is not defined, then
            %                             by default the method computes the gain value for
            %                             *critical damping* with :math:`k_v = 2\cdot \sqrt{k_p}`.
            %   rtype     (char, vector): Specifies the *rotation representation type* for the
            %                             desired poses of the links as reference. The rotation
            %                             can be represented either in quaternions ``quat`` or
            %                             in Euler-angles ``eul`` (default type: ``eul``).
            % Returns:
            %   hand_conf (struct): Configuration structure that specifies the current
            %   *qualitative state* of the hands.
            %
            % See Also:
            %   :meth:`WBM.clnkConfigState`.
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
            % Returns a specific section of the computed the *visualization data
            % parameters* from the forward dynamics of the given robot model.
            %
            % The method returns a *dynamic data structure* with parameter fields
            % of a specified time period of the computed *forward dynamics states*
            % :math:`\mathcal{X}`, in dependency of the given forward dynamics
            % method and the torque controller. The calculated parameter values
            % of the controller and the forward dynamics can be used for the
            % visualization of certain parameters in a plot or for further
            % analysis purposes.
            %
            % Following *pose correction types* for the position-regulation
            % system of the forward dynamics will be supported:
            %
            %   - ``none`` -- No pose corrections.
            %   - ``nfb``  -- No floating base and without pose corrections.
            %   - ``fpc``  -- Foot pose correction.
            %   - ``hpc``  -- Hand pose correction.
            %   - ``fhpc`` -- Foot and hand pose correction.
            %
            % In dependency of the specified *pose correction*, the method can
            % be called as follows:
            %
            %   *none, nfb:*
            %      .. py:method:: getFDynVisDataSect(stmChi, fhTrqControl, start_idx, end_idx[, pc_type])
            %
            %   *none, fpc:*
            %      .. py:method:: getFDynVisDataSect(stmChi, fhTrqControl, start_idx, end_idx, foot_conf, clnk_conf, fe_c, ac, ac_f[, pc_type])
            %
            %   *none, fpc, fhpc:*
            %      .. py:method:: getFDynVisDataSect(stmChi, fhTrqControl, start_idx, end_idx, fhTotCWrench, foot_conf, hand_conf, f_cp, ac_f[, pc_type])
            %
            %   *nfb:*
            %      .. py:method:: getFDynVisDataSect(stmChi, fhTrqControl, start_idx, end_idx, clnk_conf, fe_c, ac[, pc_type])
            %
            %   *nfb:*
            %      .. py:method:: getFDynVisDataSect(stmChi, fhTrqControl, start_idx, end_idx, fhTotCWrench, hand_conf, f_cp[, pc_type])
            %
            %   *fpc:*
            %      .. py:method:: getFDynVisDataSect(stmChi, fhTrqControl, start_idx, end_idx, foot_conf, ac_f[, pc_type])
            %
            %   *hpc:*
            %      .. py:method:: getFDynVisDataSect(stmChi, fhTrqControl, start_idx, end_idx, hand_conf, fe_h, ac_h[, pc_type])
            %
            %   *fhpc:*
            %      .. py:method:: getFDynVisDataSect(stmChi, fhTrqControl, start_idx, end_idx, foot_conf, hand_conf, fe_h, ac_h, ac_f[, pc_type])
            %
            % Arguments:
            %   stmChi        (double, matrix): Integration output matrix :math:`\mathcal{X}`
            %                                   with the calculated *forward dynamics states*
            %                                   (row-vectors) of the given robot model.
            %   fhTrqControl (function_handle): Function handle to a specified time-dependent
            %                                   *torque control function* that controls the
            %                                   dynamics of the robot system.
            %   start_idx        (int, scalar): Time index position (row) of the integration
            %                                   output matrix :math:`\mathcal{X}`, where
            %                                   the section starts.
            %   end_idx          (int, scalar): Time index position (row) of the integration
            %                                   output matrix :math:`\mathcal{X}`, where
            %                                   the section ends.
            %   varargin        (cell, vector): Variable-length input argument list to pass extra
            %                                   parameters for the specified forward dynamics
            %                                   function to be integrated by the ODE-solver
            %                                   (*optional*).
            %
            %                                   The *extra parameters* to be passed are defined
            %                                   in the respective forward dynamics function
            %                                   (see :meth:`WBM.intForwardDynamics` -- *Other
            %                                   Parameters*).
            %   pc_type         (char, vector): String and last element of ``varargin`` to
            %                                   indicate the *pose correction type* to be used
            %                                   for the forward dynamics ODE-function, specified
            %                                   by one of the given string values: ``'none'``,
            %                                   ``'nfb'``, ``'fpc'``, ``'hpc'``, ``'fhpc'``.
            %
            %                                   The default string is ``'none'`` (*optional*).
            % Returns:
            %   vis_data (struct): Dynamic structure with a parameter data section of the
            %   torque controller and the forward dynamics of the given robot model for
            %   visualization and analysis purposes.
            %
            % See Also:
            %   :meth:`WBM.getFDynVisData`, :meth:`WBM.intForwardDynamics` and
            %   :meth:`WBM.visualizeForwardDynamics`.
            if ( (start_idx < 1)   || (end_idx < 1)   || ...
                 (start_idx > len) || (end_idx > len) || (start_idx > end_idx) )
                error('WBM::getFDynVisDataSect: %s', WBM.wbmErrorMsg.VAL_OUT_OF_BOUNDS);
            end
            vis_data = getFDynVisData(obj, stmChi(start_idx:end_idx,:), fhTrqControl, varargin{:});
        end

        sim_config = setupSimulation(~, sim_config, rot3d)

        [] = visualizeForwardDynamics(obj, stmPos, sim_config, sim_tstep, vis_ctrl)

        function simulateForwardDynamics(obj, stmPos, sim_config, sim_tstep, nRpts, vis_ctrl)
            % Simulates and visualizes the computed *forward dynamics*, i.e. the
            % motions, of a given robot model within a specified time interval
            % :math:`[t_0, t_f]`.
            %
            % Arguments:
            %   stmPos    (double, matrix): Position and orientation data from the *forward
            %                               dynamics states* of the integration output matrix
            %                               :math:`\mathcal{X}` of the given robot model.
            %   sim_config (:class:`~WBM.wbmSimConfig`): Configuration object with the configuration
            %                                            settings for the visualizer of the robot
            %                                            simulation.
            %   sim_tstep (double, scalar): Simulation time-step size to control the visualization
            %                               speed of the robot simulation.
            %
            %                               In order to keep the simulation close to real time, the
            %                               *simulation step size* :math:`t_{{step}_s}` is the same
            %                               *step size* :math:`t_{step}` of the given time interval
            %                               :math:`[t_0, t_f]` (see :meth:`WBM.intForwardDynamics`
            %                               -- ``tspan``).
            %
            %                               To slow down the visualization speed of the robot
            %                               simulation (slow motion), :math:`t_{{step}_s}` can
            %                               be increased by a *factor* :math:`x > 1` such that
            %                               :math:`t_{{step}_s} = x\cdot t_{step}`.
            %   nRpts        (int, scalar): Number of simulation repititions to be displayed on
            %                               the screen.
            %   vis_ctrl          (struct): Data structure to control the graphic elements of
            %                               the robot that should be drawn during the simulation
            %                               (*optional*).
            %
            %                               The structure is specified by following data fields:
            %
            %                                  - ``drawJnts`` (*logical, scalar*): Draw the joint nodes
            %                                    of the robot (default: *true*).
            %                                  - ``drawCom``  (*logical, scalar*): Draw the node of the
            %                                    center of mass (default: *true*).
            %                                  - ``drawSkel`` (*logical, scalar*): Draw the skeleton of
            %                                    the robot (default: *true*).
            %                                  - ``drawBody`` (*logical, scalar*): Draw the body parts,
            %                                    i.e. the hull, of the robot (default: *true*).
            %                                  - ``vis_speed`` (*double, scalar*): Visualization speed
            %                                    to keep the simulation close to real-time when the
            %                                    simulation step size is changed (default: 1.0).
            %
            %                               **Note:** If the control structure is not defined, then by
            %                               default all graphic elements of the robot are enabled and
            %                               will be drawn in the simulation. In dependency of the
            %                               complexity of the given robot model and the performance of
            %                               the computer this may affect the visualization speed of the
            %                               robot simulation.
            % See Also:
            %   :meth:`WBM.visualizeForwardDynamics`, :meth:`WBM.intForwardDynamics` and
            %   :meth:`WBM.getPositionsData`.
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
            % Plots the trajectory curve of the center of mass (CoM) of the
            % simulated floating-base robot.
            %
            % Arguments:
            %   stmPos (double, matrix): Position and orientation data from the *forward
            %                            dynamics states* of the integration output matrix
            %                            :math:`\mathcal{X}` of the given robot model.
            %   prop           (struct): Data structure to specify the plot properties of
            %                            trajectory curve of the CoM, i.e. the appearance
            %                            and behavior of the line object (*optional*).
            %
            %                            The plot properties are specified by following fields:
            %
            %                               - ``fwnd_title``        (*char, vector*): The figure-window
            %                                 title of the trajectory plot.
            %                               - ``title``             (*char, vector*): The title of the
            %                                 trajectory curve of the CoM.
            %                               - ``title_fnt_sz``    (*double, scalar*): Font size of the
            %                                 trajectory title, specified as a positive value in points
            %                                 (default value: 15).
            %                               - ``line_color`` (*double/char, vector*): Color of the trajectory
            %                                 curve, specified by a RGB-triplet or a color name (default
            %                                 color: ``'blue'``).
            %                               - ``spt_marker``      (*double, scalar*): Marker symbol for the
            %                                 start point of the trajectory curve. The symbols for the
            %                                 marker are the same as specified in Matlab (default symbol:
            %                                 ``'*'``).
            %                               - ``spt_color``       (*double, scalar*): Color of the trajectory
            %                                 start point, specified by a RGB-triplet or a color name
            %                                 (default color: ``'red'``).
            %                               - ``label_fnt_sz``    (*double, scalar*): Font size of the axis
            %                                 labels, specified as a positive value in points (default
            %                                 value: 15).
            %
            %                            **Note:** If the plot properties are not given, then the default
            %                            settings will be used.
            % See Also:
            %   :meth:`WBM.getPositionsData`.
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

            % draw the trajectory line:
            %         x-axis      y-axis      z-axis
            plot3(x_b(1:m,1), x_b(1:m,2), x_b(1:m,3), 'Color', prop.line_color);
            hold on;
            % mark the start point ...
            hsp = plot3(x_b(1,1), x_b(1,2), x_b(1,3), 'LineStyle', 'none', ...
                        'Marker', prop.spt_marker, 'MarkerEdgeColor', prop.spt_color);
            axis square;
            grid on;

            % add title and axis labels ...
            if ~isempty(prop.title)
                title(prop.title, 'Interpreter', 'latex', 'FontSize', prop.title_fnt_sz);
            end
            xlabel('$x_{\mathbf{x_b}}$', 'Interpreter', 'latex', 'FontSize', prop.label_fnt_sz);
            ylabel('$y_{\mathbf{x_b}}$', 'Interpreter', 'latex', 'FontSize', prop.label_fnt_sz);
            zlabel('$z_{\mathbf{x_b}}$', 'Interpreter', 'latex', 'FontSize', prop.label_fnt_sz);

            legend(hsp, 'init. position', 'Location', 'northeast');
        end

        function setPayloadLinks(obj, pl_lnk_data)
            % Assigns the data of the given payload objects to specific *payload
            % links* of the floating-base robot.
            %
            % Arguments:
            %   pl_lnk_data (struct/cell-array): :math:`(1 \times n_{plds})` data structures
            %                                    of the given *payload objects*, where each
            %                                    payload will be assigned to a specific link
            %                                    of the floating-base robot with
            %                                    :math:`n_{plds} \geq 1`.
            %
            %                                    If the robot has multiple payload objects for
            %                                    the simulation, then the data structures of
            %                                    each payload must be stored in a cell-array.
            %
            %                                    The data structure for the payload objects
            %                                    contains following fields:
            %
            %                                       - ``name``       (*char, vector*): URDF-name of the specified
            %                                         *reference link frame* or *contact link frame*. The link
            %                                         frame can be an end-effector frame of a hand or a special
            %                                         frame on which a payload object is mounted.
            %                                       - ``lnk_p_cm`` (*double, vector*): :math:`(3 \times 1)` Cartesian
            %                                         position vector relative from the center of mass *cm* of the
            %                                         payload object to the link frame *lnk*.
            %                                       - ``t_idx``     (*uint8, scalar*): Index number of a specified
            %                                         *tool link object* to define that the grabbed payload object is
            %                                         also a tool (*optional*).
            %
            %                                         **Note:** If the index value is 0, then the payload object is not
            %                                         linked with a tool.
            %                                       - ``vb_idx``    (*uint8, scalar*): Index number of a *volume body
            %                                         object* that will be linked to the given link frame (*optional*).
            %
            %                                         **Note:** If the index value is 0, then the payload is not assigned
            %                                         to any given geometric volume body of the simulation environment.
            %                                       - ``m_rb``     (*double, scalar*): Mass of the rigid body (solid
            %                                         volume body) in :math:`[\si{kg}]` (*optional*).
            %
            %                                            - ``I_cm`` (*double, matrix*): :math:`(3 \times 3)` *inertia
            %                                              tensor* of a rigid body or geometric object with the origin
            %                                              of the coordinate system at the center of mass (CoM) of the
            %                                              object body.
            %
            %                                         **Note:** The mass ``m_rb`` is linked with the inertia ``I_cm`` at
            %                                         the CoM of the given rigid body. So if ``m_rb`` is undefined or the
            %                                         value is zero, then also ``I_cm`` is undefined. This can be helpful
            %                                         if the mass of the object is currently unknown.
            %
            % Note:
            %   The indices 1 and 2 are always reserved for the *left* and the
            %   *right hand* of a given humanoid robot. The default index that
            %   will be used by the :class:`WBM` class is always 1. All further
            %   indices (> 2) of the array can be used for other links, on which
            %   additionally special payloads are mounted (e.g. a battery pack
            %   or a knapsack at the torso, special tools, etc.).
            %
            % See Also:
            %   :meth:`WBM.getPayloadLinks` and :class:`~WBM.wbmPayloadLink`.

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
            % Returns all defined payload links of the floating-base robot with
            % the assigned payload objects.
            %
            % Returns:
            %   [pl_links, nPlds]: 2-element tuple containing:
            %
            %      - **pl_links** (:class:`~WBM.wbmPayloadLink`, *vector*) -- :math:`(1 \times n_{plds})`
            %        array of *payload link objects*, where each payload is assigned to a
            %        specific link of the robot (default: *empty*).
            %      - **nPlds** (*uint8, scalar*) -- Number of payloads that are assigned
            %        to specific links of the robot.
            %
            % See Also:
            %   :meth:`WBM.setPayloadLinks` and :class:`~WBM.wbmPayloadLink`.
            pl_links = obj.mwbm_config.payload_links;
            nPlds    = obj.mwbm_config.nPlds;
        end

        function pl_tbl = getPayloadTable(obj)
            % Creates a table of the defined payload links with the assigned
            % payload objects.
            %
            % The output table lists all data values of the each defined payload
            % link, specified by following column names as variables: ``link_name``,
            % ``lnk_p_cm``, ``t_idx``, ``vb_idx``, ``mass``, ``inertia``.
            %
            % Returns:
            %   pl_tbl (table): Table array with named variables for the payload
            %   object data of all defined payload links of the robot.
            %
            % See Also:
            %   :meth:`WBM.setPayloadLinks` and :class:`~WBM.wbmPayloadLink`.
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
            % Computes the *frame*, i.e. the transformation matrix :math:`H`, of
            % a specified *payload link* w.r.t. the current joint configuration
            % :math:`q_j`.
            %
            % The homogeneous transformation of the grabbed or selected payload,
            % relative from the payload's center of mass *cm* to the world frame
            % *wf*, will be obtained as follows:
            %
            %   .. math::
            %       :label: payload_frame
            %
            %       {}^{wf}H_{cm} = {}^{wf}H_{lnk}\cdot {}^{lnk}H_{cm}
            %
            % The method can be called by one of the given modes:
            %
            % **Normal mode** -- Computes the transformation matrix of the given
            % payload link, specified by the base orientation and the positions:
            %
            %   .. py:method:: payloadFrame(wf_R_b, wf_p_b, q_j[, pl_idx])
            %
            % **Optimized mode** -- Computes the transformation matrix of the
            % given payload link at the current state of the robot system:
            %
            %   .. py:method:: payloadFrame([pl_idx])
            %
            % Arguments:
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf*.
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*.
            %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint
            %                            positions vector in :math:`[\si{\radian}]`.
            %   pl_idx  (uint8, scalar): Index number of the specified *payload
            %                            link object* of the robot (*optional*).
            %
            %                            **Note:** If the index of the payload link
            %                            object is undefined, then the first element
            %                            of the list will be used as default.
            % Note:
            %   It is assumed that the orientation of the payload frame *pl* has
            %   the same orientation as the given payload link frame *lnk* of the
            %   robot, i.e. the link of an end-effector (hand, finger, etc.) or
            %   another specific link (torso, leg, etc.) where the payload is
            %   mounted on that body part.
            %
            % Returns:
            %   wf_H_cm (double, matrix): :math:`(4 \times 4)` homogeneous transformation
            %   matrix relative from the payload's center of mass *cm* to the world frame
            %   *wf*.
            %
            % See Also:
            %   :meth:`WBMBase.transformationMatrix`.
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
            % get the homog. transformation of the payload frame centered at the
            % CoM (to the link frame):
            lnk_H_cm = eye(4,4);
            lnk_H_cm(1:3,4) = lnk_p_cm; % position from the payload's CoM to the frame {lnk}.

            wf_H_cm = wf_H_lnk * lnk_H_cm; % payload transformation matrix
        end

        f_pl = handPayloadForces(obj, hand_conf, fhTotCWrench, f_cp, wf_v_lnk, wf_a_lnk)

        function f_pl = dynPayloadForce(obj, pl_idx, wf_v_lnk, wf_a_lnk, w_e)
            % Computes the *dynamic payload force* :math:`f_{pl}`, i.e. the payload
            % wrench that is acting on the contact point :math:`p_{\small C_i}`
            % of the payload object, in dependency of a given external wrench
            % :math:`w_e`.
            %
            % For the simplification of the distances, the given payload link frame
            % *lnk* represents the contact frame :math:`\mathrm{C_{i \in \{1,\ldots,n\}}}`
            % at the contact point :math:`p_{\small C_i}`.
            %
            % Arguments:
            %   pl_idx    (uint8, scalar): Index number of the specified *payload
            %                              link object* of the robot (*optional*).
            %   wf_v_lnk (double, vector): :math:`(6 \times 1)` mixed velocity
            %                              vector from the link frame *lnk* to
            %                              the world frame *wf*.
            %   wf_a_lnk (double, vector): :math:`(6 \times 1)` mixed acceleration
            %                              vector from the link frame *lnk* to the
            %                              world frame *wf*.
            %   w_e      (double, vector): :math:`(6 \times 1)` external wrench
            %                              vector that is acting on the contact
            %                              point :math:`p_{\small C_i}` (which is
            %                              at the link frame *lnk*).
            % Note:
            %   All input values must be from the same reference frame *lnk*.
            %
            % Returns:
            %   f_pl (double, vector): :math:`(6 \times 1)` dynamic payload force
            %   vector at the specified payload link of the robot.
            %
            % See Also:
            %   :func:`utilities.mbd.payloadWrenchRB` and :meth:`WBM.generalizedInertiaPL`.
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

            % calculate the acting force of the payload object
            % at the current contact point p_c:
            M_pl = generalizedInertiaPL(obj, pl_idx); % = lnk_M_cm
            f_pl = WBM.utilities.mbd.payloadWrenchRB(M_pl, v_pl, a_pl, w_e); % = f_lnk
        end

                                                          % PL ... Payload
        function M_pl = generalizedInertiaPL(obj, pl_idx)
            % Computes the *generalized inertia matrix* of the given payload object
            % at the contact point :math:`p_{\small C_i}` that is associated with
            % a specified payload link *lnk*.
            %
            % The method applies a simplified position and orientation determination
            % for the center of mass (CoM) of the payload, such that
            %
            %   .. math::
            %       :label: inertia_pos_rotm
            %
            %       &{}^{lnk}R_{cm} = {}^{lnk}R_{\small C_i}\cdot {}^{\small C_i}R_{cm} \quad\text{and}\\
            %       &{}^{lnk}p_{cm} = {}^{lnk}R_{\small C_i}\cdot {}^{\small C_i}p_{cm}\:,
            %
            % where :math:`{}^{lnk}R_{cm} = {}^{lnk}R_{\small C_i} = {}^{\small C_i}R_{cm} = \mathbb{I}_3`
            % and :math:`{}^{lnk}p_{cm} = {}^{\small C_i}p_{cm}`. The contact frame
            % :math:`\mathrm{C_{i \in \{1,\ldots,n\}}}` and the payload frame *cm*
            % (centered at CoM) have the same orientation as the reference link frame
            % *lnk*. The contact point :math:`p_{\small C_i} = {}^{lnk}p_{cm}` is set
            % at the origin :math:`o_{lnk}` of the reference link frame.
            %
            % Note:
            %   The given position for the object's CoM must be from the same
            %   reference frame *lnk*.
            %
            % Arguments:
            %   pl_idx    (uint8, scalar): Index number of the specified *payload
            %                              link object* of the robot (*optional*).
            % Returns:
            %   M_pl (double, matrix): :math:`(6 \times 6)` generalized inertia matrix
            %   of the given payload object (from the payload frame *cm* to the link
            %   frame *lnk*).
            %
            % See Also:
            %   :func:`utilities.rb.generalizedInertia`.
            m_rb     = obj.mwbm_config.payload_links(1,pl_idx).m_rb;
            lnk_p_cm = obj.mwbm_config.payload_links(1,pl_idx).lnk_p_cm;

            if (m_rb == 0)
                M_pl = zeros(6,6);
            else
                % generalized inertia M_pl = lnk_M_cm of the payload object at
                % the contact point p_c = lnk_p_cm with lnk_R_cm = I_3:
                I_cm = obj.mwbm_config.payload_links(1,pl_idx).I_cm;
                M_pl = WBM.utilities.rb.generalizedInertia(m_rb, I_cm, lnk_p_cm);
            end
        end

        function setToolLinks(obj, ee_lnk_names, ee_vqT_tt, pl_idx)
            % Assigns the data of the given tools to specific *tool links*
            % (end-effectors) of the floating-base robot.
            %
            % The method can be called as follows:
            %
            %   .. py:method:: setToolLinks(ee_lnk_names, ee_vqT_tt[, pl_idx])
            %
            % It assigns the *tool-tip frames* (tt) of the given tools to specific
            % *end-effector links* (ee) of the robot. If a given payload object is
            % at the same time also a tool, then the method links the tool object
            % with the corresponding payload object.
            %
            % Arguments:
            %   ee_link_names (cellstr, vector): :math:`(1 \times n)` array of string
            %                                    matching URDF-names of the *end-effector
            %                                    links*.
            %   ee_vqT_tt      (double, matrix): :math:`(7 \times n)` frame matrix for the
            %                                    VQ-transformation frames of the given tools,
            %                                    relative from the tool-tip frames *tt* to
            %                                    the link frames of the end-effectors *ee*.
            %   pl_idx          (uint8, vector): :math:`(1 \times n)` index number array
            %                                    of the specified *payload link objects*,
            %                                    to define that specific payload objects
            %                                    are also tools (default array: *array of
            %                                    zeros*) -- *optional*.
            %
            %                                    **Note:** If an index value of the array
            %                                    is set to 0, then the tool is not linked
            %                                    with a given payload object.
            % Note:
            %   All input arrays must have the same length :math:`n` with
            %   :math:`1 \leq n \leq \mathrm{MAX\_NUM\_TOOLS}`.
            %
            % See Also:
            %   :class:`~WBM.wbmToolLink`, :meth:`WBM.getToolLinks` and
            %   :attr:`WBM.MAX_NUM_TOOLS`.

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

            % set the end-effector link names and the tool-tip frames:
            % if some grabbed payload objects are also tools, link the
            % tool links with the corresponding payload objects.
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
            % Returns all defined tool links of the floating-base robot with the
            % assigned tool objects.
            %
            % Returns:
            %   [tool_links, nTools]: 2-element tuple containing:
            %
            %      - **tool_links** (:class:`~WBM.wbmToolLink`, *vector*) -- :math:`(1 \times n_{tools})`
            %        array of *tool link objects*, where each tool is assigned to a specific
            %        end-effector link of the robot (default: *empty*).
            %      - **nTools** (*uint8, scalar*) -- Number of tools that are assigned to
            %        specific end-effector links of the robot.
            %
            % See Also:
            %   :meth:`WBM.setToolLinks` and :class:`~WBM.wbmToolLink`.
            tool_links = obj.mwbm_config.tool_links;
            nTools     = obj.mwbm_config.nTools;
        end

        function tool_tbl = getToolTable(obj)
            % Creates a table of the defined tool links with the assigned tool
            % objects.
            %
            % The output table lists all data values of each defined tool link,
            % specified by following column names as variables: ``link_name``,
            % ``ee_vqT_tt``, ``pl_idx``.
            %
            % Returns:
            %   tool_tbl (table): Table array with named variables for the tool
            %   object data of all defined tool links of the robot.
            %
            % See Also:
            %   :meth:`WBM.setToolLinks` and :class:`~WBM.wbmToolLink`.
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
            % Updates the tool frame, i.e. the VQ-transformation, of the
            % selected tool.
            %
            % Arguments:
            %   ee_vqT_tt (double, vector): :math:`(7 \times 1)` VQ-transformation frame
            %                               of the tool, relative from the tool-tip frame
            %                               *tt* to the link frame of the end-effector *ee*.
            %   t_idx      (uint8, scalar): Index number of the *tool link object* in the
            %                               list that should be updated.
            % Note:
            %   The index number :math:`i_t` of the tool link object must be in
            %   the range of :math:`1 \leq i_t \leq \mathrm{MAX\_NUM\_TOOLS}`.
            %
            % See Also:
            %   :class:`~WBM.wbmToolLink`, :meth:`WBM.getToolLinks` and
            %   :attr:`WBM.MAX_NUM_TOOLS`.
            if (obj.mwbm_config.nTools == 0)
                error('WBM::updateToolFrame: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end
            if (t_idx > obj.MAX_NUM_TOOLS)
                error('WBM::updateToolFrame: %s', WBM.wbmErrorMsg.MAX_NUM_LIMIT);
            end
            if (size(ee_vqT_tt,1) ~= 7)
                error('WBM::updateToolFrame: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            % update the tool frame ...
            obj.mwbm_config.tool_links(t_idx,1).ee_vqT_tt = ee_vqT_tt;
        end

        function wf_H_tt = toolFrame(obj, varargin)
            % Computes the *tool frame*, i.e. the transformation matrix :math:`H`,
            % of a specified tool link w.r.t. the current joint configuration
            % :math:`q_j`.
            %
            % The homogeneous transformation of the chosen tool, relative from
            % the tool-tip frame *tt* to the world frame *wf*, will be obtained
            % as follows:
            %
            %   .. math::
            %       :label: tool_frame
            %
            %       {}^{wf}H_{tt} = {}^{wf}H_{ee}\cdot {}^{ee}H_{tt}
            %
            % The method can be called by one of the given modes:
            %
            % **Normal mode** -- Computes the transformation matrix of the given
            % tool link, specified by the base orientation and the positions:
            %
            %   .. py:method:: toolFrame(wf_R_b, wf_p_b, q_j[, t_idx])
            %
            % **Optimized mode** -- Computes the transformation matrix of the
            % given tool link at the current state of the robot system:
            %
            %   .. py:method:: toolFrame([t_idx])
            %
            % Arguments:
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf*.
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*.
            %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint
            %                            vector in :math:`[\si{\radian}]`.
            %   t_idx   (uint8, scalar): Index number of the specified *tool link
            %                            object* of the robot (*optional*).
            %
            %                            **Note:** If the index of the tool link
            %                            object is not given, then the first element
            %                            of the list will be used as default.
            % Note:
            %   In general the orientation of the tool frame *tt* has not the
            %   same orientation as the frame of the end-effector *ee* of a
            %   hand or of a finger.
            %
            % Returns:
            %   wf_H_tt (double, matrix): :math:`(4 \times 4)` homogeneous transformation
            %   matrix relative from the tool-tip frame *tt* to the world frame *wf*.
            %
            % See Also:
            %   :meth:`WBMBase.transformationMatrix`.
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
            % get the homog. transformation of the tool-tip frame (to the ee-frame):
            ee_H_tt = WBM.utilities.tfms.frame2tform(ee_vqT_tt);
            wf_H_tt = wf_H_ee * ee_H_tt; % tool transformation matrix
        end

        function wf_J_tt = jacobianTool(obj, varargin)
            % Computes the *Jacobian* of a specified *tool link* of the robot
            % w.r.t. the current joint configuration :math:`q_j`.
            %
            % The Jacobian of the given tool link will be obtained by applying
            % the *velocity transformation matrix*
            %
            %   .. math::
            %      :label: velocity_tform
            %
            %      {}^{tt[wf]}X_{ee[wf]} =
            %      \begin{bmatrix}
            %         \mathbb{I}_3 & \text{-}S({}^{wf}R_{ee}\cdot {}^{ee}p_{tt})\cdot \mathbb{I}_3\\
            %         \mathbf{0}   & \mathbb{I}_3
            %      \end{bmatrix}\:,
            %
            % such that :math:`{}^{wf}J_{tt} = {}^{tt[wf]}X_{ee[wf]}\cdot {}^{ee[wf]}X_{ee}`, where
            % the notations :math:`tt[wf]` and :math:`ee[wf]` denoting the frames with origin
            % :math:`o_{tt}` and :math:`o_{ee}` with the orientation :math:`[wf]`. The transformation
            % matrix :math:`X` maps the velocities of the geometric Jacobian in frame *ee* to the
            % velocities in frame *tt*.
            %
            % The method can be called by one of the given modes:
            %
            % **Normal mode** -- Computes the Jacobian matrix of the given tool
            % link, specified by the base orientation and the positions:
            %
            %   .. py:method:: jacobianTool(wf_R_b, wf_p_b, q_j[, t_idx])
            %
            % **Optimized mode** -- Computes the Jacobian matrix of the given
            % tool link at the current state of the robot system:
            %
            %   .. py:method:: jacobianTool([t_idx])
            %
            % Arguments:
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf*.
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*.
            %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                            vector in :math:`[\si{\radian}]`.
            %   t_idx   (uint8, scalar): Index number of the specified *tool link
            %                            object* of the robot (*optional*).
            %
            %                            **Note:** If the index of the tool link
            %                            object is not given, then the first element
            %                            of the list will be used as default.
            % Returns:
            %   wf_J_tt (double, matrix): :math:`(6 \times n)` Jacobian matrix relative from
            %   the tool-tip frame *tt* to the world frame *wf* with :math:`n = n_{dof} + 6`.
            %
            % See Also:
            %   :meth:`WBMBase.jacobian` and :func:`utilities.tfms.adjoint`.
            %
            % References:
            %   - :cite:`Traversaro2016`, p. 6, eq. (27).
            %   - :cite:`Siciliano2010`, p. 150, eq. (3.112).
            %   - :cite:`Craig2005`, p. 158, eq. (5.103).

            % References:
            %   [TS16]   Traversaro, S.; Saccon, A.: Multibody Dynamics Notation.
            %            Eindhoven University of Technology, Department of Mechanical Engineering,
            %            2016, p. 6, eq. (27), URL: <https://pure.tue.nl/ws/portalfiles/portal/25753352>.
            %   [SSVO10] Siciliano, B.; Sciavicco, L.; Villani, L.; Oriolo, G.:
            %            Robotics: Modelling, Planning and Control. Springer, 2010,
            %            p. 150, eq. (3.112).
            %   [Cra05]  Craig, John J.: Introduction to Robotics: Mechanics and Control.
            %            3rd Edition, Pearson/Prentice Hall, 2005, p. 158, eq. (5.103).
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
            %
            %  The transformation matrix X maps the velocities of the geometric
            %  Jacobian in frame {ee} to velocities in frame {tt} and is defined
            %  as
            %
            %                         | I    -S(wf_R_ee * ee_p_tt)*I |
            %       tt[wf]_X_ee[wf] = |                              | ,
            %                         | 0                I           |
            %
            %  s.t. wf_J_tt = tt[wf]_X_ee[wf] * ee[wf]_J_ee, where the notations
            %  tt[wf] and ee[wf] denoting the frames with origin o_tt and o_ee
            %  with the orientation [wf].
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
            % Returns the current *joint configuration states* of those chains of
            % the robot body that are specified in the given list of *chain names*.
            %
            % The method can be called as follows:
            %
            %   .. py:method:: getStateJntChains(chain_names[, q_j, dq_j])
            %
            % Arguments:
            %   chain_names (cellstr, vector): Column-array with :math:`k \geq 1` string
            %                                  matching *chain names* of the robot's effectors.
            %   q_j          (double, vector): :math:`(n_{dof} \times 1)` vector of the
            %                                  current joint positions of the robot in
            %                                  :math:`[\si{\radian}]` (*optional*).
            %   dq_j         (double, vector): :math:`(n_{dof} \times 1)` vector of the
            %                                  current joint velocities of the robot in
            %                                  :math:`[\si{\radian/s}]` (*optional*).
            % Returns:
            %   [chn_q, chn_dq]: 2-element tuple containing:
            %
            %      - **chn_q** (*cell, vector*) -- :math:`(k \times 1)` array of joint
            %        position vectors of the specified chain names of the robot body.
            %      - **chn_dq** (*cell, vector*) -- :math:`(k \times 1)` array of joint
            %        velocity vectors of the specified chain names of the robot body.
            %
            %   **Note:** The order of the joint position and velocity vectors are in
            %   the same order as the given chain names list.
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
            % Returns the current *joint configuration states* of those joints of
            % the robot that are specified in the given *joint names list*.
            %
            % The method can be called as follows:
            %
            %   .. py:method:: getStateJointNames(joint_names[, q_j, dq_j])
            %
            % Arguments:
            %   joint_names (cellstr, vector): Column-array with :math:`k \geq 1` string
            %                                  matching *joint names* of the robot.
            %   q_j          (double, vector): :math:`(n_{dof} \times 1)` vector of the
            %                                  current joint positions of the robot in
            %                                  :math:`[\si{\radian}]` (*optional*).
            %   dq_j         (double, vector): :math:`(n_{dof} \times 1)` vector of the
            %                                  current joint velocities of the robot in
            %                                  :math:`[\si{\radian/s}]` (*optional*).
            % Returns:
            %   [jnt_q, jnt_dq]: 2-element tuple containing:
            %
            %      - **jnt_q** (*double, vector*) -- :math:`(k \times 1)` joint position
            %        vector of the specified joint names of the robot.
            %      - **jnt_dq** (*double, vector*) -- :math:`(k \times 1)` joint velocity
            %        vector of the specified joint names of the robot.
            %
            %   **Note:** The order of the joint positions and velocities are in
            %   the same order as the given joint names list.
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
            % Returns the current *joint configuration states* of those joints of
            % the robot that are specified in the given *joint index list*.
            %
            % The method can be called as follows:
            %
            %   .. py:method:: getStateJointIdx(jnt_idx[, q_j, dq_j])
            %
            % Arguments:
            %   jnt_idx (int, vector): :math:`(k \times 1)` vector with the index
            %                          numbers of the specified joints of the
            %                          robot model in ascending order.
            %   q_j  (double, vector): :math:`(n_{dof} \times 1)` vector of the
            %                          current joint positions of the robot in
            %                          :math:`[\si{\radian}]` (*optional*).
            %   dq_j (double, vector): :math:`(n_{dof} \times 1)` vector of the
            %                          current joint velocities of the robot in
            %                          :math:`[\si{\radian/s}]` (*optional*).
            % Returns:
            %   [jnt_q, jnt_dq]: 2-element tuple containing:
            %
            %      - **jnt_q** (*double, vector*) -- :math:`(k \times 1)` joint position
            %        vector of the specified joint indices of the robot.
            %      - **jnt_dq** (*double, vector*) -- :math:`(k \times 1)` joint velocity
            %        vector of the specified joint indices of the robot.
            %
            %   **Note:** The order of the joint positions and velocities are in
            %   the same order as the given joint names list.
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
            % Returns the *state parameters* of the computed *forward dynamics
            % states* of the robot model.
            %
            % Arguments:
            %   stChi (double, vector/matrix): Integration output vector or matrix
            %                                  :math:`\mathcal{X}` containing the
            %                                  computed *forward dynamic states* of
            %                                  the robot model.
            %
            %                                  **Note:** In dependency of the situation,
            %                                  the integration output :math:`\mathcal{X}`
            %                                  represents either a *single state*
            %                                  (column-vector) or a *time sequence of
            %                                  states* (matrix).
            % Returns:
            %   stParams (:class:`~WBM.wbmStateParams`): Data object with the state
            %   parameters of the forward dynamics states of the robot model.
            %
            %   **Note:** If the given integration output ``stChi`` is a *time
            %   sequence of states* then the state variables of the data object
            %   are matrices otherwise, column-vectors.
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
            % Returns the *base frames* and *joint positions* of the computed
            % *forward dynamics states* of the floating-base robot.
            %
            % Arguments:
            %   stChi (double, vector/matrix): Integration output vector or matrix
            %                                  :math:`\mathcal{X}` containing the
            %                                  computed *forward dynamic states* of
            %                                  the robot model.
            %
            %                                  **Note:** In dependency of the situation,
            %                                  the integration output :math:`\mathcal{X}`
            %                                  represents either a *single state*
            %                                  (column-vector) or a *time sequence of
            %                                  states* (matrix).
            % Returns:
            %   [vqT_b, q_j]: 2-element tuple containing:
            %
            %      - **vqT_b** (*double, vector/matrix*) -- Base frames array (base
            %        VQ-transformations) of the floating-base robot.
            %      - **q_j** (*double, vector/matrix*) --  Joint positions array
            %        of the floating-base robot in :math:`[\si{\radian}]`.
            %
            %   **Note:** If the given integration output ``stChi`` is a *time
            %   sequence of states*, then the return values are data matrices. If
            %   ``stChi`` represents only a *single state*, the return values are
            %   column-vectors.
            len  = obj.mwbm_config.stvLen;
            cutp = obj.mwbm_model.ndof + 7; % 3 + 4 + ndof

            if iscolumn(stChi)
                WBM.utilities.chkfun.checkCVecDim(stChi, len, 'WBM::getPositions');
                % extract the base VQ-transformation
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
            % Returns the *position* and *orientation data* of a given time
            % sequence of *forward dynamics states* of the robot model.
            %
            % Arguments:
            %   stmChi (double, matrix): Integration output matrix :math:`\mathcal{X}`
            %                            with the calculated *forward dynamics states*
            %                            (row-vectors) of the given robot model.
            % Returns:
            %   stmPos (double, matrix): :math:`(m \times (n_{dof} + 7))` data matrix
            %   with position and orientation data :math:`[x_b, qt_b, q_j]` of the
            %   forward dynamics states of the robot model.
            [m, n] = size(stmChi);
            if (n ~= obj.mwbm_config.stvLen)
                error('WBM::getPositionsData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            cutp   = obj.mwbm_model.ndof + 7; % 3 + 4 + ndof
            stmPos = stmChi(1:m,1:cutp);      % m -by- [x_b, qt_b, q_j]
        end

        function [v_b, dq_j] = getMixedVelocities(obj, stChi)
            % Returns the *mixed velocities* of the computed *forward dynamics
            % states* of the robot model.
            %
            % Arguments:
            %   stChi (double, vector/matrix): Integration output vector or matrix
            %                                  :math:`\mathcal{X}` containing the
            %                                  computed *forward dynamic states* of
            %                                  the robot model.
            %
            %                                  **Note:** In dependency of the situation,
            %                                  the integration output :math:`\mathcal{X}`
            %                                  represents either a *single state*
            %                                  (column-vector) or a *time sequence of
            %                                  states* (matrix).
            % Returns:
            %   [v_b, dq_j]: 2-element tuple containing:
            %
            %      - **v_b** (*double, vector/matrix*) -- Generalized base velocities
            %        array (Cartesian and rotational velocities of the base).
            %      - **dq_j** (*double, vector/matrix*) --  Joint velocities array of
            %        the robot model in :math:`[\si{\radian/s}]`.
            %
            %   **Note:** If the given integration output ``stChi`` is a *time
            %   sequence of states*, then the return values are data matrices. If
            %   ``stChi`` represents only a *single state*, the return values are
            %   column-vectors.
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
                dq_j = stChi(1:m,ndof+14:len);    % m -by- dq_j
                return
            end
            % else ...
            error('WBM::getMixedVelocities: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
        end

        function v_b = getBaseVelocities(obj, stChi)
            % Returns the *generalized base velocities* of the computed *forward
            % dynamics states* of the robot model.
            %
            % Arguments:
            %   stChi (double, vector/matrix): Integration output vector or matrix
            %                                  :math:`\mathcal{X}` containing the
            %                                  computed *forward dynamic states* of
            %                                  the robot model.
            %
            %                                  **Note:** In dependency of the situation,
            %                                  the integration output :math:`\mathcal{X}`
            %                                  represents either a *single state*
            %                                  (column-vector) or a *time sequence of
            %                                  states* (matrix).
            % Returns:
            %   v_b (double, vector/matrix): Generalized base velocities array of the
            %   forward dynamics states of the robot model.
            %
            %   **Note:** If the given integration output ``stChi`` is a *time sequence
            %   of states* then ``v_b`` is a matrix, otherwise a column-vector.
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
            % Displays the current configuration settings of the given
            % floating-base robot.
            %
            % Arguments:
            %   prec (int, scalar): Precision number to specify for the values
            %                       the number of digits after the decimal point
            %                       (default precision: 2) -- *optional*.
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
