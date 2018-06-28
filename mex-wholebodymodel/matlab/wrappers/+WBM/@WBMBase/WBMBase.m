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

classdef WBMBase < handle
    % The class :class:`!WBMBase` is the *base class* of the *whole-body model*
    % (WBM) for YARP-based floating-base robots, such as the iCub humanoid robot.
    % It is a *wrapper class* for the fast *C++ MEX Whole-Body Model Interface*
    % for Matlab (`mex-wholeBodyModel`_) and provides all necessary basic methods
    % of the `yarpWholeBodyInterface`_ for state estimations, kinematic/dynamic
    % models and actuators.
    %
    % Attributes:
    %   fixed_link    (char, vector): String matching URDF name of the default *fixed
    %                                 reference link frame* (floating-base link).
    %
    %                                 **Note:** The default fixed link (floating-base
    %                                 link) may be different for each robot and the
    %                                 selection of the floating base link depends also
    %                                 from the given situation. Furthermore, the
    %                                 specified fixed link (reference link) can also
    %                                 be a *contact constraint link*.
    %   init_wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix as
    %                                 *initial orientation* from base frame *b*
    %                                 to the world frame *wf* (default orientation:
    %                                 *identity matrix*).
    %   init_wf_p_b (double, vector): :math:`(3 \times 1)` vector to specify the
    %                                 *initial position* from the base frame *b*
    %                                 to the world frame *wf* (default position:
    %                                 :math:`[0, 0, 0]^T`).
    %   g_wf        (double, vector): :math:`(3 \times 1)` Cartesian gravity vector
    %                                 in the world frame *wf*. If the gravity is not
    %                                 defined, then the gravity vector is by default
    %                                 a 0-vector.
    %   ndof        (uint16, scalar): Number of degrees of freedom of the given robot.
    %   frict_coeff         (struct): Data structure for the friction coefficients of
    %                                 the joints of the given robot model (*read only*).
    %
    %                                 The structure contains following field variables:
    %
    %                                    - ``v``: :math:`(n_{dof} \times 1)` coefficient vector for the *viscous friction*.
    %                                    - ``c``: :math:`(n_{dof} \times 1)` coefficient vector for the *Coulomb friction*.
    %
    %   joint_limits        (struct): Data structure for the *joint positions limits*
    %                                 of the given robot model (*read only*).
    %
    %                                 The structure consists of two limits fields:
    %
    %                                    - ``lwr``: :math:`(n_{dof} \times 1)` vector for the *lower* joint limits.
    %                                    - ``upr``: :math:`(n_{dof} \times 1)` vector for the *upper* joint limits.
    %
    %   robot_model (:class:`~WBM.wbmRobotModel`): Model object with the model parameters of
    %                                              the given floating-base robot (*read only*).
    %
    %   DF_ROBOT_MODEL (char, vector): Default *URDF robot model* for the whole-body interface:
    %                                  ``'icubGazeboSim'`` (*constant*).
    %
    %                                  **Note:** The URDF-file of robot model *icubGazeboSim* is
    %                                  located in the directory of the `yarpWholeBodyInterface`_
    %                                  library.
    %   MAX_NUM_JOINTS  (int, scalar): Maximum number of joints that a robot model can have: 35
    %                                  (*constant*).
    properties(Dependent)
        % public properties for fast get/set methods:
        fixed_link@char
        init_wf_R_b@double matrix
        init_wf_p_b@double vector
        g_wf@double        vector
        ndof@uint16        scalar
        frict_coeff@struct
        joint_limits@struct
        robot_model@WBM.wbmRobotModel
    end

    properties(Constant)
        DF_ROBOT_MODEL = 'icubGazeboSim';
        MAX_NUM_JOINTS = 35;
    end

    properties(Access = protected)
        mwbm_model@WBM.wbmRobotModel
    end

    methods
        function obj = WBMBase(robot_model)
            % Constructor.
            %
            % The constructor initializes the given robot model and sets the
            % world frame (wf) to the initial position and orientation with
            % a specified gravitation.
            %
            % Arguments:
            %   robot_model (:class:`~WBM.wbmRobotModel`): Model object with the model parameters
            %                                              of the given floating-base robot.
            % Returns:
            %   obj: An instance of the :class:`!WBMBase` class.
            if ~exist('robot_model', 'var')
                error('WBMBase::WBMBase: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end

            initWBM(obj, robot_model);
            % set the world frame (WF) to the initial parameters ...
            setInitWorldFrame(obj, robot_model.wf_R_b_init, robot_model.wf_p_b_init, robot_model.g_wf);
        end

        function newObj = copy(obj)
            % Clones the current object.
            %
            % This copy method replaces the ``matlab.mixin.Copyable.copy()``
            % method and creates a *deep copy* of the current object by using
            % directly the memory.
            %
            % Returns:
            %   newObj: An exact copy (clone) of the current :class:`!WBMBase` object.
            newObj = WBM.utilities.copyObj(obj);
        end

        function delete(~)
            % Destructor.
            %
            % Removes all class properties from the workspace (free-up memory).
        end

        function initModel(obj, urdf_model_name)
            % Initializes the whole-body model of the floating-base robot.
            %
            % The method can be called in two different modes:
            %
            % **Normal mode**
            %
            % Arguments:
            %   urdf_model_name (char, vector): String matching *name* of an
            %                                   existing robot model [#f5]_.
            %
            % **Optimized mode** -- *No arguments*.
            %
            %   The MEX whole-body interface loads the configuration files
            %   of the *default robot model* that is specified in the system
            %   environment variable ``YARP_ROBOT_NAME`` [#f5]_.
            if ~exist('urdf_model_name', 'var')
                % optimized mode:
                obj.mwbm_model.urdf_robot_name = getenv('YARP_ROBOT_NAME');
                mexWholeBodyModel('model-initialize');
                return
            end
            % else, use the model name of the robot that is supported by
            % the yarpWholeBodyInterface (the URDF-file must exist in the
            % directory of the yarpWholeBodyInterface library) ...
            obj.mwbm_model.urdf_robot_name = urdf_model_name;
            mexWholeBodyModel('model-initialize', obj.mwbm_model.urdf_robot_name);
        end

        function initModelURDF(obj, urdf_file_name)
            % Initializes the whole-body model of a YARP-based robot with a
            % specific URDF-file.
            %
            % Arguments:
            %   urdf_model_name (char, vector): String with the full path to the
            %                                   URDF-file of the user defined
            %                                   robot model to be read.
            if ~exist('urdf_file_name', 'var')
                error('WBMBase::initModelURDF: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            if ~WBM.utilities.fileExist(urdf_file_name)
                error('WBMBase::initModelURDF: %s', WBM.wbmErrorMsg.FILE_NOT_EXIST);
            end

            obj.mwbm_model.urdf_robot_name = urdf_file_name;
            mexWholeBodyModel('model-initialize-urdf', obj.mwbm_model.urdf_robot_name);
        end

        function setWorldFrame(obj, wf_R_b, wf_p_b, g_wf)
            % Sets the world frame (wf) at a given *position* and *orientation*
            % relative from a specified *fixed link frame* (base reference frame).
            %
            % Arguments:
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf*.
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*.
            %   g_wf   (double, vector): :math:`(3 \times 1)` Cartesian gravity vector
            %                            in the world frame *wf* (*optional*).
            %
            %                            If the gravity is not given, then the default
            %                            gravity vector :math:`[0, 0, \text{-}9.81]^T`
            %                            will be used.
            % Note:
            %   The *base reference frame*, also called *floating-base frame*, is
            %   attached to the *fixed reference link* of the robot. The specified
            %   fixed link (base reference link) of the robot can also be a *contact
            %   constraint link*.
            if (nargin < 3)
                error('WBMBase::setWorldFrame: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            if (nargin == 3)
                % use the default gravity vector ...
                g_wf = obj.mwbm_model.g_wf;
            end

            wf_R_b_arr = reshape(wf_R_b, 9, 1); % reshape the matrix into a 1-column array ...
            mexWholeBodyModel('set-world-frame', wf_R_b_arr, wf_p_b, g_wf);
        end

        function setInitWorldFrame(obj, wf_R_b, wf_p_b, g_wf)
            % Setup the world frame (wf) with the given *initial parameters* or
            % update it with new parameters that are previously changed.
            %
            % The method can be called in two ways:
            %
            %   - .. py:method:: setInitWorldFrame(wf_R_b, wf_p_b[, g_wf])
            %   - .. py:method:: setInitWorldFrame()
            %
            % Arguments:
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf*.
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*.
            %   g_wf   (double, vector): :math:`(3 \times 1)` Cartesian gravity vector
            %                            in the world frame *wf* (*optional*).
            %
            %                            If the gravity is not given, then the default
            %                            gravity vector :math:`[0, 0, \text{-}9.81]^T`
            %                            will be used.
            % Note:
            %   If no parameters are given, then ``setInitWorldFrame`` updates
            %   the world frame *wf* with the new initial parameters from the
            %   model configuration that were changed individually from outside.
            %
            % See Also:
            %   :attr:`~WBM.WBMBase.init_wf_R_b`, :attr:`~WBM.WBMBase.init_wf_p_b`
            %   and :attr:`~WBM.WBMBase.g_wf`.
            switch nargin
                case 4
                    % replace all old initial parameters with the new values ...
                    obj.mwbm_model.wf_R_b_init = wf_R_b;
                    obj.mwbm_model.wf_p_b_init = wf_p_b;
                    obj.mwbm_model.g_wf        = g_wf;
                case 3
                    % replace only the orientation and the translation ...
                    obj.mwbm_model.wf_R_b_init = wf_R_b;
                    obj.mwbm_model.wf_p_b_init = wf_p_b;

                    obj.setWorldFrame(obj.mwbm_model.wf_R_b_init, obj.mwbm_model.wf_p_b_init);
                    return
                case 2
                    error('WBMBase::setInitWorldFrame: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            % if nargin = 1: update the world frame with new initial parameters
            %                (changed individually from outside).

            % set the world frame with the (new) initial parameters:
            setWorldFrame(obj, obj.mwbm_model.wf_R_b_init, obj.mwbm_model.wf_p_b_init, obj.mwbm_model.g_wf);
        end

        function [wf_p_b, wf_R_b] = getWorldFrameFromFixLnk(obj, urdf_fixed_link, q_j)
            % Computes the *position* and *orientation* of the floating base
            % w.r.t. a world frame (wf) that is intentionally set at a specified
            % *fixed link frame* (base reference frame).
            %
            % The *base reference frame*, also called *floating-base frame*, is
            % attached to the *fixed reference link* of the robot. Since the most
            % humanoid robots and other legged robots are not physically attached
            % to the world, the *floating-base framework* provides a more general
            % representation for the robot control. The returned position and
            % orientation of the floating base is obtained from the *forward
            % kinematics* w.r.t. the specified fixed link frame.
            %
            % The method can be called by one of the given modes:
            %
            % **Normal mode** -- Computes the base frame for a specific joint
            % configuration:
            %
            %   .. py:method:: getWorldFrameFromFixLnk(urdf_fixed_link, q_j)
            %
            % **Optimized mode** -- Computes the base frame with the current
            % joint configuration:
            %
            %   .. py:method:: getWorldFrameFromFixLnk(urdf_fixed_link)
            %
            % Arguments:
            %   urdf_fixed_link (char, vector): String matching *URDF name* of the
            %                                   fixed link as reference link.
            %   q_j           (double, vector): (:math:`n_{dof}` x 1) joint positions
            %                                   vector in :math:`[\si{\radian}]`
            %                                   (*optional*).
            % Returns:
            %   [wf_p_b, wf_R_b]: 2-element tuple containing:
            %
            %      - **wf_p_b** (*double, vector*) -- :math:`(3 \times 1)` position vector
            %        from the base frame *b* to the world frame *wf*.
            %      - **wf_R_b** (*double, matrix*) -- :math:`(3 \times 3)` rotation matrix
            %        (orientation) from the base frame *b* to the world frame *wf*.
            %
            % Note:
            %   The specified fixed link (base reference link) can also be a
            %   *contact constraint link*.
            switch nargin
                case 3
                    % normal mode:
                    [wf_p_b, wf_R_b] = computeNewWorld2Base(obj, urdf_fixed_link, q_j);
                case 2
                    % optimized mode:
                    [wf_p_b, wf_R_b] = computeNewWorld2Base(obj, urdf_fixed_link);
                otherwise
                    error('WBMBase::getWorldFrameFromFixLnk: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function [wf_p_b, wf_R_b] = getWorldFrameFromDfltFixLnk(obj, q_j)
            % Computes the *position* and *orientation* of the floating base w.r.t.
            % a world frame (wf) that is set to the default *fixed reference link
            % frame*.
            %
            % The method can be called by one of the given modes:
            %
            % **Normal mode** -- Computes the base frame for a specific joint
            % configuration:
            %
            %   .. py:method:: getWorldFrameFromFixLnk(q_j)
            %
            % **Optimized mode** -- Computes the base frame with the current
            % joint configuration:
            %
            %   .. py:method:: getWorldFrameFromFixLnk()
            %
            % Arguments:
            %   q_j (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                         vector in :math:`[\si{\radian}]` (*optional*).
            % Returns:
            %   [wf_p_b, wf_R_b]: 2-element tuple containing:
            %
            %      - **wf_p_b** (*double, vector*) -- :math:`(3 \times 1)` position vector
            %        from the base frame *b* to the world frame *wf*.
            %      - **wf_R_b** (*double, matrix*) -- :math:`(3 \times 3)` rotation matrix
            %        (orientation) from the base frame *b* to the world frame *wf*.
            if (nargin == 2)
                % normal mode:
                [wf_p_b, wf_R_b] = computeNewWorld2Base(obj, obj.mwbm_model.urdf_fixed_link, q_j);
                return
            end
            % else, optimized mode:
            [wf_p_b, wf_R_b] = computeNewWorld2Base(obj, obj.mwbm_model.urdf_fixed_link);
        end

        function setState(~, q_j, dq_j, v_b)
            % Updates or sets the *state* of the robot model, i.e. the *joint angles*,
            % *joint velocities* and the *generalized base velocity* of the floating base.
            %
            % Arguments:
            %   q_j  (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                          vector in :math:`[\si{\radian}]`.
            %   dq_j (double, vector): :math:`(n_{dof} \times 1)` joint velocities
            %                          vector in :math:`[\si{\radian/s}]`.
            %   v_b  (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                          vector (Cartesian and rotational velocity of
            %                          the base).
            if (nargin ~= 4)
                error('WBMBase::setState: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            mexWholeBodyModel('update-state', q_j, dq_j, v_b);
        end

        function [vqT_b, q_j, v_b, dq_j] = getState(~)
            % Obtains the currently stored *state* of the robot system, in
            % particular
            %
            %    - the *base VQ-transformation*,
            %    - the *joint angles* and *velocities*  and
            %    - the *generalized base velocity*.
            %
            % Returns:
            %   [vqT_b, q_j, v_b, dq_j]: 4-element tuple containing:
            %
            %      - **vqT_b** (*double, vector*) -- :math:`(7 \times 1)` VQ-transformation
            %        frame relative from the base *b* to the world frame *wf* [#f9]_.
            %      - **q_j**   (*double, vector*) -- :math:`(n_{dof} \times 1)` joint angle
            %        vector in :math:`[\si{\radian}]`.
            %      - **v_b**   (*double, vector*) -- :math:`(6 \times 1)` generalized base
            %        velocity vector.
            %      - **dq_j**  (*double, vector*) -- :math:`(n_{dof} \times 1)` joint angle
            %        velocity vector in :math:`[\si{\radian/s}]`.
            [vqT_b, q_j, v_b, dq_j] = mexWholeBodyModel('get-state');
        end

        function stFltb = getFloatingBaseState(~)
            % Returns the current *state*, i.e. the position, orientation and
            % velocity of the robot's floating base *b*.
            %
            % Returns:
            %   stFltb (:class:`~WBM.wbmFltgBaseState`): Floating-base state object with
            %                                            the current state parameters of
            %                                            the robot's floating base *b*
            %                                            related to the world frame *wf*.
            % Note:
            %   The values of the floating-base state are derived from the currently
            %   stored state of the robot system.
            %
            % See Also:
            %   :meth:`WBMBase.getState`.
            stFltb = WBM.wbmFltgBaseState;
            [R_b, p_b, v_b] = mexWholeBodyModel('get-base-state');

            stFltb.wf_R_b = R_b; % orientation of the base (in axis-angle representation)
            stFltb.wf_p_b = p_b; % cartesian position of the base
            stFltb.v_b    = v_b; % cartesian velocity and the rotational velocity of the base
        end

        function wf_H_lnk = transformationMatrix(obj, varargin)
            % Computes the *homogeneous transformation matrix* :math:`H` of a
            % specified link of the robot w.r.t. the current joint configuration
            % :math:`q_j`.
            %
            % The method can be called by one of the given modes:
            %
            % **Normal mode** -- Computes the transformation matrix of the given
            % link, specified by the base orientation and the positions:
            %
            %   .. py:method:: transformationMatrix(wf_R_b, wf_p_b, q_j[, urdf_link_name])
            %
            % **Optimized mode** -- Computes the transformation matrix of the
            % given link at the current state of the robot system:
            %
            %   .. py:method:: transformationMatrix([urdf_link_name])
            %
            % Arguments:
            %   wf_R_b       (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                                  (orientation) from the base frame *b*
            %                                  to world frame *wf*.
            %   wf_p_b       (double, vector): :math:`(3 \times 1)` position vector
            %                                  from the the base frame *b* to the
            %                                  world frame *wf*.
            %   q_j          (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                                  vector in :math:`[\si{\radian}]`.
            %   urdf_link_name (char, vector): String matching *URDF name* of the link (*optional*).
            %
            %                                  **Note:** If the link is not given, then the method
            %                                  uses as default the *fixed link* of the robot.
            % Returns:
            %   wf_H_lnk (double, matrix): :math:`(4 \times 4)` homogeneous transformation
            %   matrix relative from the link frame *lnk* to the world frame *wf*.

            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            switch nargin
                case 5 % normal modes:
                    urdf_link_name = varargin{1,4};
                case 4
                    % use the default link frame (reference frame) ...
                    urdf_link_name = obj.mwbm_model.urdf_fixed_link;
                case 2 % optimized modes:
                    % urdf_link_name = varargin{1}
                    wf_H_lnk = mexWholeBodyModel('transformation-matrix', varargin{1,1});
                    return
                case 1
                    wf_H_lnk = mexWholeBodyModel('transformation-matrix', obj.mwbm_model.urdf_fixed_link);
                    return
                otherwise
                    error('WBMBase::transformationMatrix: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            wf_H_lnk = mexWholeBodyModel('transformation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, urdf_link_name);
        end

        function M = massMatrix(~, wf_R_b, wf_p_b, q_j)
            % Computes the *generalized mass matrix* :math:`M` of the floating-base
            % robot w.r.t. the given joint configuration :math:`q_j`.
            %
            % The method can be called in two different modes:
            %
            % **Normal mode** -- Computes the mass matrix, specified by the
            % base orientation and the positions:
            %
            % Arguments:
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf*.
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*.
            %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                            vector in :math:`[\si{\radian}]`.
            %
            % **Optimized mode** -- *No arguments* (use the current state of the
            % robot system).
            %
            % Returns:
            %   M (double, matrix): :math:`(n \times n)` generalized mass matrix
            %   of the robot, where :math:`n = n_{dof} + 6`.
            switch nargin
                case 4
                    % normal mode:
                    wf_R_b_arr = reshape(wf_R_b, 9, 1);
                    M = mexWholeBodyModel('mass-matrix', wf_R_b_arr, wf_p_b, q_j);
                case 1
                    % optimized mode:
                    M = mexWholeBodyModel('mass-matrix');
                otherwise
                    error('WBMBase::massMatrix: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function [jlmts_lwr, jlmts_upr] = getJointLimits(~)
            % Obtains the lower and upper *joint position limits* of the given
            % robot model.
            %
            % Returns:
            %   [jlmts_lwr, jlmts_upr]: 2-element tuple containing:
            %      - **jlmts_lwr** (*double, vector*) -- :math:`(n_{dof} \times 1)` lower
            %        joint limits vector.
            %      - **jlmts_upr** (*double, vector*) -- :math:`(n_{dof} \times 1)` upper
            %        joint limits vector.

            % get the lower (min.) and upper (max.) joint limits ...
            [jlmts_lwr, jlmts_upr] = mexWholeBodyModel('joint-limits');
        end

        function resv = isJointLimit(obj, q_j)
            % Verifies, if the given joint configuration exceeds the upper or
            % lower joint limits of the given robot.
            %
            % Returns:
            %   resv (logical, vector): :math:`(n_{dof} \times 1)` vector with the
            %   logical results to determine if the current joint configuration is
            %   within the given limits of the robot. Each value is set to *true*
            %   if the corresponding joint has exceeded its limit, *false* otherwise.
            resv = ~((q_j > obj.mwbm_model.jlmts.lwr) & (q_j < obj.mwbm_model.jlmts.upr));
        end

        function dv_b = generalizedBaseAcc(obj, M, c_qv, ddq_j)
            % Computes the *generalized floating-base acceleration* for a
            % *hybrid-dynamic system*.
            %
            % This method may useful for example in inverse dynamics, when the
            % base acceleration is unknown.
            %
            % Arguments:
            %   M     (double, matrix): :math:`(n \times n)` generalized mass matrix
            %                           with :math:`n = n_{dof} + 6`.
            %   c_qv  (double, vector): :math:`(n \times 1)` generalized bias force
            %                           vector with :math:`n = n_{dof} + 6`.
            %   ddq_j (double, vector): :math:`(n_{dof} \times 1)` joint angle acceleration
            %                           vector in :math:`[\si{\radian/{s^2}}]`.
            % Returns:
            %   dv_b (double, vector): :math:`(6 \times 1)` generalized base acceleration
            %   vector of the floating base *b*.
            %
            % See Also:
            %   :func:`WBM.utilities.mbd.generalizedBaseAcc`.
            dv_b = WBM.utilities.mbd.generalizedBaseAcc(M, c_qv, ddq_j, obj.mwbm_model.ndof);
        end

        tau_j = inverseDynamics(obj, varargin)

        function wf_J_lnk = jacobian(obj, varargin)
            % Computes the *Jacobian* of a specified link of the robot w.r.t. the
            % current joint configuration :math:`q_j`.
            %
            % The method can be called by one of the given modes:
            %
            % **Normal mode** -- Computes the Jacobian matrix of the given link,
            % specified by the base orientation and the positions:
            %
            %   .. py:method:: jacobian(wf_R_b, wf_p_b, q_j[, urdf_link_name])
            %
            % **Optimized mode** -- Computes the Jacobian matrix of the given
            % link at the current state of the robot system:
            %
            %   .. py:method:: jacobian([urdf_link_name])
            %
            % Arguments:
            %   wf_R_b       (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                                  (orientation) from the base frame *b*
            %                                  to world frame *wf*.
            %   wf_p_b       (double, vector): :math:`(3 \times 1)` position vector
            %                                  from the the base frame *b* to the
            %                                  world frame *wf*.
            %   q_j          (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                                  vector in :math:`[\si{\radian}]`.
            %   urdf_link_name (char, vector): String matching *URDF name* of the link (*optional*).
            %
            %                                  **Note:** If the link is not given, then the method
            %                                  uses as default the *fixed link* of the robot.
            % Returns:
            %   wf_J_lnk (double, matrix): :math:`(6 \times n)` Jacobian matrix relative from
            %   the link frame *lnk* to the world frame *wf* with :math:`n = n_{dof} + 6`.

            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            switch nargin
                case 5 % normal modes:
                    urdf_link_name = varargin{1,4};
                case 4
                    % use the default link frame (reference frame) ...
                    urdf_link_name = obj.mwbm_model.urdf_fixed_link;
                case 2 % optimized modes:
                    % urdf_link_name = varargin{1}
                    wf_J_lnk = mexWholeBodyModel('jacobian', varargin{1,1});
                    return
                case 1
                    wf_J_lnk = mexWholeBodyModel('jacobian', obj.mwbm_model.urdf_fixed_link);
                    return
                otherwise
                    error('WBMBase::jacobian: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            wf_J_lnk = mexWholeBodyModel('jacobian', wf_R_b_arr, varargin{1,2}, varargin{1,3}, urdf_link_name);
        end

        function djdq_lnk = dJdq(obj, varargin)
            % Computes the *bias acceleration*, i.e. the *product* of the time
            % derivative of the Jacobian and the joint velocities w.r.t. the
            % current state and the specified link of the robot.
            %
            % Note:
            %   The bias acceleration :math:`\dot{J}\dot{q}_j` is an acceleration
            %   that is not due to a robot acceleration.
            %
            % This method is useful for the *Operational Space Control* and for
            % the calculation of the *external contact constraints*. It can be
            % called by one of the given modes:
            %
            % **Normal mode** -- Computes the bias acceleration of the given link,
            % specified by the base orientation, the positions and the velocities:
            %
            %   .. py:method:: dJdq(wf_R_b, wf_p_b, q_j, dq_j, v_b[, urdf_link_name])
            %
            % **Optimized mode** -- Computes the bias acceleration of the given
            % link at the current state of the robot system:
            %
            %   .. py:method:: dJdq([urdf_link_name])
            %
            % Arguments:
            %   wf_R_b       (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                                  (orientation) from the base frame *b*
            %                                  to world frame *wf*.
            %   wf_p_b       (double, vector): :math:`(3 \times 1)` position vector
            %                                  from the the base frame *b* to the
            %                                  world frame *wf*.
            %   q_j          (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                                  vector in :math:`[\si{\radian}]`.
            %   dq_j         (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                                  vector in :math:`[\si{\radian/s}]`.
            %   v_b          (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                                  vector (Cartesian and rotational velocity of
            %                                  the base).
            %   urdf_link_name (char, vector): String matching *URDF name* of the link (*optional*).
            %
            %                                  **Note:** If the link is not given, then the method
            %                                  uses as default the *fixed link* of the robot.
            % Returns:
            %   djdq_lnk (double, vector): :math:`(6 \times 1)` bias acceleration vector relative
            %   from the link frame *lnk* to the world frame *wf* in operational space.

            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % dq_j   = varargin{4}
            % v_b    = varargin{5}
            switch nargin
                case 7 % normal modes:
                    urdf_link_name = varargin{1,6};
                case 6
                    % default link name (reference link) ...
                    urdf_link_name = obj.mwbm_model.urdf_fixed_link;
                case 2 % optimized modes:
                    % urdf_link_name = varargin{1}
                    djdq_lnk = mexWholeBodyModel('dJdq', varargin{1,1});
                    return
                case 1
                    djdq_lnk = mexWholeBodyModel('dJdq', obj.mwbm_model.urdf_fixed_link);
                    return
                otherwise
                    error('WBMBase::dJdq: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            djdq_lnk = mexWholeBodyModel('dJdq', wf_R_b_arr, varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5}, urdf_link_name);
        end

        function h_c = centroidalMomentum(~, wf_R_b, wf_p_b, q_j, dq_j, v_b)
            % Computes the *centroidal momentum* :math:`h_c` of a robot system.
            %
            % The centroidal momentum of a humanoid robot is the sum of all
            % link moments projected to the center of mass (CoM) of the robot.
            %
            % The method is a function of the state variables :math:`q_j, \dot{q}_j`
            % and the generalized base velocity :math:`v_b` and can be called by
            % one of the given modes:
            %
            % **Normal mode** -- Computes the centroidal momentum specified by
            % the base orientation, the positions and the velocities:
            %
            % Arguments:
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf*.
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*.
            %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                            vector in :math:`[\si{\radian}]`.
            %   dq_j   (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                            vector in :math:`[\si{\radian/s}]`.
            %   v_b    (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                            vector (Cartesian and rotational velocity of
            %                            the base).
            %
            % **Optimized mode** -- *No arguments* (use the current state of the robot system).
            %
            % Returns:
            %   h_c (double, vector): :math:`(6 \times 1)` centroidal moment vector
            %   of the robot.
            switch nargin
                case 6
                    % normal mode:
                    wf_R_b_arr = reshape(wf_R_b, 9, 1);
                    h_c = mexWholeBodyModel('centroidal-momentum', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                case 1
                    % optimized mode:
                    h_c = mexWholeBodyModel('centroidal-momentum');
                otherwise
                    error('WBMBase::centroidalMomentum: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function vqT_lnk = forwardKinematics(obj, varargin)
            % Computes the *forward kinematic transformation* of a specific link
            % w.r.t. the current joint configuration :math:`q_j`.
            %
            % The method can be called by one of the given modes:
            %
            % **Normal mode** -- Computes the forward kinematic transformation of
            % the given link, specified by the base orientation and the positions:
            %
            %   .. py:method:: forwardKinematics(wf_R_b, wf_p_b, q_j[, urdf_link_name])
            %
            % **Optimized mode** -- Computes the forward kinematic transformation
            % of the given link at the current state of the robot system:
            %
            %   .. py:method:: forwardKinematics([urdf_link_name])
            %
            % Arguments:
            %   wf_R_b       (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                                  (orientation) from the base frame *b*
            %                                  to world frame *wf*.
            %   wf_p_b       (double, vector): :math:`(3 \times 1)` position vector
            %                                  from the the base frame *b* to the
            %                                  world frame *wf*.
            %   q_j          (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                                  vector in :math:`[\si{\radian}]`.
            %   urdf_link_name (char, vector): String matching *URDF name* of the link
            %                                  (*optional*).
            %
            % Returns:
            %   vqT_lnk (double, vector): :math:`(7 \times 1)` VQ-transformation frame
            %   relative from the link frame *lnk* to the world frame *wf* [#f9]_.

            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            switch nargin
                case 5 % normal modes:
                    urdf_link_name = varargin{1,4};
                case 4
                    % default link name (reference link) ...
                    urdf_link_name = obj.mwbm_model.urdf_fixed_link;
                case 2 % optimized modes:
                    % urdf_link_name = varargin{1}
                    vqT_lnk = mexWholeBodyModel('forward-kinematics', varargin{1,1});
                    return
                case 1
                    vqT_lnk = mexWholeBodyModel('forward-kinematics', obj.mwbm_model.urdf_fixed_link);
                    return
                otherwise
                    error('WBMBase::forwardKinematics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            vqT_lnk = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, varargin{1,2}, varargin{1,3}, urdf_link_name);
        end

        function c_qv = generalizedBiasForces(~, wf_R_b, wf_p_b, q_j, dq_j, v_b)
            % Computes the *generalized bias forces* :math:`C(q_j,\dot{q}_j)` in
            % the dynamics of rigid-body systems.
            %
            % It accounts the *Coriolis*, *centrifugal* and *gravity forces*
            % that are depending on the joint angles :math:`q_j`, the joint
            % velocities :math:`\dot{q}_j` and the generalized base velocity
            % :math:`v_b` (:cite:`Featherstone2008`, p. 40).
            %
            % The method can be called in two different modes:
            %
            % **Normal mode** -- Computes the generalized bias forces, specified
            % by the base orientation, the positions and the velocities:
            %
            % Arguments:
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf*.
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*.
            %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                            vector in :math:`[\si{\radian}]`.
            %   dq_j   (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                            vector in :math:`[\si{\radian/s}]`.
            %   v_b    (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                            vector (Cartesian and rotational velocity of
            %                            the base).
            %
            % **Optimized mode** -- *No arguments* (use the current state of the robot system).
            %
            % Returns:
            %   c_qv (double, vector): :math:`(n \times 1)` generalized bias force
            %   vector of the robot with :math:`n = n_{dof} + 6`.

            % References:
            %   [Fea08] Featherstone, Roy: Rigid Body Dynamics Algorithms. Springer, 2008, p. 40.
            switch nargin
                case 6
                    % normal mode:
                    wf_R_b_arr = reshape(wf_R_b, 9, 1);
                    c_qv = mexWholeBodyModel('generalized-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                case 1
                    % optimized mode:
                    c_qv = mexWholeBodyModel('generalized-forces');
                otherwise
                    error('WBMBase::generalizedBiasForces: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function tau_gen = generalizedForces(~, varargin)
            % Computes the *generalized forces* :math:`\tau_{gen}` in dependency
            % of some given external forces that are acting on the robot system,
            % such that
            %
            %   .. math::
            %      :label: generalized_forces
            %
            %      \tau_{gen} = C(q_j,\dot{q}_j) - J_{e}^{T} f_e\:,
            %
            % where :math:`C(q_j,\dot{q}_j)` denotes the *generalized bias forces*
            % and :math:`J_{e}^{T} f_e` the *external forces* in joint space
            % (:cite:`Featherstone2008`, p. 40).
            %
            % The method can be called in two different modes:
            %
            % **Normal mode** -- Computes the generalized forces, specified by
            % the base orientation, the positions, velocities and the *external
            % forces* in joint space:
            %
            %   .. py:method:: generalizedForces(wf_R_b, wf_p_b, q_j, dq_j, v_b, Je_t, f_e)
            %
            % **Optimized mode** -- Computes the generalized forces at the current
            % state of the robot system, in dependency of the given external force
            % vector :math:`f_e` and the Jacobian :math:`J_{e}^{T}`:
            %
            %   .. py:method:: generalizedForces(Je_t, f_e)
            %
            % Arguments:
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf*.
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*.
            %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                            vector in :math:`[\si{\radian}]`.
            %   dq_j   (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                            vector in :math:`[\si{\radian/s}]`.
            %   v_b    (double, vector): :math:`(6 \times 1)` generalized base
            %                            velocity vector.
            %   Je_t   (double, matrix): :math:`(n \times 6)` transposed Jacobian
            %                            to transform the external forces into the
            %                            joint space with :math:`n = n_{dof} + 6`.
            %   f_e    (double, vector): :math:`(6 \times 1)` external force vector
            %                            that is acting on the robot system.
            % Returns:
            %   tau_gen (double, vector): :math:`(n \times 1)` generalized force
            %   vector of the robot with :math:`n = n_{dof} + 6`.
            %
            % See Also:
            %   :meth:`WBMBase.generalizedBiasForces`.

            % References:
            %   [Fea08] Featherstone, Roy: Rigid Body Dynamics Algorithms. Springer, 2008, p. 40.

            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % dq_j   = varargin{4}
            % v_b    = varargin{5}
            switch nargin
                case 8 % normal mode:
                    Je_t = varargin{1,6};
                    f_e  = varargin{1,7};

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    c_qv = mexWholeBodyModel('generalized-forces', wf_R_b_arr, varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5});
                case 3 % optimized mode:
                    Je_t = varargin{1,1};
                    f_e  = varargin{1,2};

                    c_qv = mexWholeBodyModel('generalized-forces');
                otherwise
                    error('WBMBase::generalizedForces: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            tau_gen = c_qv - Je_t*f_e;
        end

        function c_qv = coriolisBiasForces(~, wf_R_b, wf_p_b, q_j, dq_j, v_b)
            % Computes the *bias Coriolis* and *centrifugal forces* :math:`C(q_j,\dot{q}_j)`
            % in the dynamics of rigid-body systems.
            %
            % The method can be called in two different modes:
            %
            % **Normal mode** -- Computes the bias Coriolis and centrifugal forces,
            % specified by the base orientation, the positions and the velocities:
            %
            % Arguments:
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf*.
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*.
            %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                            vector in :math:`[\si{\radian}]`.
            %   dq_j   (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                            vector in :math:`[\si{\radian/s}]`.
            %   v_b    (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                            vector (Cartesian and rotational velocity of
            %                            the base).
            %
            % **Optimized mode** -- *No arguments* (use the current state of the robot system).
            %
            % Returns:
            %   c_qv (double, vector): :math:`(n \times 1)` bias Coriolis and centrifugal
            %   force vector of the robot with :math:`n = n_{dof} + 6`.
            %
            % Note:
            %   This method is similar to the generalized bias force method and depends
            %   on the same state parameter triplet :math:`[q_j, \dot{q}_j, v_b]`.
            %
            % See Also:
            %   :meth:`WBMBase.generalizedBiasForces`.
            switch nargin
                case 6
                    % normal mode:
                    wf_R_b_arr = reshape(wf_R_b, 9, 1);
                    c_qv = mexWholeBodyModel('coriolis-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                case 1
                    % optimized mode:
                    c_qv = mexWholeBodyModel('coriolis-forces');
                otherwise
                    error('WBMBase::coriolisBiasForces: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function g_q = gravityBiasForces(~, wf_R_b, wf_p_b, q_j)
            % Computes the *gravity bias forces* :math:`G(q_j)` in the dynamics of
            % rigid-body systems.
            %
            % The method can be called in two different modes:
            %
            % **Normal mode** -- Computes the gravity bias forces, specified by
            % the base orientation and the positions:
            %
            % Arguments:
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf*.
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*.
            %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                            vector in :math:`[\si{\radian}]`.
            %
            % **Optimized mode** -- *No arguments* (use the current state of the robot system).
            %
            % Returns:
            %   g_q (double, vector): :math:`(n \times 1)` gravity bias force vector
            %   of the robot with :math:`n = n_{dof} + 6`.
            %
            % Note:
            %   This method is similar to the generalized bias force method, but
            %   depends only on the given joint configuration :math:`q_j`.
            %
            % See Also:
            %   :meth:`WBMBase.generalizedBiasForces`.
            switch nargin
                case 4
                    % normal mode:
                    wf_R_b_arr = reshape(wf_R_b, 9, 1);
                    g_q = mexWholeBodyModel('gravity-forces', wf_R_b_arr, wf_p_b, q_j);
                case 1
                    % optimized mode:
                    g_q = mexWholeBodyModel('gravity-forces');
                otherwise
                    error('WBMBase::gravityForces: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function tau_fr = frictionForces(obj, dq_j)
            % Computes the *frictional torque-forces* :math:`F(\dot{q}_j)` by
            % using a simplified model.
            %
            % The method depends on the current *joint angle acceleration*
            % :math:`\dot{q}_j` and on the given *friction coefficients* of
            % the joints. It can be called in two different modes:
            %
            % **Normal mode** -- Computes the frictional torque-forces in
            % dependency of the current joint velocities:
            %
            % Arguments:
            %   dq_j (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                          vector in :math:`[\si{\radian/s}]`.
            %
            % **Optimized mode** -- *No arguments* (use the current state of the robot system).
            %
            % Returns:
            %   tau_fr (double, vector): :math:`(n_{dof} \times 1)` frictional torque-force
            %   vector with negated values.
            %
            % Note:
            %   The friction coefficients of the joints must be predefined in
            %   :attr:`wbmRobotModel.frict_coeff`, otherwise the whole-body model
            %   assumes that the robot system is *frictionless*.
            %
            % References:
            %   - :cite:`Sciavicco2000`, p. 133 and p. 141.
            %   - :cite:`Craig2005`, pp. 188-189, eq. (6.110)-(6.112).
            %   - :cite:`Corke2017`, p. 252, eq. (9.1) and (9.2).

            % References:
            %   [SS00]  Sciavicco L.; Siciliano, B.:
            %           Modelling and Control of Robot Manipulators.
            %           2nd Edition, Springer, 2000, p. 133 and p. 141.
            %   [Cra05] Craig, John J.: Introduction to Robotics: Mechanics and Control.
            %           3rd Edition, Pearson/Prentice Hall, 2005,
            %           pp. 188-189, eq. (6.110)-(6.112).
            %   [Cor17] Corke, Peter I.: Robotics, Vision & Control: Fundamental Algorithms in Matlab.
            %           2nd Edition, Springer, 2017, p. 252, eq. (9.1) and (9.2).
            if (nargin == 1)
                [~,~,~,dq_j] = mexWholeBodyModel('get-state');
            end
            epsilon = 1e-12; % min. value to treat a number as zero ...

            if (sum(dq_j ~= 0) <= epsilon) % if dq_j = 0
                tau_fr = zeros(obj.mwbm_model.ndof,1);
                return
            end
            tau_cf = -obj.mwbm_model.frict_coeff.c .* sign(dq_j); % Coulomb friction torques
            tau_vf = -obj.mwbm_model.frict_coeff.v .* dq_j;       % viscous friction torques
            tau_fr =  tau_vf + tau_cf;                            % friction torques
        end

        function [M, c_qv, h_c] = wholeBodyDynamics(~, wf_R_b, wf_p_b, q_j, dq_j, v_b)
            % Obtains the main components for the whole-body dynamics of a robot
            % system, in particular it computes w.r.t. the state parameter
            % triplet :math:`[q_j, \dot{q}_j, v_b]`,
            %
            %   - the *generalized mass matrix*,
            %   - the *generalized bias forces*  and
            %   - the *centroidal momentum*
            %
            % of the given floating-base robot.
            %
            % The method can be called in two different modes:
            %
            % **Normal mode** -- Computes the whole-body dynamics components, specified
            % by the base orientation, the positions and the velocities:
            %
            % Arguments:
            %   wf_R_b (double, matrix): :math:`(3 \times 3)` rotation matrix
            %                            (orientation) from the base frame *b*
            %                            to world frame *wf*.
            %   wf_p_b (double, vector): :math:`(3 \times 1)` position vector from
            %                            the base frame *b* to the world frame *wf*.
            %   q_j    (double, vector): :math:`(n_{dof} \times 1)` joint positions
            %                            vector in :math:`[\si{\radian}]`.
            %   dq_j   (double, vector): :math:`(n_{dof} \times 1)` joint angle velocity
            %                            vector in :math:`[\si{\radian/s}]`.
            %   v_b    (double, vector): :math:`(6 \times 1)` generalized base velocity
            %                            vector (Cartesian and rotational velocity of
            %                            the base).
            %
            % **Optimized mode** -- *No arguments* (use the current state of the robot system).
            %
            % Returns:
            %   [M, c_qv, h_c]: 3-element tuple containing:
            %
            %      - **M**    (*double, matrix*) -- :math:`(n \times n)` generalized mass
            %        matrix of the robot,
            %      - **c_qv** (*double, vector*) -- :math:`(n \times 1)` generalized bias
            %        force vector,
            %      - **h_c**  (*double, vector*) -- :math:`(6 \times 1)` centroidal moment
            %        vector of the robot,
            %
            %   with :math:`n = n_{dof} + 6`.
            switch nargin
                case 6
                    % normal mode:
                    wf_R_b_arr = reshape(wf_R_b, 9, 1);

                    M    = mexWholeBodyModel('mass-matrix', wf_R_b_arr, wf_p_b, q_j);
                    c_qv = mexWholeBodyModel('generalized-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                    h_c  = mexWholeBodyModel('centroidal-momentum', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b); % = omega
                case 1
                    % optimized mode:
                    M    = mexWholeBodyModel('mass-matrix');
                    c_qv = mexWholeBodyModel('generalized-forces');
                    h_c  = mexWholeBodyModel('centroidal-momentum'); % = omega
                otherwise
                    error('WBMBase::wholeBodyDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function set.fixed_link(obj, rlnk_name)
            if isempty(rlnk_name)
                error('WBMBase::set.fixed_link: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end
            % update the fixed link (= new default reference link) ...
            obj.mwbm_model.urdf_fixed_link = rlnk_name;
        end

        function rlnk_name = get.fixed_link(obj)
            rlnk_name = obj.mwbm_model.urdf_fixed_link;
        end

        function set.init_wf_R_b(obj, wf_R_b)
            WBM.utilities.chkfun.checkMatDim(wf_R_b, 3, 3, 'WBMBase::set.wf_R_b_init');
            obj.mwbm_model.wf_R_b_init = wf_R_b;
        end

        function wf_R_b = get.init_wf_R_b(obj)
            wf_R_b = obj.mwbm_model.wf_R_b_init;
        end

        function set.init_wf_p_b(obj, wf_p_b)
            WBM.utilities.chkfun.checkCVecDim(wf_p_b, 3, 'WBMBase::set.wf_p_b_init');
            obj.mwbm_model.wf_p_b_init = wf_p_b;
        end

        function wf_p_b = get.init_wf_p_b(obj)
            wf_p_b = obj.mwbm_model.wf_p_b_init;
        end

        function set.g_wf(obj, g_wf)
            WBM.utilities.chkfun.checkCVecDim(new_g, 3, 'WBMBase::set.g_wf');
            obj.mwbm_model.g_wf = g_wf;
        end

        function g_wf = get.g_wf(obj)
            g_wf = obj.mwbm_model.g_wf;
        end

        function set.ndof(obj, ndof)
            if (obj.mwbm_model.ndof > 0)
                error('WBMBase::set.ndof: %s', WBM.wbmErrorMsg.VALUE_IS_INIT);
            end
            obj.mwbm_model.ndof = ndof;

            if isempty(obj.mwbm_model.frict_coeff.v)
                % initialize the friction vectors with zeros (no friction) ...
                obj.mwbm_model.frict_coeff.v = zeros(ndof,1);
                obj.mwbm_model.frict_coeff.c = obj.mwbm_model.frict_coeff.v;
            end
        end

        function ndof = get.ndof(obj)
            ndof = obj.mwbm_model.ndof;
        end

        function set.frict_coeff(obj, frict_coeff)
            if ~isstruct(frict_coeff)
                error('WBMBase::set.frict_coeff: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            n = obj.mwbm_model.ndof;
            WBM.utilities.chkfun.checkCVecDs(frict_coeff.v, frict_coeff.c, n, n, 'WBMBase::set.frict_coeff');

            % update the friction coefficients ...
            obj.mwbm_model.frict_coeff.v = frict_coeff.v;
            obj.mwbm_model.frict_coeff.c = frict_coeff.c;
        end

        function frict_coeff = get.frict_coeff(obj)
            frict_coeff = obj.mwbm_model.frict_coeff;
        end

        function jlmts = get.joint_limits(obj)
            jlmts = obj.mwbm_model.jlmts;
        end

        function robot_model = get.robot_model(obj)
            robot_model = obj.mwbm_model;
        end

        function dispModel(obj, prec)
            % Displays the current model settings of the given floating-base robot.
            %
            % Arguments:
            %   prec (int, scalar): Precision number to specify for the values
            %                       the number of digits after the decimal point
            %                       (default precision: 2) -- *optional*.
            if ~exist('prec', 'var')
                prec = 2;
            end

            [strPath, strName, ext] = fileparts(obj.mwbm_model.urdf_robot_name);
            if ~isempty(ext)
                strURDFname = sprintf([' URDF filename:     %s\n' ...
                                       ' path:              %s'], ...
                                      strcat(strName, ext), strPath);
            else
                strURDFname = sprintf(' URDF robot model:  %s', strName);
            end

            if ~any(obj.mwbm_model.frict_coeff.v,1)
                % if frict_coeff.v = 0:
                strFrictions = '  frictionless';
            else
                strFrictions = sprintf(['  viscous frictions:  %s\n' ...
                                        '  Coulomb frictions:  %s'], ...
                                       mat2str(obj.mwbm_model.frict_coeff.v, prec), ...
                                       mat2str(obj.mwbm_model.frict_coeff.c, prec));
            end

            strParams = sprintf(['WBM Parameters:\n\n' ...
                                 ' robot type:        %s\n' ...
                                 ' #DoFs:             %d\n' ...
                                 '%s\n' ...
                                 ' fixed link to WF:  %s\n' ...
                                 ' R (base to WF):    %s\n' ...
                                 ' p (base to WF):    %s\n' ...
                                 ' g (WF):            %s\n\n' ...
                                 ' joint friction coefficients:\n%s\n'], ...
                                obj.mwbm_model.yarp_robot_type, obj.mwbm_model.ndof, ...
                                strURDFname, obj.mwbm_model.urdf_fixed_link, ...
                                mat2str(obj.mwbm_model.wf_R_b_init, prec), ...
                                mat2str(obj.mwbm_model.wf_p_b_init, prec), ...
                                mat2str(obj.mwbm_model.g_wf, prec), strFrictions);
            fprintf('%s\n', strParams);
        end

    end

    methods(Access = private)
        function initWBM(obj, robot_model)
            % verify input ...
            if ~isa(robot_model, 'WBM.wbmRobotModel')
                error('WBMBase::initWBM: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if (robot_model.ndof > obj.MAX_NUM_JOINTS)
                error('WBMBase::initWBM: %s', WBM.wbmErrorMsg.MAX_NUM_LIMIT);
            end
            if isempty(robot_model.yarp_robot_type)
                error('WBMBase::initWBM: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end

            obj.mwbm_model = WBM.wbmRobotModel;
            obj.mwbm_model.yarp_robot_type = robot_model.yarp_robot_type;
            obj.mwbm_model.ndof            = robot_model.ndof;
            obj.mwbm_model.urdf_fixed_link = robot_model.urdf_fixed_link;

            if (robot_model.ndof > 0)
                if ~isempty(robot_model.frict_coeff.v)
                    WBM.utilities.chkfun.checkCVecDim(robot_model.frict_coeff.v, robot_model.ndof, 'WBMBase::initWBM');

                    obj.mwbm_model.frict_coeff.v = robot_model.frict_coeff.v;
                    if ~isempty(obj.mwbm_model.frict_coeff.c)
                        WBM.utilities.chkfun.checkCVecDim(robot_model.frict_coeff.c, robot_model.ndof, 'WBMBase::initWBM');

                        obj.mwbm_model.frict_coeff.c = robot_model.frict_coeff.c;
                    else
                        % viscous friction only: set the Coulomb friction to 0.
                        obj.mwbm_model.frict_coeff.c = zeros(robot_model.ndof,1);
                    end
                else
                    % frictionless model: set both coefficient vectors to 0.
                    obj.mwbm_model.frict_coeff.v = zeros(robot_model.ndof,1);
                    obj.mwbm_model.frict_coeff.c = obj.mwbm_model.frict_coeff.v;
                end
            end

            % Initialize the mex-WholeBodyModel for a floating-base robot,
            % using Unified Robot Description Format (URDF):
            if isempty(robot_model.urdf_robot_name)
                % optimized mode:
                initModel(obj); % use the default (URDF) model ...
            else
                % normal mode:
                [~,model_name, ext] = fileparts(robot_model.urdf_robot_name);
                if ~isempty(ext)
                    % use directly a specific URDF-file for the robot model ...
                    initModelURDF(obj, robot_model.urdf_robot_name);
                else
                    % set the model name of the robot which is supported by the WB-Toolbox ...
                    initModel(obj, model_name);
                end
            end
            % buffer the joint limits of the robot model for fast access ...
            [obj.mwbm_model.jlmts.lwr, obj.mwbm_model.jlmts.upr] = getJointLimits(obj);
        end

        function [nw_p_b, nw_R_b] = computeNewWorld2Base(obj, urdf_link_name, q_j)
            [ow_vqT_b,~,~,~] = mexWholeBodyModel('get-state'); % VQ-transformation (from base to old world) ...

            % get the homogeneous transformation matrix H (from base to old world):
            [ow_p_b, ow_R_b] = WBM.utilities.tfms.frame2posRotm(ow_vqT_b);
            ow_H_b = WBM.utilities.tfms.posRotm2tform(ow_p_b, ow_R_b);

            % get the VQ-transformation (from reference link to old world):
            if (nargin == 2)
                ow_vqT_rlnk = forwardKinematics(obj, urdf_link_name);
            else
                ow_vqT_rlnk = forwardKinematics(obj, ow_R_b, ow_p_b, q_j, urdf_link_name);
            end

            % compute the new homogeneous transformation matrix H (from base to new world):
            ow_H_rlnk = WBM.utilities.tfms.frame2tform(ow_vqT_rlnk);
            nw_H_b    = ow_H_rlnk \ ow_H_b; % = ow_H_rlnk^(-1) * ow_H_b

            % extract the translation and rotation values (from base to new world) ...
            [nw_p_b, nw_R_b] = WBM.utilities.tfms.tform2posRotm(nw_H_b);
        end

    end
end
