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
% GNU General Public License for more details.
%
% A copy of the GNU Lesser General Public License can be found along
% with the WBML. If not, see <http://www.gnu.org/licenses/>.

classdef WBMBase < handle
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
        % Constructor:
        function obj = WBMBase(robot_model)
            if ~exist('robot_model', 'var')
                error('WBMBase::WBMBase: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end

            initWBM(obj, robot_model);
            % set the world frame (WF) to the initial parameters ...
            setInitWorldFrame(obj, robot_model.wf_R_b_init, robot_model.wf_p_b_init, robot_model.g_wf);
        end

        % Copy function:
        function newObj = copy(obj)
            newObj = WBM.utilities.copyObj(obj);
        end

        % Destructor:
        function delete(~)
            % remove all class properties from workspace (free-up memory) ...
        end

        function initModel(obj, urdf_model_name)
            if ~exist('urdf_model_name', 'var')
                % Optimized mode: use the default (URDF) robot model of the
                % yarpWholeBodyInterface which is defined in the environment
                % variable YARP_ROBOT_NAME.
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
            % setup the world frame with the initial parameters or update it with the new parameters:
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
            % if nargin = 1:
            % update the world frame with the new initial parameters that are individually changed from outside.

            % set the world frame with the (new) initial parameters:
            setWorldFrame(obj, obj.mwbm_model.wf_R_b_init, obj.mwbm_model.wf_p_b_init, obj.mwbm_model.g_wf);
        end

        function [wf_p_b, wf_R_b] = getWorldFrameFromFixLnk(obj, urdf_fixed_link, q_j)
            % compute the position and orientation of the floating base w.r.t. a world frame (WF)
            % that is fixed to a specified link frame (reference frame):
            switch nargin
                case 3
                    % normal mode: compute the base frame for a specific joint configuration (joint positions)
                    [wf_p_b, wf_R_b] = computeNewWorld2Base(obj, urdf_fixed_link, q_j);
                case 2
                    % optimized mode: compute the base frame with the current joint configuration
                    [wf_p_b, wf_R_b] = computeNewWorld2Base(obj, urdf_fixed_link);
                otherwise
                    error('WBMBase::getWorldFrameFromFixLnk: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function [wf_p_b, wf_R_b] = getWorldFrameFromDfltFixLnk(obj, q_j)
            % compute the position and orientation of the floating base w.r.t. a world frame (WF)
            % that is fixed to the default reference link frame:
            if (nargin == 2)
                % normal mode:
                [wf_p_b, wf_R_b] = computeNewWorld2Base(obj, obj.mwbm_model.urdf_fixed_link, q_j);
                return
            end
            % else, optimized mode:
            [wf_p_b, wf_R_b] = computeNewWorld2Base(obj, obj.mwbm_model.urdf_fixed_link);
        end

        function setState(~, q_j, dq_j, v_b)
            if (nargin ~= 4)
                error('WBMBase::setState: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            mexWholeBodyModel('update-state', q_j, dq_j, v_b);
        end

        function [vqT_b, q_j, v_b, dq_j] = getState(~)
            [vqT_b, q_j, v_b, dq_j] = mexWholeBodyModel('get-state');
        end

        function stFltb = getFloatingBaseState(~)
            stFltb = WBM.wbmFltgBaseState;
            [R_b, p_b, v_b] = mexWholeBodyModel('get-base-state');

            stFltb.wf_R_b = R_b; % orientation of the base (in axis-angle representation)
            stFltb.wf_p_b = p_b; % cartesian position of the base
            stFltb.v_b    = v_b; % cartesian velocity and the rotational velocity of the base
        end

        function wf_H_lnk = transformationMatrix(obj, varargin)
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
            % get the lower (min.) and upper (max.) joint limits ...
            [jlmts_lwr, jlmts_upr] = mexWholeBodyModel('joint-limits');
        end

        function resv = isJointLimit(obj, q_j)
            resv = ~((q_j > obj.mwbm_model.jlmts.lwr) & (q_j < obj.mwbm_model.jlmts.upr));
        end

        function dv_b = generalizedBaseAcc(obj, M, c_qv, ddq_j)
            dv_b = WBM.utilities.mbd.generalizedBaseAcc(M, c_qv, ddq_j, obj.mwbm_model.ndof);
        end

        tau_j = inverseDynamics(obj, varargin)

        function wf_J_lnk = jacobian(obj, varargin)
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
            if (nargin == 1)
                [~,~,~,dq_j] = mexWholeBodyModel('get-state');
            end
            epsilon = 1e-12; % min. value to treat a number as zero ...

            % Compute the friction forces (torques) F(dq_j) with a simplified model:
            % Further details about the computation are available in:
            %   [1] Modelling and Control of Robot Manipulators, L. Sciavicco & B. Siciliano, 2nd Edition, Springer, 2008,
            %       p. 133 & p. 141.
            %   [2] Introduction to Robotics: Mechanics and Control, John J. Craig, 3rd Edition, Pearson/Prentice Hall, 2005,
            %       pp. 188-189, eq. (6.110)-(6.112).
            %   [3] Robotics, Vision & Control: Fundamental Algorithms in Matlab, Peter I. Corke, Springer, 2011,
            %       pp. 201-202, eq. (9.4) & (9.5).
            if (sum(dq_j ~= 0) <= epsilon) % if dq_j = 0
                tau_fr = zeros(obj.mwbm_model.ndof,1);
                return
            end
            tau_cf = -obj.mwbm_model.frict_coeff.c .* sign(dq_j); % Coulomb friction torques
            tau_vf = -obj.mwbm_model.frict_coeff.v .* dq_j;       % viscous friction torques
            tau_fr =  tau_vf + tau_cf;                            % friction torques
        end

        function [M, c_qv, h_c] = wholeBodyDynamics(~, wf_R_b, wf_p_b, q_j, dq_j, v_b)
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

            % Initialize the mex-WholeBodyModel for a floating base robot,
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
