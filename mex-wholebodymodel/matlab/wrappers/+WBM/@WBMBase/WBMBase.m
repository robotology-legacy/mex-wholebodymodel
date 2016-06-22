classdef WBMBase < handle
    properties(Dependent)
        % public properties for fast get/set methods:
        urdfLinkName@char
        wf_R_rootLnk@double matrix
        wf_p_rootLnk@double vector
        g_wf@double         vector
        ndof@uint16         scalar
        joint_limits@struct
        robot_model@WBM.wbmBaseRobotModel
    end

    properties(Constant)
        MAX_NUM_JOINTS = 35;
    end

    properties(Access = protected)
        mwbm_model@WBM.wbmBaseRobotModel
    end

    methods
        % Constructor:
        function obj = WBMBase(robot_model)
            if ~exist('robot_model', 'var')
                error('WBMBase::WBMBase: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            obj.initWBM(robot_model);
            % set the world frame (WF) to the initial parameters ...
            obj.updateWorldFrame(robot_model.wf_R_rootLnk, robot_model.wf_p_rootLnk, ...
                                 robot_model.g_wf);
        end

        % Copy-function:
        function newObj = copy(obj)
            try
                % Matlab-tuning: try to use directly the memory (faster)
                % note: this works only for R2010b or newer.
                objByteArray = getByteStreamFromArray(obj);
                newObj = getArrayFromByteStream(objByteArray);
            catch
                % else, for R2010a and earlier, serialize via a
                % temporary file (slower).
                fname = [tempname '.mat'];
                save(fname, 'obj');
                newObj = load(fname);
                newObj = newObj.obj;
                delete(fname);
            end
        end

        % Destructor:
        function delete(~)
            % remove all class properties from workspace (free-up memory) ...
        end

        function initModel(obj, urdf_model_name)
            if ~exist('urdf_model_name', 'var')
                % Optimized mode: use the default (URDF) robot model which
                % is defined in the environment variable YARP_ROBOT_NAME
                % of the WB-Toolbox ...
                obj.mwbm_model.urdf_robot = getenv('YARP_ROBOT_NAME');
                mexWholeBodyModel('model-initialise');
                return
            end
            % else, use the model name of the robot that is supported by the
            % WB-Toolbox (the URDF-file(s) must exist in the directory of
            % the WB-Toolbox) ...
            obj.mwbm_model.urdf_robot = urdf_model_name;
            mexWholeBodyModel('model-initialise', obj.mwbm_model.urdf_robot);
        end

        function initModelURDF(obj, urdf_file_name)
            if ~exist('urdf_file_name', 'var')
                error('WBMBase::initModelURDF: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            if ~WBM.utilities.fileExist(urdf_file_name)
                error('WBMBase::initModelURDF: %s', WBM.wbmErrorMsg.FILE_NOT_EXIST);
            end

            obj.mwbm_model.urdf_robot = urdf_file_name;
            mexWholeBodyModel('model-initialise-urdf', obj.mwbm_model.urdf_robot);
        end

        function setWorldFrame(obj, wf_R_rootLnk, wf_p_rootLnk, g_wf)
            if (nargin < 3)
                error('WBMBase::setWorldFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            if ~exist('g_wf', 'var')
                % use the default gravity vector ...
                g_wf = obj.mwbm_model.g_wf;
            end
            wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1); % reshape the matrix into a 1-column array ...
            mexWholeBodyModel('set-world-frame', wf_R_rlnk_arr, wf_p_rootLnk, g_wf);
        end

        function updateWorldFrame(obj, wf_R_rootLnk, wf_p_rootLnk, g_wf)
            switch nargin
                case 4
                    % replace all old default parameters with the new values ...
                    obj.mwbm_model.wf_R_rootLnk = wf_R_rootLnk;
                    obj.mwbm_model.wf_p_rootLnk = wf_p_rootLnk;
                    obj.mwbm_model.g_wf         = g_wf;
                case 3
                    % replace only the orientation and translation ...
                    obj.mwbm_model.wf_R_rootLnk = wf_R_rootLnk;
                    obj.mwbm_model.wf_p_rootLnk = wf_p_rootLnk;
                case 2
                    error('WBMBase::updateWorldFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            % update the world frame with the new (or previously changed) parameters ...
            obj.setWorldFrame(obj.mwbm_model.wf_R_rootLnk, obj.mwbm_model.wf_p_rootLnk, ...
                              obj.mwbm_model.g_wf);
        end

        function [w_p_b, w_R_b] = getWorldFrameFromFixedLink(obj, urdf_link_name, q_j)
            % compute the base world frame (WF) from a given contact (constraint) link:
            switch nargin
                case 3
                    % normal mode: compute the WF for a specific joint configuration (positions)
                    [w_p_b, w_R_b] = obj.computeNewWorld2Base(urdf_link_name, q_j);
                case 2
                    % optimized mode: compute the WF with the current joint configuration
                    [w_p_b, w_R_b] = obj.computeNewWorld2Base(urdf_link_name);
                otherwise
                    error('WBMBase::getWorldFrameFromFixedLink: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function [w_p_b, w_R_b] = getWorldFrameFromDfltFixedLink(obj, q_j)
            % compute the base world frame (WF) from the default contact (constraint) link:
            if exist('q_j', 'var')
                % normal mode:
                [w_p_b, w_R_b] = obj.computeNewWorld2Base(obj.mwbm_model.urdf_link_name, q_j);
                return
            end
            % else, optimized mode:
            [w_p_b, w_R_b] = obj.computeNewWorld2Base(obj.mwbm_model.urdf_link_name);
        end

        function setState(~, q_j, dq_j, v_b)
            if (nargin ~= 4)
                error('WBMBase::setState: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            mexWholeBodyModel('update-state', q_j, dq_j, v_b);
        end

        function [vqT_b, q_j, v_b, dq_j] = getState(~)
            [q_j, vqT_b, dq_j, v_b] = mexWholeBodyModel('get-state');
        end

        function stFltb = getFloatingBaseState(~)
            stFltb = WBM.wbmFltgBaseState;
            [R_b, p_b, v_b] = mexWholeBodyModel('get-floating-base-state');

            stFltb.wf_R_b = R_b; % orientation of the base (in axis-angle representation)
            stFltb.wf_p_b = p_b; % cartesian position of the base
            stFltb.wf_v_b = v_b; % cartesian velocity and the rotational velocity of the base
        end

        function wf_H_rlnk = transformationMatrix(obj,varargin)
            % wf_R_rootLnk = varargin{1}
            % wf_p_rootLnk = varargin{2}
            % q_j          = varargin{3}
            switch nargin
                case 5 % normal modes:
                    urdf_link_name = varargin{1,4};
                case 4
                    % use the default link frame ...
                    urdf_link_name = obj.mwbm_model.urdf_link_name;
                case 2 % optimized modes:
                    % urdf_link_name = varargin{1}
                    wf_H_rlnk = mexWholeBodyModel('rototranslation-matrix', varargin{1,1});
                    return
                case 1
                    wf_H_rlnk = mexWholeBodyModel('rototranslation-matrix', obj.mwbm_model.urdf_link_name);
                    return
                otherwise
                    error('WBMBase::transformationMatrix: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            wf_R_rlnk_arr = reshape(varargin{1,1}, 9, 1);
            wf_H_rlnk = mexWholeBodyModel('rototranslation-matrix', wf_R_rlnk_arr, varargin{1,2}, varargin{1,3}, urdf_link_name);
        end

        function M = massMatrix(~, wf_R_rootLnk, wf_p_rootLnk, q_j)
            switch nargin
                case 4
                    % normal mode:
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    M = mexWholeBodyModel('mass-matrix', wf_R_rlnk_arr, wf_p_rootLnk, q_j);
                case 1
                    % optimized mode:
                    M = mexWholeBodyModel('mass-matrix');
                otherwise
                    error('WBMBase::massMatrix: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function [jl_lower, jl_upper] = getJointLimits(~)
            [jl_lower, jl_upper] = mexWholeBodyModel('joint-limits');
        end

        function resv = isJointLimit(obj, q_j)
            resv = ~((q_j > obj.mwbm_model.joint_ll) & (q_j < obj.mwbm_model.joint_ul));
        end

        function tau_gen = inverseDynamics(obj, varargin) % not implemented yet in C++!
            % wf_R_rootLnk = varargin{1}
            % wf_p_rootLnk = varargin{2}
            % q_j          = varargin{3}
            % dq_j         = varargin{4}
            % v_b          = varargin{5}
            % ddq_j        = varargin{6}
            % dv_b         = varargin{7}
            switch nargin
                case 9 % normal modes:
                    urdf_link_name = varargin{1,8};
                case 8
                    % use the default link frame name ...
                    urdf_link_name = obj.mwbm_model.urdf_link_name;
                case 3 % optimized modes:
                    % dq_j           = varargin{1}
                    % urdf_link_name = varargin{2}
                    tau    = mexWholeBodyModel('inverse-dynamics', varargin{1,2});
                    tau_fr = frictionForces(obj, varargin{1,1});
                    tau_fr = vertcat(zeros(6,1), tau_fr);

                    tau_gen = tau + tau_fr;
                    return
                case 2
                    % dq_j = varargin{1}
                    tau    = mexWholeBodyModel('inverse-dynamics', obj.mwbm_model.urdf_link_name);
                    tau_fr = frictionForces(obj, varargin{1,1});
                    tau_fr = vertcat(zeros(6,1), tau_fr);

                    tau_gen = tau + tau_fr;
                    return
            otherwise
                error('WBMBase::inverseDynamics: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            wf_R_rlnk_arr = reshape(varargin{1,1}, 9, 1);
            tau    = mexWholeBodyModel('inverse-dynamics', wf_R_rlnk_arr, varargin{1,2}, varargin{1,3}, varargin{1,4}, ...
                                       varargin{1,5}, varargin{1,6},  varargin{1,7}, urdf_link_name);
            tau_fr = frictionForces(obj, varargin{1,4});
            tau_fr = vertcat(zeros(6,1), tau_fr);

            tau_gen = tau + tau_fr;
        end

        function J = jacobian(obj, varargin)
            % wf_R_rootLnk = varargin{1}
            % wf_p_rootLnk = varargin{2}
            % q_j          = varargin{3}
            switch nargin
                case 5 % normal modes:
                    urdf_link_name = varargin{1,4};
                case 4
                    % use the default link frame ...
                    urdf_link_name = obj.mwbm_model.urdf_link_name;
                case 2 % optimized modes:
                    % urdf_link_name = varargin{1}
                    J = mexWholeBodyModel('jacobian', varargin{1,1});
                    return
                case 1
                    J = mexWholeBodyModel('jacobian', obj.mwbm_model.urdf_link_name);
                    return
                otherwise
                    error('WBMBase::jacobian: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            wf_R_rlnk_arr = reshape(varargin{1,1}, 9, 1);
            J = mexWholeBodyModel('jacobian', wf_R_rlnk_arr, varargin{1,2}, varargin{1,3}, urdf_link_name);
        end

        function dJ = dJdq(obj, varargin)
            % wf_R_rootLnk = varargin{1}
            % wf_p_rootLnk = varargin{2}
            % q_j          = varargin{3}
            % dq_j         = varargin{4}
            % v_b          = varargin{5}
            switch nargin
                case 7 % normal modes:
                    urdf_link_name = varargin{1,6};
                case 6
                    urdf_link_name = obj.mwbm_model.urdf_link_name; % default link name ...
                case 2 % optimized modes:
                    % urdf_link_name = varargin{1}
                    dJ = mexWholeBodyModel('djdq', varargin{1,1});
                    return
                case 1
                    dJ = mexWholeBodyModel('djdq', obj.mwbm_model.urdf_link_name);
                    return
                otherwise
                    error('WBMBase::dJdq: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            wf_R_rlnk_arr = reshape(varargin{1,1}, 9, 1);
            dJ = mexWholeBodyModel('djdq', wf_R_rlnk_arr, varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5}, urdf_link_name);
        end

        function h_c = centroidalMomentum(~, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            switch nargin
                case 6
                    % normal mode:
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    h_c = mexWholeBodyModel('centroidal-momentum', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b);
                case 1
                    % optimized mode:
                    h_c = mexWholeBodyModel('centroidal-momentum');
                otherwise
                    error('WBMBase::centroidalMomentum: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function wf_vqT_rlnk = forwardKinematics(obj, varargin)
            % wf_R_rootLnk = varargin{1}
            % wf_p_rootLnk = varargin{2}
            % q_j          = varargin{3}
            switch nargin
                case 5 % normal modes:
                    urdf_link_name = varargin{1,4};
                case 4
                    urdf_link_name = obj.mwbm_model.urdf_link_name; % default link name ...
                case 2 % optimized modes:
                    % urdf_link_name = varargin{1}
                    wf_vqT_rlnk = mexWholeBodyModel('forward-kinematics', varargin{1,1});
                    return
                case 1
                    wf_vqT_rlnk = mexWholeBodyModel('forward-kinematics', obj.mwbm_model.urdf_link_name);
                    return
                otherwise
                    error('WBMBase::forwardKinematics: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            wf_R_rlnk_arr = reshape(varargin{1,1}, 9, 1);
            wf_vqT_rlnk = mexWholeBodyModel('forward-kinematics', wf_R_rlnk_arr, varargin{1,2}, varargin{1,3}, urdf_link_name);
        end

        function C_qv = generalizedBiasForces(~, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            switch nargin
                case 6
                    % normal mode:
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    C_qv = mexWholeBodyModel('generalised-forces', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b);
                case 1
                    % optimized mode:
                    C_qv = mexWholeBodyModel('generalised-forces');
                otherwise
                    error('WBMBase::generalizedBiasForces: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function tau_c = coriolisCentrifugalForces(~, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            switch nargin
                case 6
                    % normal mode:
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    tau_c = mexWholeBodyModel('coriolis-centrifugal-forces', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b);
                case 1
                    % optimized mode:
                    tau_c = mexWholeBodyModel('coriolis-centrifugal-forces');
                otherwise
                    error('WBMBase::coriolisCentrifugalForces: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function tau_g = gravityForces(~, wf_R_rootLnk, wf_p_rootLnk, q_j)
            switch nargin
                case 4
                    % normal mode:
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    tau_g = mexWholeBodyModel('gravity-forces', wf_R_rlnk_arr, wf_p_rootLnk, q_j);
                case 1
                    % optimized mode:
                    tau_g = mexWholeBodyModel('gravity-forces');
                otherwise
                    error('WBMBase::gravityForces: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function tau_fr = frictionForces(obj, dq_j)
            if ~exist('dq_j', 'var')
                [~,~,~,dq_j] = getState(obj);
            end
            epsilon = 1e-12; % min. value to treat a number as zero ...

            % Compute the friction forces (torques) F(dq) with a simplified model:
            % Further details about the computation are available in:
            %   [1] Modelling and Control of Robot Manipulators, L. Sciavicco & B. Siciliano, 2nd Edition, Springer, 2008,
            %       p. 133 & p. 141.
            %   [2] Introduction to Robotics: Mechanics and Control, John J. Craig, 3rd Edition, Pearson/Prentice Hall, 2005,
            %       pp. 188-189, eq. (6.110)-(6.112).
            %   [3] Robotics, Vision & Control: Fundamental Algorithms in Matlab, Peter I. Corke, Springer, 2011,
            %       pp. 201-202, eq. (9.4) & (9.5).
            if (sum(dq_j ~= 0) <= epsilon) % if dq_j = 0:
                tau_fr = zeros(obj.mwbm_model.ndof,1);
                return
            end
            tau_vf = -obj.mwbm_model.vfrict_coeff .* dq_j;       % viscous friction torques
            tau_cf = -obj.mwbm_model.cfrict_coeff .* sign(dq_j); % Coulomb friction torques
            tau_fr =  tau_vf + tau_cf;                           % friction torques
        end

        function set.urdfLinkName(obj, new_link_name)
            if isempty(new_link_name)
                error('WBMBase::set.urdfLinkName: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end
            % update the default link name ...
            obj.mwbm_model.urdf_link_name = new_link_name;
        end

        function lnk_name = get.urdfLinkName(obj)
            lnk_name = obj.mwbm_model.urdf_link_name;
        end

        function set.wf_R_rootLnk(obj, wf_R_rlnk)
            if ( (size(wf_R_rlnk,1) ~= 3) || (size(wf_R_rlnk,2) ~= 3) )
                error('WBMBase::set.wf_R_rootLnk: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            obj.mwbm_model.wf_R_rootLnk = wf_R_rlnk;
        end

        function wf_R_rlnk = get.wf_R_rootLnk(obj)
            wf_R_rlnk = obj.mwbm_model.wf_R_rootLnk;
        end

        function set.wf_p_rootLnk(obj, wf_p_rlnk)
            if (size(wf_p_rlnk,1) ~= 3)
                error('WBMBase::set.wf_p_rootLnk: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            obj.mwbm_model.wf_p_rootLnk = wf_p_rlnk;
        end

        function wf_p_rlnk = get.wf_p_rootLnk(obj)
            wf_p_rlnk = obj.mwbm_model.wf_p_rootLnk;
        end

        function set.g_wf(obj, g)
            if (size(g,1) ~= 3)
                error('WBMBase::set.g_wf: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            obj.mwbm_model.g_wf = g;
        end

        function g_wf = get.g_wf(obj)
            g_wf = obj.mwbm_model.g_wf;
        end

        function ndof = get.ndof(obj)
            ndof = obj.mwbm_model.ndof;
        end

        function jl = get.joint_limits(obj)
            jl.lower = obj.mwbm_model.joint_ll;
            jl.upper = obj.mwbm_model.joint_ul;
        end

        function robot_model = get.robot_model(obj)
            robot_model = obj.mwbm_model;
        end

        function dispModel(obj, prec)
            if ~exist('prec', 'var')
                prec = 2;
            end

            [strPath, strName, ext] = fileparts(obj.mwbm_model.urdf_robot);
            if ~isempty(ext)
                strURDFname = sprintf([' URDF filename:       %s\n' ...
                                       ' path:                %s'], ...
                                      strcat(strName, ext), strPath);
            else
                strURDFname = sprintf(' URDF robot model:    %s', strName);
            end

            if ~any(obj.mwbm_model.vfrict_coeff,1) % if vfrict_coeff = 0:
                strFrictions = '  frictionless';
            else
                strFrictions = sprintf(['  viscous frictions:  %s\n' ...
                                        '  Coulomb frictions:  %s'], ...
                                       mat2str(obj.mwbm_model.vfrict_coeff, prec), ...
                                       mat2str(obj.mwbm_model.cfrict_coeff, prec));
            end

            strParams = sprintf(['WBM Parameters:\n\n' ...
                                 ' #DoFs:               %d\n' ...
                                 '%s\n' ...
                                 ' URDF ref. link name: %s\n\n' ...
                                 ' R (root link to world frame):  %s\n' ...
                                 ' p (root link to world frame):  %s\n' ...
                                 ' g (world frame):               %s\n\n' ...
                                 ' joint friction coefficients:\n%s\n'], ...
                                obj.mwbm_model.ndof, strURDFname, ...
                                obj.mwbm_model.urdf_link_name, ...
                                mat2str(obj.mwbm_model.wf_R_rootLnk, prec), ...
                                mat2str(obj.mwbm_model.wf_p_rootLnk, prec), ...
                                mat2str(obj.mwbm_model.g_wf, prec), strFrictions);
            disp(strParams);
        end

    end

    methods(Access = private)
        function initWBM(obj, robot_model)
            if ~isa(robot_model, 'WBM.wbmBaseRobotModel')
                error('WBMBase::initWBM: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            % verify ndof ...
            if (robot_model.ndof == 0)
                error('WBMBase::initWBM: %s', WBM.wbmErrorMsg.VALUE_IS_ZERO);
            end
            if (robot_model.ndof > obj.MAX_NUM_JOINTS)
                error('WBMBase::initWBM: %s', WBM.wbmErrorMsg.MAX_NUM_LIMIT);
            end

            obj.mwbm_model = WBM.wbmBaseRobotModel;
            obj.mwbm_model.ndof           = robot_model.ndof;
            obj.mwbm_model.urdf_link_name = robot_model.urdf_link_name;

            if ~isempty(robot_model.vfrict_coeff)
                if (size(robot_model.vfrict_coeff,1) ~= robot_model.ndof)
                    error('WBMBase::initWBM: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                end

                obj.mwbm_model.vfrict_coeff = robot_model.vfrict_coeff;
                if ~isempty(obj.mwbm_model.cfrict_coeff)
                    if (size(robot_model.cfrict_coeff,1) ~= robot_model.ndof)
                        error('WBMBase::initWBM: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                    end

                    obj.mwbm_model.cfrict_coeff = robot_model.cfrict_coeff;
                else
                    % only viscous friction: set the Coulomb friction to 0.
                    obj.mwbm_model.cfrict_coeff = zeros(robot_model.ndof,1);
                end
            else
                % frictionless model: set both coefficient vectors to 0.
                obj.mwbm_model.vfrict_coeff = zeros(robot_model.ndof,1);
                obj.mwbm_model.cfrict_coeff = obj.mwbm_model.vfrict_coeff;
            end

            % Initialize the mex-WholeBodyModel for a floating base robot,
            % using Unified Robot Description Format (URDF):
            if isempty(robot_model.urdf_robot)
                % optimized mode:
                obj.initModel(); % use the default (URDF) model ...
            else
                % normal mode:
                [~,model_name, ext] = fileparts(robot_model.urdf_robot);
                if ~isempty(ext)
                    % use directly a specific URDF-file for the robot model ...
                    obj.initModelURDF(robot_model.urdf_robot);
                else
                    % set the model name of the robot which is supported by the WB-Toolbox ...
                    obj.initModel(model_name);
                end
            end
            % set the joint limits for the robot model ...
            [obj.mwbm_model.joint_ll, obj.mwbm_model.joint_ul] = obj.getJointLimits();
        end

        function [nw_p_b, nw_R_b] = computeNewWorld2Base(obj, urdf_link_name, q_j)
            % get the transformation values from the base to the old world ...
            [ow_vqT_b,~,~,~] = obj.getState();
            % get the homogeneous transformation matrix H
            % from the base to the old world ...
            [ow_p_b, ow_R_b] = WBM.utilities.frame2posRotm(ow_vqT_b);
            ow_H_b = WBM.utilities.posRotm2tform(ow_p_b, ow_R_b);

            % get the transformation values from the reference link to the
            % old world:
            if (nargin == 2)
                ow_vqT_refLnk = obj.forwardKinematics(urdf_link_name);
            else
                ow_vqT_refLnk = obj.forwardKinematics(ow_R_b, ow_p_b, q_j, urdf_link_name);
            end

            % compute the hom. transformation matrix H from the base to
            % the new world:
            ow_H_refLnk = WBM.utilities.frame2tform(ow_vqT_refLnk);
            nw_H_b = ow_H_refLnk \ ow_H_b;

            % extract the translation and the rotation values ...
            [nw_p_b, nw_R_b] = WBM.utilities.tform2posRotm(nw_H_b);
        end

    end
end
