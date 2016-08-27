classdef WBMBase < handle
    properties(Dependent)
        % public properties for fast get/set methods:
        fixed_link@char
        wf_R_b@double matrix
        wf_p_b@double vector
        g_wf@double   vector
        ndof@uint16   scalar
        frict_coeff@struct
        joint_limits@struct
        robot_model@WBM.wbmBaseRobotModel
    end

    properties(Constant)
        DFLT_ROBOT_MODEL = 'icubGazeboSim';
        MAX_NUM_JOINTS   = 35;
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
            obj.updateWorldFrame(robot_model.wf_R_b, robot_model.wf_p_b, ...
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
                obj.mwbm_model.urdf_robot_name = getenv('YARP_ROBOT_NAME');
                mexWholeBodyModel('model-initialise');
                return
            end
            % else, use the model name of the robot that is supported by the
            % WB-Toolbox (the URDF-file(s) must exist in the directory of
            % the WB-Toolbox) ...
            obj.mwbm_model.urdf_robot_name = urdf_model_name;
            mexWholeBodyModel('model-initialise', obj.mwbm_model.urdf_robot_name);
        end

        function initModelURDF(obj, urdf_file_name)
            if ~exist('urdf_file_name', 'var')
                error('WBMBase::initModelURDF: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            if ~WBM.utilities.fileExist(urdf_file_name)
                error('WBMBase::initModelURDF: %s', WBM.wbmErrorMsg.FILE_NOT_EXIST);
            end

            obj.mwbm_model.urdf_robot_name = urdf_file_name;
            mexWholeBodyModel('model-initialise-urdf', obj.mwbm_model.urdf_robot_name);
        end

        function setWorldFrame(obj, wf_R_b, wf_p_b, g_wf)
            if (nargin < 3)
                error('WBMBase::setWorldFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            if ~exist('g_wf', 'var')
                % use the default gravity vector ...
                g_wf = obj.mwbm_model.g_wf;
            end
            wf_R_b_arr = reshape(wf_R_b, 9, 1); % reshape the matrix into a 1-column array ...
            mexWholeBodyModel('set-world-frame', wf_R_b_arr, wf_p_b, g_wf);
        end

        function updateWorldFrame(obj, wf_R_b, wf_p_b, g_wf)
            switch nargin
                case 4
                    % replace all old default parameters with the new values ...
                    obj.mwbm_model.wf_R_b = wf_R_b;
                    obj.mwbm_model.wf_p_b = wf_p_b;
                    obj.mwbm_model.g_wf   = g_wf;
                case 3
                    % replace only the orientation and translation ...
                    obj.mwbm_model.wf_R_b = wf_R_b;
                    obj.mwbm_model.wf_p_b = wf_p_b;
                case 2
                    error('WBMBase::updateWorldFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            % update the world frame with the new (or previously changed) parameters ...
            obj.setWorldFrame(obj.mwbm_model.wf_R_b, obj.mwbm_model.wf_p_b, ...
                              obj.mwbm_model.g_wf);
        end

        function [wf_p_b, wf_R_b] = getWorldFrameFromFixedLink(obj, urdf_link_name, q_j)
            % compute the base world frame (WF) from a given contact (constraint) link:
            switch nargin
                case 3
                    % normal mode: compute the WF for a specific joint configuration (positions)
                    [wf_p_b, wf_R_b] = obj.computeNewWorld2Base(urdf_link_name, q_j);
                case 2
                    % optimized mode: compute the WF with the current joint configuration
                    [wf_p_b, wf_R_b] = obj.computeNewWorld2Base(urdf_link_name);
                otherwise
                    error('WBMBase::getWorldFrameFromFixedLink: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function [wf_p_b, wf_R_b] = getWorldFrameFromDfltFixedLink(obj, q_j)
            % compute the base world frame (WF) from the default contact (constraint) link:
            if exist('q_j', 'var')
                % normal mode:
                [wf_p_b, wf_R_b] = obj.computeNewWorld2Base(obj.mwbm_model.urdf_fixed_link, q_j);
                return
            end
            % else, optimized mode:
            [wf_p_b, wf_R_b] = obj.computeNewWorld2Base(obj.mwbm_model.urdf_fixed_link);
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

        function wf_H_lnk = transformationMatrix(obj, varargin)
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            switch nargin
                case 5 % normal modes:
                    urdf_link_name = varargin{1,4};
                case 4
                    % use the default link frame ...
                    urdf_link_name = obj.mwbm_model.urdf_fixed_link;
                case 2 % optimized modes:
                    % urdf_link_name = varargin{1}
                    wf_H_lnk = mexWholeBodyModel('rototranslation-matrix', varargin{1,1});
                    return
                case 1
                    wf_H_lnk = mexWholeBodyModel('rototranslation-matrix', obj.mwbm_model.urdf_fixed_link);
                    return
                otherwise
                    error('WBMBase::transformationMatrix: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            wf_H_lnk = mexWholeBodyModel('rototranslation-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3}, urdf_link_name);
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
                    error('WBMBase::massMatrix: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function [jl_lower, jl_upper] = getJointLimits(~)
            [jl_lower, jl_upper] = mexWholeBodyModel('joint-limits');
        end

        function resv = isJointLimit(obj, q_j)
            resv = ~((q_j > obj.mwbm_model.jlim.lwr) & (q_j < obj.mwbm_model.jlim.upr));
        end

        function tau_ctrl = inverseDynamics(obj, varargin) % not implemented yet in C++!
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % dq_j   = varargin{4}
            % v_b    = varargin{5}
            % ddq_j  = varargin{6}
            % dv_b   = varargin{7}
            switch nargin
                case 8 % normal mode:
                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    tau    = mexWholeBodyModel('inverse-dynamics', wf_R_b_arr, varargin{1,2}, varargin{1,3}, ...
                                               varargin{1,4}, varargin{1,5}, varargin{1,6},  varargin{1,7});
                    tau_fr = frictionForces(obj, varargin{1,4}); % friction torques (negative values)
                    tau_fr = vertcat(zeros(6,1), tau_fr);

                    tau_ctrl = tau + tau_fr;
                case 5 % optimized mode:
                    % dq_j  = varargin{1}
                    % ddq_j = varargin{2}
                    % dv_b  = varargin{3}

                    % Note: The same vector dq_j is already stored inside of the mex-subroutine. Because
                    %       before any function can be used in optimized mode, the function "setState"
                    %       must be called previously to update the state parameters q_j, dq_j and v_b.
                    tau    = mexWholeBodyModel('inverse-dynamics', varargin{1,2}, varargin{1,3});
                    tau_fr = frictionForces(obj, varargin{1,1});
                    tau_fr = vertcat(zeros(6,1), tau_fr);

                    tau_ctrl = tau + tau_fr;
            otherwise
                error('WBMBase::inverseDynamics: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function J = jacobian(obj, varargin)
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            switch nargin
                case 5 % normal modes:
                    urdf_link_name = varargin{1,4};
                case 4
                    % use the default link frame ...
                    urdf_link_name = obj.mwbm_model.urdf_fixed_link;
                case 2 % optimized modes:
                    % urdf_link_name = varargin{1}
                    J = mexWholeBodyModel('jacobian', varargin{1,1});
                    return
                case 1
                    J = mexWholeBodyModel('jacobian', obj.mwbm_model.urdf_fixed_link);
                    return
                otherwise
                    error('WBMBase::jacobian: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            J = mexWholeBodyModel('jacobian', wf_R_b_arr, varargin{1,2}, varargin{1,3}, urdf_link_name);
        end

        function dJ = dJdq(obj, varargin)
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            % dq_j   = varargin{4}
            % v_b    = varargin{5}
            switch nargin
                case 7 % normal modes:
                    urdf_link_name = varargin{1,6};
                case 6
                    urdf_link_name = obj.mwbm_model.urdf_fixed_link; % default link name ...
                case 2 % optimized modes:
                    % urdf_link_name = varargin{1}
                    dJ = mexWholeBodyModel('djdq', varargin{1,1});
                    return
                case 1
                    dJ = mexWholeBodyModel('djdq', obj.mwbm_model.urdf_fixed_link);
                    return
                otherwise
                    error('WBMBase::dJdq: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            dJ = mexWholeBodyModel('djdq', wf_R_b_arr, varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5}, urdf_link_name);
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
                    error('WBMBase::centroidalMomentum: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function wf_vqT_lnk = forwardKinematics(obj, varargin)
            % wf_R_b = varargin{1}
            % wf_p_b = varargin{2}
            % q_j    = varargin{3}
            switch nargin
                case 5 % normal modes:
                    urdf_link_name = varargin{1,4};
                case 4
                    urdf_link_name = obj.mwbm_model.urdf_fixed_link; % default link name ...
                case 2 % optimized modes:
                    % urdf_link_name = varargin{1}
                    wf_vqT_lnk = mexWholeBodyModel('forward-kinematics', varargin{1,1});
                    return
                case 1
                    wf_vqT_lnk = mexWholeBodyModel('forward-kinematics', obj.mwbm_model.urdf_fixed_link);
                    return
                otherwise
                    error('WBMBase::forwardKinematics: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            wf_vqT_lnk = mexWholeBodyModel('forward-kinematics', wf_R_b_arr, varargin{1,2}, varargin{1,3}, urdf_link_name);
        end

        function C_qv = generalizedBiasForces(~, wf_R_b, wf_p_b, q_j, dq_j, v_b)
            switch nargin
                case 6
                    % normal mode:
                    wf_R_b_arr = reshape(wf_R_b, 9, 1);
                    C_qv = mexWholeBodyModel('generalised-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                case 1
                    % optimized mode:
                    C_qv = mexWholeBodyModel('generalised-forces');
                otherwise
                    error('WBMBase::generalizedBiasForces: %s', WBM.wbmErrorMsg.WRONG_ARG);
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
                    f_c  = varargin{1,6};
                    Jc_t = varargin{1,7};

                    wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
                    C_qv = mexWholeBodyModel('generalised-forces', wf_R_b_arr, varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5});
                case 3 % optimized mode:
                    f_c  = varargin{1,1};
                    Jc_t = varargin{1,2};

                    C_qv = mexWholeBodyModel('generalised-forces');
                otherwise
                    error('WBMBase::generalizedForces: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            tau_gen = C_qv - Jc_t*f_c;
        end

        function tau_c = coriolisCentrifugalForces(~, wf_R_b, wf_p_b, q_j, dq_j, v_b)
            switch nargin
                case 6
                    % normal mode:
                    wf_R_b_arr = reshape(wf_R_b, 9, 1);
                    tau_c = mexWholeBodyModel('coriolis-centrifugal-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                case 1
                    % optimized mode:
                    tau_c = mexWholeBodyModel('coriolis-centrifugal-forces');
                otherwise
                    error('WBMBase::coriolisCentrifugalForces: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function tau_g = gravityForces(~, wf_R_b, wf_p_b, q_j)
            switch nargin
                case 4
                    % normal mode:
                    wf_R_b_arr = reshape(wf_R_b, 9, 1);
                    tau_g = mexWholeBodyModel('gravity-forces', wf_R_b_arr, wf_p_b, q_j);
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
            tau_vf = -obj.mwbm_model.frict_coeff.v .* dq_j;       % viscous friction torques
            tau_cf = -obj.mwbm_model.frict_coeff.c .* sign(dq_j); % Coulomb friction torques
            tau_fr =  tau_vf + tau_cf;                            % friction torques
        end

        function [M, C_qv, h_c] = wholeBodyDynamics(~, wf_R_b, wf_p_b, q_j, dq_j, v_b)
            switch nargin
                case 6
                    % normal mode:
                    wf_R_b_arr = reshape(wf_R_b, 9, 1);

                    M    = mexWholeBodyModel('mass-matrix', wf_R_b_arr, wf_p_b, q_j);
                    C_qv = mexWholeBodyModel('generalised-forces', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b);
                    h_c  = mexWholeBodyModel('centroidal-momentum', wf_R_b_arr, wf_p_b, q_j, dq_j, v_b); % = omega
                case 1
                    % optimized mode:
                    M    = mexWholeBodyModel('mass-matrix');
                    C_qv = mexWholeBodyModel('generalised-forces');
                    h_c  = mexWholeBodyModel('centroidal-momentum'); % = omega
                otherwise
                    error('WBMBase::wholeBodyDynamics: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function set.fixed_link(obj, lnk_name)
            if isempty(lnk_name)
                error('WBMBase::set.fixed_link: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end
            % update the default link name ...
            obj.mwbm_model.urdf_fixed_link = lnk_name;
        end

        function lnk_name = get.fixed_link(obj)
            lnk_name = obj.mwbm_model.urdf_fixed_link;
        end

        function set.wf_R_b(obj, new_rotm)
            if ( (size(new_rotm,1) ~= 3) || (size(new_rotm,2) ~= 3) )
                error('WBMBase::set.wf_R_b: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            obj.mwbm_model.wf_R_b = new_rotm;
        end

        function wf_R_b = get.wf_R_b(obj)
            wf_R_b = obj.mwbm_model.wf_R_b;
        end

        function set.wf_p_b(obj, new_pos)
            if (size(new_pos,1) ~= 3)
                error('WBMBase::set.wf_p_b: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            obj.mwbm_model.wf_p_b = new_pos;
        end

        function wf_p_b = get.wf_p_b(obj)
            wf_p_b = obj.mwbm_model.wf_p_b;
        end

        function set.g_wf(obj, new_g)
            if (size(new_g,1) ~= 3)
                error('WBMBase::set.g_wf: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            obj.mwbm_model.g_wf = new_g;
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
            if ( (size(frict_coeff.v,1) ~= obj.mwbm_model.ndof) || ...
                 (size(frict_coeff.c,1) ~= obj.mwbm_model.ndof) )
                error('WBMBase::set.frict_coeff: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            % update the friction coefficients ...
            obj.mwbm_model.frict_coeff.v = frict_coeff.v;
            obj.mwbm_model.frict_coeff.c = frict_coeff.c;
        end

        function frict_coeff = get.frict_coeff(obj)
            frict_coeff = obj.mwbm_model.frict_coeff;
        end

        function jl = get.joint_limits(obj)
            jl = obj.mwbm_model.jlim;
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

            if ~any(obj.mwbm_model.frict_coeff.v,1) % if frict_coeff.v = 0:
                strFrictions = '  frictionless';
            else
                strFrictions = sprintf(['  viscous frictions:  %s\n' ...
                                        '  Coulomb frictions:  %s'], ...
                                       mat2str(obj.mwbm_model.frict_coeff.v, prec), ...
                                       mat2str(obj.mwbm_model.frict_coeff.c, prec));
            end

            strParams = sprintf(['WBM Parameters:\n\n' ...
                                 ' #DoFs:             %d\n' ...
                                 '%s\n' ...
                                 ' fixed link to WF:  %s\n' ...
                                 ' R (base to WF):    %s\n' ...
                                 ' p (base to WF):    %s\n' ...
                                 ' g (WF):            %s\n\n' ...
                                 ' joint friction coefficients:\n%s\n'], ...
                                obj.mwbm_model.ndof, strURDFname, ...
                                obj.mwbm_model.urdf_fixed_link, ...
                                mat2str(obj.mwbm_model.wf_R_b, prec), ...
                                mat2str(obj.mwbm_model.wf_p_b, prec), ...
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
            if (robot_model.ndof > obj.MAX_NUM_JOINTS)
                error('WBMBase::initWBM: %s', WBM.wbmErrorMsg.MAX_NUM_LIMIT);
            end

            obj.mwbm_model = WBM.wbmBaseRobotModel;
            obj.mwbm_model.ndof            = robot_model.ndof;
            obj.mwbm_model.urdf_fixed_link = robot_model.urdf_fixed_link;

            if (robot_model.ndof > 0)
                if ~isempty(robot_model.frict_coeff.v)
                    if (size(robot_model.frict_coeff.v,1) ~= robot_model.ndof)
                        error('WBMBase::initWBM: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                    end

                    obj.mwbm_model.frict_coeff.v = robot_model.frict_coeff.v;
                    if ~isempty(obj.mwbm_model.frict_coeff.c)
                        if (size(robot_model.frict_coeff.c,1) ~= robot_model.ndof)
                            error('WBMBase::initWBM: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
                        end

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
                obj.initModel(); % use the default (URDF) model ...
            else
                % normal mode:
                [~,model_name, ext] = fileparts(robot_model.urdf_robot_name);
                if ~isempty(ext)
                    % use directly a specific URDF-file for the robot model ...
                    obj.initModelURDF(robot_model.urdf_robot_name);
                else
                    % set the model name of the robot which is supported by the WB-Toolbox ...
                    obj.initModel(model_name);
                end
            end
            % buffer the joint limits of the robot model for fast access ...
            [obj.mwbm_model.jlim.lwr, obj.mwbm_model.jlim.upr] = obj.getJointLimits();
        end

        function [nw_p_b, nw_R_b] = computeNewWorld2Base(obj, urdf_link_name, q_j)
            % get the transformation values from the base to the old world ...
            [ow_vqT_b,~,~,~] = obj.getState();
            % get the homogeneous transformation matrix H
            % from the base to the old world ...
            [ow_p_b, ow_R_b] = WBM.utilities.frame2posRotm(ow_vqT_b);
            ow_H_b = WBM.utilities.posRotm2tform(ow_p_b, ow_R_b);

            % get the transformation values from the reference link (contact link)
            % to the old world:
            if (nargin == 2)
                ow_vqT_rlnk = obj.forwardKinematics(urdf_link_name);
            else
                ow_vqT_rlnk = obj.forwardKinematics(ow_R_b, ow_p_b, q_j, urdf_link_name);
            end

            % compute the homogeneous transformation matrix H from the base to
            % the new world:
            ow_H_rlnk = WBM.utilities.frame2tform(ow_vqT_rlnk);
            nw_H_b    = ow_H_rlnk \ ow_H_b;

            % extract the translation and the rotation values ...
            [nw_p_b, nw_R_b] = WBM.utilities.tform2posRotm(nw_H_b);
        end

    end
end
