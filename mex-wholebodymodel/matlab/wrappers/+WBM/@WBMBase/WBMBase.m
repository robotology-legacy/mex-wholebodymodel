classdef WBMBase < handle
    properties(Dependent)
        % public properties for fast get/set methods:
        urdfLinkName@char
        wf_R_rootLnk@double matrix
        wf_p_rootLnk@double vector
        g_wf@double         vector
        robot_model@WBM.wbmBaseModelParams
    end

    properties(Access = protected)
        mwbm_params@WBM.wbmBaseModelParams
    end

    methods%(Access = public)
        % Constructor:
        function obj = WBMBase(model_params)
            if ~exist('model_params', 'var')
                error('WBMBase::WBMBase: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            obj.initWBM(model_params);
            % set the world frame (WF) to the initial parameters ...
            obj.updateWorldFrame(model_params.wf_R_rootLnk, model_params.wf_p_rootLnk, ...
                                 model_params.g_wf);
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

        function initModel(obj, urdf_robot_name)
            if ~exist('urdf_robot_name', 'var')
                % Optimized mode: use the default (URDF) robot name which is
                % defined in the environment variable YARP_ROBOT_NAME for
                % the WB(I)-Toolbox ...
                obj.mwbm_params.urdfRobot = getenv('YARP_ROBOT_NAME');
                wholeBodyModel('model-initialise');
                return
            end
            % else, use the robot name that is supported by the WB(I)-Toolbox
            % [URDF-file(s) must exist in the directory of the WB(I)-Tbx] ...
            obj.mwbm_params.urdfRobot = urdf_robot_name;
            wholeBodyModel('model-initialise', obj.mwbm_params.urdfRobot);
        end

        function initModelURDF(obj, urdf_file_name)
            if ~exist('urdf_file_name', 'var')
                error('WBMBase::initModelURDF: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            if ~exist('urdf_file_name', 'file')
                error('WBMBase::initModelURDF: %s', WBM.wbmErrorMsg.FILE_NOT_EXIST);
            end

            obj.mwbm_params.urdfRobot = urdf_file_name;
            wholeBodyModel('model-initialise-urdf', obj.mwbm_params.urdfRobot);
        end

        function setWorldFrame(obj, wf_R_rootLnk, wf_p_rootLnk, g_wf)
            switch nargin
                case {3, 4}
                    if ~exist('g_wf', 'var')
                        % use the default gravity vector ...
                        g_wf = obj.mwbm_params.g_wf;
                    end

                    % reshape the matrix into an 1-column array ...
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    wholeBodyModel('set-world-frame', wf_R_rlnk_arr, wf_p_rootLnk, g_wf);
                otherwise
                    error('WBMBase::setWorldFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function updateWorldFrame(obj, wf_R_rootLnk, wf_p_rootLnk, g_wf)
            if ( (nargin > 1) && (nargin ~= 4) )
                error('WBMBase::updateWorldFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            if (nargin == 4)
                % replace the old default parameters with the new values ...
                obj.mwbm_params.wf_R_rootLnk = wf_R_rootLnk;
                obj.mwbm_params.wf_p_rootLnk = wf_p_rootLnk;
                obj.mwbm_params.g_wf         = g_wf;
            end
            % update the world frame with the new default parameters ...
            obj.setWorldFrame(obj.mwbm_params.wf_R_rootLnk, obj.mwbm_params.wf_p_rootLnk, ...
                              obj.mwbm_params.g_wf);
        end

        function [w_p_b, w_R_b] = getWorldFrameFromFixedLink(obj, urdf_link_name, q_j)
            switch nargin
                case 3
                    % use another contact (constraint) link (*)
                    [w_p_b, w_R_b] = obj.computeNewWorld2Base(urdf_link_name, q_j);
                case 2
                    if exist('urdf_link_name', 'var')
                        % (*) ...
                        [w_p_b, w_R_b] = obj.computeNewWorld2Base(urdf_link_name);
                    else
                        % use the current (default) link name (**)
                        [w_p_b, w_R_b] = obj.computeNewWorld2Base(obj.mwbm_params.urdfLinkName, q_j);
                    end
                case 1
                    % (**) ...
                    [w_p_b, w_R_b] = obj.computeNewWorld2Base(obj.mwbm_params.urdfLinkName);
                otherwise
                    % should be never reached ...
                    error('WBMBase::getWorldFrameFromFixedLink: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function setState(~, q_j, dq_j, v_b)
            if (nargin ~= 4)
                error('WBMBase::setState: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            %if ( (length(q_j) ~= length(dq_j)) || ...
            %     (length(v_b) ~= 6) )
            %    error('WBMBase::setState: %s', WBM.wbmErrorMsg.WRONG_VEC_SIZE);
            %end

            wholeBodyModel('update-state', q_j, dq_j, v_b);
        end

        function [vqT_b, q_j, v_b, dq_j] = getState(~)
            [q_j, vqT_b, dq_j, v_b] = wholeBodyModel('get-state');
        end

        function stFltb = getFloatingBaseState(obj)
            stFltb = WBM.wbmFltgBaseState;
            [R_b, p_b, v_b] = wholeBodyModel('get-floating-base-state');

            stFltb.wf_R_b = R_b; % orientation of the base (in axis-angle representation)
            stFltb.wf_p_b = p_b; % cartesian position of the base
            stFltb.wf_v_b = v_b; % cartesian velocity and the rotational velocity of the base
        end

        function M = massMatrix(~, wf_R_rootLnk, wf_p_rootLnk, q_j)
            switch nargin
                case 4
                    % Normal mode:
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    M = wholeBodyModel('mass-matrix', wf_R_rlnk_arr, wf_p_rootLnk, q_j);
                case 1
                    % Optimized mode:
                    M = wholeBodyModel('mass-matrix');
                otherwise
                    error('WBMBase::massMatrix: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function [jl_lower, jl_upper] = getJointLimits(~)
            [jl_lower, jl_upper] = wholeBodyModel('joint-limits');
        end

        function J = jacobian(obj, urdf_link_name, wf_R_rootLnk, wf_p_rootLnk, q_j)
            if ~exist('urdf_link_name', 'var')
                % use the default link name ...
                urdf_link_name = obj.mwbm_params.urdfLinkName;
            end

            switch nargin
                case {4, 5}
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    J = wholeBodyModel('jacobian', wf_R_rlnk_arr, wf_p_rootLnk, q_j, urdf_link_name);
                case {1, 2}
                    J = wholeBodyModel('jacobian', urdf_link_name);
                otherwise
                    error('WBMBase::jacobian: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function djdq = dJdq(obj, urdf_link_name, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            if ~exist('urdf_link_name', 'var')
                urdf_link_name = obj.mwbm_params.urdfLinkName; % default ...
            end

            switch nargin
                case {6, 7}
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    djdq = wholeBodyModel('djdq', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b, urdf_link_name);
                case {1, 2}
                    djdq = wholeBodyModel('djdq', urdf_link_name);
                otherwise
                    error('WBMBase::dJdq: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function h_c = centroidalMomentum(~, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            switch nargin
                case 6
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    h_c = wholeBodyModel('centroidal-momentum', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b);
                case 1
                    h_c = wholeBodyModel('centroidal-momentum');
                otherwise
                    error('WBMBase::centroidalMomentum: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function wf_vqT_lnkfr = forwardKinematics(obj, urdf_link_name, wf_R_rootLnk, wf_p_rootLnk, q_j)
            if ~exist('urdf_link_name', 'var')
                urdf_link_name = obj.mwbm_params.urdfLinkName; % default ...
            end

            switch nargin
                case {4, 5}
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    wf_vqT_lnkfr = wholeBodyModel('forward-kinematics', wf_R_rlnk_arr, wf_p_rootLnk, q_j, urdf_link_name);
                case {1, 2}
                    wf_vqT_lnkfr = wholeBodyModel('forward-kinematics', urdf_link_name);
                otherwise
                    error('WBMBase::forwardKinematics: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function C_qv = generalizedBiasForces(~, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            switch nargin
                case 6
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    C_qv = wholeBodyModel('generalised-forces', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b);
                case 1
                    C_qv = wholeBodyModel('generalised-forces');
                otherwise
                    error('WBMBase::generalizedBiasForces: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function tau_c = coriolisCentrifugalForces(~, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            switch nargin
                case 6
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    tau_c = wholeBodyModel('coriolis-centrifugal-forces', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b);
                case 1
                    tau_c = wholeBodyModel('coriolis-centrifugal-forces');
                otherwise
                    error('WBMBase::coriolisCentrifugalForces: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function tau_g = gravityForces(~, wf_R_rootLnk, wf_p_rootLnk, q_j)
            switch nargin
                case 4
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    tau_g = wholeBodyModel('gravity-forces', wf_R_rlnk_arr, wf_p_rootLnk, q_j);
                case 1
                    tau_g = wholeBodyModel('gravity-forces');
                otherwise
                    error('WBMBase::gravityForces: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function set.urdfLinkName(obj, new_link_name)
            if isempty(new_link_name)
                error('WBMBase::set.urdfLinkName: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end
            % update the default link name ...
            obj.mwbm_params.urdfLinkName = new_link_name;
        end

        function lnk_name = get.urdfLinkName(obj)
            lnk_name = obj.mwbm_params.urdfLinkName;
        end

        function set.wf_R_rootLnk(obj, wf_R_rlnk)
            if ~isequal(size(wf_R_rlnk), [3,3])
                error('WBMBase::set.wf_R_rootLnk: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            obj.mwbm_params.wf_R_rootLnk = wf_R_rlnk;
        end

        function wf_R_rlnk = get.wf_R_rootLnk(obj)
            wf_R_rlnk = obj.mwbm_params.wf_R_rootLnk;
        end

        function set.wf_p_rootLnk(obj, wf_p_rlnk)
            if (size(wf_p_rlnk,1) ~= 3)
                error('WBMBase::set.wf_p_rootLnk: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            obj.mwbm_params.wf_p_rootLnk = wf_p_rlnk;
        end

        function wf_p_rlnk = get.wf_p_rootLnk(obj)
            wf_p_rlnk = obj.mwbm_params.wf_p_rootLnk;
        end

        function set.g_wf(obj, g)
            if (size(g,1) ~= 3)
                error('WBMBase::set.g_wf: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            obj.mwbm_params.g_wf = g;
        end

        function g_wf = get.g_wf(obj)
            g_wf = obj.mwbm_params.g_wf;
        end

        function robot_model = get.robot_model(obj)
            robot_model = obj.mwbm_params;
        end

        function dispWBMParams(obj, prec)
            if ~exist('prec', 'var')
                prec = 2;
            end
            strParams = sprintf(['WBM Parameters:\n\n' ...
                                 ' URDF robot name:     %s\n' ...
                                 ' URDF ref. link name: %s\n\n' ...
                                 ' R (root link to world frame):  %s\n' ...
                                 ' p (root link to world frame):  %s\n' ...
                                 ' g (world frame):               %s\n'], ...
                                obj.mwbm_params.urdfRobot, obj.mwbm_params.urdfLinkName, ...
                                mat2str(obj.mwbm_params.wf_R_rootLnk, prec), ...
                                mat2str(obj.mwbm_params.wf_p_rootLnk, prec), ...
                                mat2str(obj.mwbm_params.g_wf, prec));
            disp(strParams);
        end

    end

    methods(Access = private)
        function initWBM(obj, model_params)
            if ~isa(model_params, 'WBM.wbmBaseModelParams')
                error('WBMBase::initWBM: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.mwbm_params = WBM.wbmBaseModelParams;
            obj.mwbm_params.urdfLinkName = model_params.urdfLinkName;

            % Initialize the mex-wholeBodyModel for a floating base robot,
            % using Unified Robot Description Format (URDF):
            if isempty(model_params.urdfRobot)
                % Optimized mode:
                obj.initModel(); % use the default (URDF) robot name ...
            else
                % Normal mode:
                if WBM.utilities.fileExist(model_params.urdfRobot)
                    % use directly a specific URDF-file for the robot ...
                    obj.initModelURDF(model_params.urdfRobot);
                else
                    % set the robot-name which is supported by the WBI ...
                    obj.initModel(model_params.urdfRobot);
                end
            end
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
                ow_vqT_refLnk = obj.forwardKinematics(urdf_link_name, ow_R_b, ow_p_b, q_j);
            end

            % compute the hom. transformation matrix H from the base to
            % the new world:
            ow_H_refLnk = WBM.utilities.frame2tform(ow_vqT_refLnk);
            nw_H_b = ow_H_refLnk \ ow_H_b;

            % extract the translation and the rotation values ...
            nw_p_b = nw_H_b(1:3,4);
            nw_R_b = nw_H_b(1:3,1:3);
        end

    end
end
