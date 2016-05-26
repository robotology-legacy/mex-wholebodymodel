classdef WBMBase < handle
    properties(Dependent)
        % public properties for fast get/set methods:
        urdfLinkName@char
        wf_R_rootLnk@double matrix
        wf_p_rootLnk@double vector
        g_wf@double         vector
        robot_model@WBM.wbmBaseModelParams
    end

    properties(Constant)
        MAX_NUM_JOINTS = 35;
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

        function initModel(obj, urdf_model_name)
            if ~exist('urdf_model_name', 'var')
                % Optimized mode: use the default (URDF) robot model which
                % is defined in the environment variable YARP_ROBOT_NAME
                % of the WB-Toolbox ...
                obj.mwbm_params.urdfRobot = getenv('YARP_ROBOT_NAME');
                mexWholeBodyModel('model-initialise');
                return
            end
            % else, use the model name of the robot that is supported by the
            % WB-Toolbox (the URDF-file(s) must exist in the directory of
            % the WB-Toolbox) ...
            obj.mwbm_params.urdfRobot = urdf_model_name;
            mexWholeBodyModel('model-initialise', obj.mwbm_params.urdfRobot);
        end

        function initModelURDF(obj, urdf_file_name)
            if ~exist('urdf_file_name', 'var')
                error('WBMBase::initModelURDF: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            if ~WBM.utilities.fileExist(urdf_file_name)
                error('WBMBase::initModelURDF: %s', WBM.wbmErrorMsg.FILE_NOT_EXIST);
            end

            obj.mwbm_params.urdfRobot = urdf_file_name;
            mexWholeBodyModel('model-initialise-urdf', obj.mwbm_params.urdfRobot);
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
                    mexWholeBodyModel('set-world-frame', wf_R_rlnk_arr, wf_p_rootLnk, g_wf);
                otherwise
                    error('WBMBase::setWorldFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function updateWorldFrame(obj, wf_R_rootLnk, wf_p_rootLnk, g_wf)
            if (nargin == 2)
                error('WBMBase::updateWorldFrame: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            switch nargin
                case 4
                    % replace all old default parameters with the new values ...
                    obj.mwbm_params.wf_R_rootLnk = wf_R_rootLnk;
                    obj.mwbm_params.wf_p_rootLnk = wf_p_rootLnk;
                    obj.mwbm_params.g_wf         = g_wf;
                case 3
                    % replace only the orientation and translation ...
                    obj.mwbm_params.wf_R_rootLnk = wf_R_rootLnk;
                    obj.mwbm_params.wf_p_rootLnk = wf_p_rootLnk;
            end
            % update the world frame with the new (or changed) parameters ...
            obj.setWorldFrame(obj.mwbm_params.wf_R_rootLnk, obj.mwbm_params.wf_p_rootLnk, ...
                              obj.mwbm_params.g_wf);
        end

        function [w_p_b, w_R_b] = getWorldFrameFromFixedLink(obj, urdf_link_name, q_j)
            % compute the base world frame (WF) from a given contact (constraint) link:
            switch nargin
                case 3
                    % Normal mode: compute the WF for a specific joint configuration (positions) ...
                    [w_p_b, w_R_b] = obj.computeNewWorld2Base(urdf_link_name, q_j);
                case 2
                    % Optimized mode: compute the WF with the current joint configuration ...
                    [w_p_b, w_R_b] = obj.computeNewWorld2Base(urdf_link_name);
                otherwise
                    error('WBMBase::getWorldFrameFromFixedLink: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function [w_p_b, w_R_b] = getWorldFrameFromDfltFixedLink(obj, q_j)
            % compute the base world frame (WF) from the default contact (constraint) link:
            if exist('q_j', 'var')
                % Normal mode:
                [w_p_b, w_R_b] = obj.computeNewWorld2Base(obj.mwbm_params.urdfLinkName, q_j);
                return
            end
            % else, optimized mode:
            [w_p_b, w_R_b] = obj.computeNewWorld2Base(obj.mwbm_params.urdfLinkName);
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

        function M = massMatrix(~, wf_R_rootLnk, wf_p_rootLnk, q_j)
            switch nargin
                case 4
                    % Normal mode:
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    M = mexWholeBodyModel('mass-matrix', wf_R_rlnk_arr, wf_p_rootLnk, q_j);
                case 1
                    % Optimized mode:
                    M = mexWholeBodyModel('mass-matrix');
                otherwise
                    error('WBMBase::massMatrix: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function [jl_lower, jl_upper] = getJointLimits(~)
            [jl_lower, jl_upper] = mexWholeBodyModel('joint-limits');
        end

        function J = jacobian(obj, urdf_link_name, wf_R_rootLnk, wf_p_rootLnk, q_j)
            if ~exist('urdf_link_name', 'var')
                % use the default link name ...
                urdf_link_name = obj.mwbm_params.urdfLinkName;
            end

            switch nargin
                case {4, 5}
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    J = mexWholeBodyModel('jacobian', wf_R_rlnk_arr, wf_p_rootLnk, q_j, urdf_link_name);
                case {1, 2}
                    J = mexWholeBodyModel('jacobian', urdf_link_name);
                otherwise
                    error('WBMBase::jacobian: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function dJ = dJdq(obj, urdf_link_name, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            if ~exist('urdf_link_name', 'var')
                urdf_link_name = obj.mwbm_params.urdfLinkName; % default ...
            end

            switch nargin
                case {6, 7}
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    dJ = mexWholeBodyModel('djdq', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b, urdf_link_name);
                case {1, 2}
                    dJ = mexWholeBodyModel('djdq', urdf_link_name);
                otherwise
                    error('WBMBase::dJdq: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function h_c = centroidalMomentum(~, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            switch nargin
                case 6
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    h_c = mexWholeBodyModel('centroidal-momentum', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b);
                case 1
                    h_c = mexWholeBodyModel('centroidal-momentum');
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
                    wf_vqT_lnkfr = mexWholeBodyModel('forward-kinematics', wf_R_rlnk_arr, wf_p_rootLnk, q_j, urdf_link_name);
                case {1, 2}
                    wf_vqT_lnkfr = mexWholeBodyModel('forward-kinematics', urdf_link_name);
                otherwise
                    error('WBMBase::forwardKinematics: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function C_qv = generalizedBiasForces(~, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            switch nargin
                case 6
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    C_qv = mexWholeBodyModel('generalised-forces', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b);
                case 1
                    C_qv = mexWholeBodyModel('generalised-forces');
                otherwise
                    error('WBMBase::generalizedBiasForces: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function tau_c = coriolisCentrifugalForces(~, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            switch nargin
                case 6
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    tau_c = mexWholeBodyModel('coriolis-centrifugal-forces', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b);
                case 1
                    tau_c = mexWholeBodyModel('coriolis-centrifugal-forces');
                otherwise
                    error('WBMBase::coriolisCentrifugalForces: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function tau_g = gravityForces(~, wf_R_rootLnk, wf_p_rootLnk, q_j)
            switch nargin
                case 4
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, 9, 1);
                    tau_g = mexWholeBodyModel('gravity-forces', wf_R_rlnk_arr, wf_p_rootLnk, q_j);
                case 1
                    tau_g = mexWholeBodyModel('gravity-forces');
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
            if ( (size(wf_R_rlnk,1) ~= 3) || (size(wf_R_rlnk,2) ~= 3) )
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

            [strPath, strName, ext] = fileparts(obj.mwbm_params.urdfRobot);
            if ~isempty(ext)
                strURDFname = sprintf([' URDF filename:       %s\n' ...
                                       ' path:                %s'], ...
                                      strcat(strName, ext), strPath);
            else
                strURDFname = sprintf(' URDF robot model:    %s', strName);
            end

            strParams = sprintf(['WBM Parameters:\n\n' ...
                                 '%s\n' ...
                                 ' URDF ref. link name: %s\n\n' ...
                                 ' R (root link to world frame):  %s\n' ...
                                 ' p (root link to world frame):  %s\n' ...
                                 ' g (world frame):               %s\n'], ...
                                strURDFname, obj.mwbm_params.urdfLinkName, ...
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

            % Initialize the mex-WholeBodyModel for a floating base robot,
            % using Unified Robot Description Format (URDF):
            if isempty(model_params.urdfRobot)
                % Optimized mode:
                obj.initModel(); % use the default (URDF) model ...
            else
                % Normal mode:
                [~,model_name, ext] = fileparts(model_params.urdfRobot);
                if ~isempty(ext)
                    % use directly a specific URDF-file for the robot model ...
                    obj.initModelURDF(model_params.urdfRobot);
                else
                    % set the model name of the robot which is supported by the WB-Toolbox ...
                    obj.initModel(model_name);
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
            [nw_p_b, nw_R_b] = WBM.utilities.tform2posRotm(nw_H_b);
        end

    end
end
