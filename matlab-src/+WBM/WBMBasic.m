classdef WBMBasic < handle & matlab.mixin.Copyable
    properties(Access = private)
        wbm_params@wbmBasicModelParams
    end
        
    methods(Access = public)
        % Constructor:
        function obj = WBMBasic(model_params)
            if ~exist('model_params', 'var')
                error('WBMBasic::WBMBasic: %s', wbmErrorMsg.WRONG_ARG);
            end
            
            initWBM(model_params);
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
        
        function initModel(obj, urdf_robot_name)
            if ~exist('urdf_robot_name', 'var')
                % Optimized mode: use as default the URDF of the iCub-Robot
                %                 for the Gazebo simulator ...
                obj.wbm_params.urdfRobot = 'icubGazeboSim';
                wholeBodyModel('model-initialise');             
                return
            end
            % else, use the robot-name that is supported by the WBI
            % (URDF-file must exist in the directory of the WBI ) ...
            obj.wbm_params.urdfRobot = urdf_robot_name;
            wholeBodyModel('model-initialise', obj.wbm_params.urdfRobot);
        end
        
        function initModelURDF(obj, urdf_file_name)
            if ~exists('urdf_file_name', 'var')
                error('WBMBasic::initModelURDF: %s', wbmErrorMsg.WRONG_ARG);
            end
            if ~exists('urdf_file_name', 'file')
                error('WBMBasic::initModelURDF: %s', wbmErrorMsg.FILE_NOT_EXIST);
            end
            
            obj.wbm_params.urdfRobot = urdf_file_name;
            wholeBodyModel('model-initialise-urdf', obj.wbm_params.urdfRobot);
        end
        
        function setLinkName(obj, new_urdf_link_name)
            if ~exists('new_urdf_link_name', 'var')
                error('WBMBasic::setLinkName: %s', wbmErrorMsg.WRONG_ARG);
            end
            % update the default link name ...
            obj.wbm_params.urdfLinkName = new_urdf_link_name;
        end
        
        function setWorldFrame(obj, wf_R_rootLnk, wf_p_rootLnk, g_wf)
            if (nargin ~= 3)
                error('WBMBasic::setWorldFrame: %s', wbmErrorMsg.WRONG_ARG);
            end
            obj.wbm_params.wf_R_rootLnk = wf_R_rootLnk;
            obj.wbm_params.wf_p_rootLnk = wf_p_rootLnk;
            obj.wbm_params.g_wf = g_wf;

            % reshape the matrix into an 1-column array ...
            wf_R_rlnk_arr = reshape(obj.wbm_params.wf_R_rootLnk, [], 1);
            
            wholeBodyModel('set-world-frame', wf_R_rlnk_arr, ...
                           obj.wbm_params.wf_p_rootLnk, obj.wbm_params.g_wf);
        end
                
        function [w_p_b, w_R_b] = getWorldFrameFromFixedLink(obj, urdf_link_name, q_j)            
            switch nargin
                case 2
                    % use another contact (constraint) link (*)
                    [w_p_b, w_R_b] = obj.computeNewWorld2Base(urdf_link_name, q_j);
                case 1
                    if exist('urdf_link_name', 'var')
                        % (*) ...
                        [w_p_b, w_R_b] = obj.computeNewWorld2Base(urdf_link_name);
                    else
                        % use the current (default) link name (**)
                        [w_p_b, w_R_b] = obj.computeNewWorld2Base(obj.wbm_params.urdfLinkName, q_j);
                    end
                case 0
                    % (**) ...
                    [w_p_b, w_R_b] = obj.computeNewWorld2Base(obj.wbm_params.urdfLinkName);
                otherwise
                    % should be never reached ...
                    error('WBMBasic::getWorldFrameFromFixedLink: %s', wbmErrorMsg.WRONG_ARG);
            end            
        end
        
        function setState(q_j, dq_j, v_b)
            if (nargin ~= 3)
                error('WBMBasic::setState: %s', wbmErrorMsg.WRONG_ARG);
            end
            %if ( (length(q_j) ~= length(dq_j)) || ...
            %     (length(v_b) ~= 6) )
            %    error('WBMBasic::setState: %s', wbmErrorMsg.WRONG_VEC_SIZE);
            %end
            
            wholeBodyModel('update-state', q_j, dq_j, v_b);
        end
        
        function [T_b, q_j, v_b, dq_j] = getState(varargin)
            [q_j, T_b, dq_j, v_b] = wholeBodyModel('get-state');
        end
                
        function M = massMatrix(wf_R_rootLnk, wf_p_rootLnk, q_j)            
            switch nargin
                case 3
                    % Normal mode:
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, [], 1);
                    M = wholeBodyModel('mass-matrix', wf_R_rlnk_arr, wf_p_rootLnk, q_j);
                case 0
                    % Optimized mode:
                    M = wholeBodyModel('mass-matrix');
                otherwise
                    error('WBMBasic::massMatrix: %s', wbmErrorMsg.WRONG_ARG);
            end
        end       
        
        function [jl_lower, jl_upper] = getJointLimits(varargin)
            [jl_lower, jl_upper] = wholeBodyModel('joint-limits');
        end
        
        function J = jacobian(obj, urdf_link_name, wf_R_rootLnk, wf_p_rootLnk, q_j)
            if ~exist('urdf_link_name', 'var')
                % use the default link name ...
                urdf_link_name = obj.wbm_params.urdfLinkName;
            end
            
            switch nargin
                case 4
                case 3
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, [], 1);
                    J = wholeBodyModel('jacobian', wf_R_rlnk_arr, wf_p_rootLnk, q_j, urdf_link_name);
                case 1
                case 0
                    J = wholeBodyModel('jacobian', urdf_link_name);
                otherwise
                    error('WBMBasic::jacobian: %s', wbmErrorMsg.WRONG_ARG);
            end
        end
        
        function djdq = dJdq(obj, urdf_link_name, wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            if ~exist('urdf_link_name', 'var')
                urdf_link_name = obj.wbm_params.urdfLinkName; % default ...
            end

            switch nargin
                case 6
                case 5
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, [], 1);
                    djdq = wholeBodyModel('djdq', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b, urdf_link_name);
                case 1
                case 0
                    djdq = wholeBodyModel('djdq', urdf_link_name);
                otherwise
                    error('WBMBasic::dJdq: %s', wbmErrorMsg.WRONG_ARG);
            end
        end
        
        function H = centrodialMomentum(wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            switch nargin
                case 5
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, [], 1);
                    H = wholeBodyModel('centroidal-momentum', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b);
                case 0 
                    H = wholeBodyModel('centroidal-momentum');
                otherwise
                    error('WBMBasic::centrodialMomentum: %s', wbmErrorMsg.WRONG_ARG);
            end
        end
        
        function p = forwardKinematics(urdf_link_name, wf_R_rootLnk, wf_p_rootLnk, q_j)
            if ~exist('urdf_link_name', 'var')
                urdf_link_name = obj.wbm_params.urdfLinkName; % default ...
            end
  
            switch nargin
                case 4
                case 3
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, [], 1);
                    p = wholeBodyModel('forward-kinematics', wf_R_rlnk_arr, wf_p_rootLnk, q_j, urdf_link_name);
                case 1
                case 0
                    p = wholeBodyModel('forward-kinematics', urdf_link_name);
                otherwise
                    error('WBMBasic::forwardKinematics: %s', wbmErrorMsg.WRONG_ARG);                    
            end            
        end
        
        function C_qv = generalBiasForces(wf_R_rootLnk, wf_p_rootLnk, q_j, dq_j, v_b)
            switch nargin
                case 5
                    wf_R_rlnk_arr = reshape(wf_R_rootLnk, [], 1);
                    C_qv = wholeBodyModel('generalised-forces', wf_R_rlnk_arr, wf_p_rootLnk, q_j, dq_j, v_b);
                case 0
                    C_qv = wholeBodyModel('generalised-forces');
                otherwise
                    error('WBMBasic::generalBiasForces: %s', wbmErrorMsg.WRONG_ARG);
            end
        end       
  
        function wbm_params = getWBMParams(obj)
            wbm_params = obj.wbm_params;
        end
        
        function dispWBMParams(obj, precision)
            if ~exist('precision', 'var')
                precision = 2;
            end
            strParams = sprintf(['WBM parameters:\n\n'
                                 ' URDF robot name:     %s\n' ...
                                 ' URDF ref. link name: %s\n\n' ...
                                 ' R (root link to world frame):\n\n  %s\n\n' ...
                                 ' p (root link to world frame):\n\n  %s\n\n' ...
                                 ' g (world frame):\n\n %s\n\n'], ...
                                obj.wbm_params.urdfRobotName, obj.wbm_params.urdfLinkName, ...
                                mat2str(obj.wbm_params.wf_R_rootLnk, precision), ...
                                mat2str(obj.wbm_params.wf_p_rootLnk, precision), ...
                                mat2str(obj.wbm_params.g_wf, precision));
           disp(strParams);
        end

    end
    
    methods(Access = private)
        function initWBM(obj, model_params)
            if ~isa(model_params, 'wbmBasicModelParams')
                error('WBMBasic::initWBM: %s', wbmErrorMsg.WRONG_DATA_TYPE);
            end            
            obj.wbm_params = wbmBasicModelParams;
            obj.wbm_params.urdfLinkName = model_params.urdfLinkName;
            
            % Initialize the mex-wholeBodyModel for a floating base robot,
            % using Unified Robot Description Format (URDF):
            if isempty(model_params.urdfRobot)
                % Optimized mode:
                obj.initModel(); % use default robot
            else
                % Normal mode:
                if exists(model_params.urdfRobot, 'file')
                    % use directly a specific URDF-file for the robot ...
                    obj.initModelURDF(model_params.urdfRobot);
                else
                    % set the robot-name which is supported by the WBI ...
                    obj.initModel(model_params.urdfRobot);
                end
            end
            % set the world frame (WF) to the initial parameters ...
            setWorldFrame(model_params.wf_R_rootLnk, model_params.wf_p_rootLnk, ...
                          model_params.g_wf);
        end
        
        function [nw_p_b, nw_R_b] = computeNewWorld2Base(obj, urdf_link_name, q_j)
            % get the transformation values from the base to the old world ...
            [ow_qH_b,~,~,~] = obj.getState();
            [ow_p_b, ow_R_b] = frame2posRotm(ow_qH_b);
            % create the homogenous transformation matrix H
            % (from base to old world) ...
            ow_H_b = [ow_R_b ow_p_b; zeros(1,3) 1];
            
            % get the transformation values from the reference link to the
            % old world:
            if (nargin == 1)
                [ow_qH_refLnk] = obj.forwardKinematics(urdf_link_name);
            else
                [ow_qH_refLnk] = obj.forwardKinematics(urdf_link_name, ow_R_b, ow_p_b, q_j);
            end
            [ow_p_refLnk, ow_R_refLnk] = frame2posRotm(ow_qH_refLnk);
            
            % compute the hom. transformation matrix H from the base to
            % the new world:
            ow_H_refLnk = [ow_R_refLnk ow_p_refLnk; zeros(1,3) 1];
            nw_H_b = ow_H_refLnk \ ow_H_b;
            
            % extract the translation and the rotation values ...
            nw_p_b = nw_H_b(1:3,4);
            nw_R_b = nw_H_b(1:3,1:3);
        end

    end
end
