classdef WBMBasic < handle & matlab.mixin.Copyable
    properties(Access = private)
        wbm_params@wbmBasicModelParams
    end
    
    properties(Access = private, Constant)
        wbm_strWrongArgErr = 'Wrong number of input arguments!';
        wbm_strDataTypeErr = 'Wrong data type!';
        wbm_strVecSizeErr  = 'Wrong vector size!';
        wbm_strFrmCtrlErr  = 'Unknown control parameter!';
        wbm_strFileErr     = 'File does not exist on given path!';
        %wbm_strPathErr     = 'This is not a path!';
    end
    
    methods(Access = public)
        % Constructor:
        function obj = WBMBasic(wbm_params, init_wf_ctrl)
            if ~exist('wbm_params', 'var')
                error('WBMBasic::WBMBasic: %s', obj.wb_strWrongArgErr);
            end
                        
            if ~exist('init_wf_ctrl', 'var')
                % % set the world frame to the current root link as default ... (wrong)
                % default: set the world frame to a chosen reference link ... 
                init_wf_ctrl = 'wfRootLnk';
            end
            initWBM(wbm_params, init_wf_ctrl);
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
            %obj.wbm_params = obj.getWBMParams();
            %newObj = WBMBasic(params);
        end
        
        function initModel(obj, urdf_robot_name)
            if ~exist('urdf_robot_name', 'var')
                % Optimized mode: use as default the URDF of the iCub-Robot
                %                 for the Gazebo simulator ...
                obj.wbm_params.urdfRobotName = 'icubGazeboSim';
                wholeBodyModel('model-initialise');             
                return
            end
            % else, use the URDF of the given model of a floating base robot ...
            obj.wbm_params.urdfRobotName = urdf_robot_name;
            wholeBodyModel('model-initialise', obj.wbm_params.urdfRobotName);
        end
        
        function initModelURDF(obj, urdf_file_name)
            if ~exists('urdf_file_name', 'var')
                error('WBMBasic::setWorldLink: %s', obj.wbm_strWrongArgErr);
            end
            if ~exists('urdf_file_name', 'file')
                error('WBMBasic::setWorldLink: %s', obj.wbm_strFileErr);
            end
            
            wholeBodyModel('model-initialise-urdf', urdf_file_name);
        end
        
        function setWorldFrame(obj, R_rootlnk_wf, p_rootlnk_wf, g_wf)
            if (nargin ~= 3)
                error('WBMBasic::setWorldFrame: %s', obj.wbm_strWrongArgErr);
            end
            obj.wbm_params.R_rootlnk_wf = R_rootlnk_wf;
            obj.wbm_params.p_rootlnk_wf = p_rootlnk_wf;
            obj.wbm_params.g_wf = g_wf;

            % reshape the matrix into an 1-column array ...
            R_rlnk_wf_arr = reshape(obj.wbm_params.R_rootlnk_wf, [], 1);
            
            wholeBodyModel('set-world-frame', R_rlnk_wf_arr, ...
                           obj.wbm_params.p_rootlnk_w, obj.wbm_params.g_wf);
        end
        
        % function setWorldLink(obj, urdf_link_name, R_reflnk_wf, p_reflnk_wf, g_wf) % deprecated
        %     if ( ~exist('R_reflnk_wf', 'var') && ~exist('p_reflnk_wf', 'var') )
        %         error('WBMBasic::setWorldLink: %s', obj.wbm_strWrongArgErr);
        %     end
        %     obj.wbm_params.R_reflnk_wf = R_reflnk_wf;
        %     obj.wbm_params.p_reflnk_wf = p_reflnk_wf;
        % 
        %     R_rlnk_wf_arr = reshape(obj.wbm_params.R_reflnk_wf, [], 1);
        % 
        %     switch nargin
        %         case 4
        %             % Normal mode: set a specific URDF reference link ...
        %             obj.wbm_params.urdfRefLinkName = urdf_link_name;                    
        %             obj.wbm_params.g_wf = g_wf; % set the gravity ...
        % 
        %             wholeBodyModel('set-world-link', obj.wbm_params.urdfRefLinkName, ...
        %                            R_rlnk_wf_arr, obj.wbm_params.p_reflnk_wf, ...
        %                            obj.wbm_params.g_wf);
        %         case 2
        %             % Optimized mode: use the previously set URDF-name of the
        %             % reference link or the default ...
        %             wholeBodyModel('set-world-link', R_rlnk_wf_arr, obj.wbm_params.p_reflnk_wf);
        %         otherwise
        %             error('WBMBasic::setWorldLink: %s', obj.wbm_strWrongArgErr);
        %     end
        % end
        
        function [p_w2b, R_w2b] = getWorldFrameFromFixedLink(obj, urdf_link_name, q_j)
            switch nargin
                case 2
                    [p_w2b, R_w2b]= computeNewWorld2Base(urdf_link_name, q_j);
                case 1
                    [p_w2b, R_w2b]= computeNewWorld2Base(urdf_link_name);
                otherwise
                    error('WBMBasic::getWorldFrameFromFixedLink: %s', obj.wbm_strWrongArgErr);
            end            
        end
        
        function setState(obj, q_j, dq_j, v_b)            
            if (nargin ~= 3)
                error('WBMBasic::setState: %s', obj.wbm_strWrongArgErr);
            end
            %if ( (length(q_j) ~= length(dq_j)) || ...
            %     (length(v_b) ~= 6) )
            %    error('WBMBasic::setState: %s', obj.wbm_strVecSizeErr);
            %end
            
            wholeBodyModel('update-state', q_j, dq_j, v_b);
        end
        
        function [T_b, q_j, v_b, dq_j] = getState(varargin)
            [q_j, T_b, dq_j, v_b] = wholeBodyModel('get-state');
        end
                
        function M = massMatrix(obj, R_rootlnk_wf, p_rootlnk_wf, q_j)            
            switch nargin
                case 3
                    % Normal mode:
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    M = wholeBodyModel('mass-matrix', R_rlnk_wf_arr, p_rootlnk_wf, q_j);
                case 0
                    % Optimized mode:
                    M = wholeBodyModel('mass-matrix');
                otherwise
                    error('WBMBasic::massMatrix: %s', obj.wbm_strWrongArgErr);
            end
        end       
        
        function [jl_lower, jl_upper] = getJointLimits(varargin)
            [jl_lower, jl_upper] = wholeBodyModel('joint-limits');
        end
        
        function J = jacobian(obj, urdf_link_name, R_rootlnk_wf, p_rootlnk_wf, q_j)
            switch nargin
                case 4
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    J = wholeBodyModel('jacobian', R_rlnk_wf_arr, p_rootlnk_wf, q_j, urdf_link_name);                    
                case 1
                    J = wholeBodyModel('jacobian', urdf_link_name);
                otherwise
                    error('WBMBasic::jacobian: %s', obj.wbm_strWrongArgErr);
            end
        end
        
        function djdq = dJdq(obj, urdf_link_name, R_rootlnk_wf, p_rootlnk_wf, q_j, dq_j, v_b)
            switch nargin
                case 6
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    djdq = wholeBodyModel('djdq', R_rlnk_wf_arr, p_rootlnk_wf, q_j, dq_j, v_b, urdf_link_name);
                case 1
                    djdq = wholeBodyModel('djdq', urdf_link_name);
                otherwise
                    error('WBMBasic::dJdq: %s', obj.wbm_strWrongArgErr);
            end
        end
        
        function H = centrodialMomentum(obj, R_rootlnk_wf, p_rootlnk_wf, q_j, dq_j, v_b)
            switch nargin
                case 5
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    H = wholeBodyModel('centroidal-momentum', R_rlnk_wf_arr, p_rootlnk_wf, q_j, dq_j, v_b);
                case 0 
                    H = wholeBodyModel('centroidal-momentum');
                otherwise
                    error('WBMBasic::centrodialMomentum: %s', obj.wbm_strWrongArgErr);
            end
        end
        
        function p = forwardKinematics(obj, urdf_link_name, R_rootlnk_wf, p_rootlnk_wf, q_j)
            switch nargin
                case 4
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    p = wholeBodyModel('forward-kinematics', R_rlnk_wf_arr, p_rootlnk_wf, q_j, urdf_link_name);
                case 1
                    p = wholeBodyModel('forward-kinematics', urdf_link_name);
                otherwise
                    error('WBMBasic::forwardKinematics: %s', obj.wbm_strWrongArgErr);                    
            end            
        end
        
        function C_qv = genBiasForces(obj, R_rootlnk_wf, p_rootlnk_wf, q_j, dq_j, v_b)
            switch nargin
                case 5
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    C_qv = wholeBodyModel('generalised-forces', R_rlnk_wf_arr, p_rootlnk_wf, q_j, dq_j, v_b);
                case 0
                    C_qv = wholeBodyModel('generalised-forces');
                otherwise
                    error('WBMBasic::genBiasForces: %s', obj.wbm_strWrongArgErr);
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
                                 ' R (ref. link to world frame):\n\n  %s\n\n' ...
                                 ' p (ref. link to world frame):\n\n  %s\n\n' ...
                                 ' g (world frame):\n\n %s\n\n'], ...
                                obj.wbm_params.urdfRobotName, obj.wbm_params.urdfLinkName, ...
                                mat2str(obj.wbm_params.R_rootlnk_wf, precision), ...
                                mat2str(obj.wbm_params.p_rootlnk_wf, precision), ...
                                mat2str(obj.wbm_params.R_reflnk_wf, precision), ...
                                mat2str(obj.wbm_params.p_reflnk_wf, precision), ...
                                mat2str(obj.wbm_params.g_wf, precision));
           disp(strParams);
        end

    end
    
    methods(Access = private)
        function initWBM(obj, wbm_params, init_wf_ctrl)
            if ~isa(wbm_params, 'wbmBasicModelParams')
                error('WBMBasic::initWBM: %s', obj.wbm_strDataTypeErr);
            end
            
            % Initialization:
            %
            obj.wbm_params = wbmBasicModelParams;
            
            % initialize the mex-wholeBodyModel of a floating base robot
            % by using the Unified Robot Description Format (URDF):
            if isempty(wbm_params.urdfRobotName)
                % Optimized mode:
                initModel();
            else
                % Normal mode:
                initModel(wbm_params.urdfRobotName);
            end
            
            % set the world frame (WF) to a given rototranslation from a
            % chosen link:
            switch init_wf_ctrl
                case 'wfRefLnk'
                    % use a reference link of the robot ...
                    if isempty(wbm_params.urdfRefLinkName)
                        % Optimized mode:
                        setWorldLink(wbm_params.R_reflnk_wf, wbm_params.p_reflnk_wf);                
                    else
                        % Normal mode:
                        setWorldLink(wbm_params.urdfRefLinkName, wbm_params.R_reflnk_wf, ...
                                     wbm_params.p_reflnk_wf, wbm_params.g_wf);
                    end            
                case 'wfRootLnk'
                    % use the current root link for the WF ...
                    setWorldFrame(wbm_params.R_rootlnk_wf, wbm_params.p_rootlnk_wf, ...
                                  wbm_params.g_wf);
                otherwise
                    error('WBMBasic::initWBM: %s', obj.wbm_strFrmCtrlErr);
            end
        end
        
        function [p_nw2b, R_nw2b] = computeNewWorld2Base(urdf_link_name, q_j)
            % get the transformation values from the old world to the base ...
            [qH_ow2b,~,~,~] = getState();
            [p_ow2b, R_ow2b] = frame2posRotm(qH_ow2b);
            % create the homogenous transformation matrix H
            % (from old world to base) ...
            H_ow2b = [R_ow2b p_ow2b; zeros(1,3) 1];
            
            % get the transformation values from the old world to the
            % reference link:
            if (nargin == 1)
                [qH_ow2refLnk] = forwardKinematics(urdf_link_name);
            else
                [qH_ow2refLnk] = forwardKinematics(urdf_link_name, R_ow2b, p_ow2b, q_j);
            end
            [p_ow2refLnk, R_ow2refLnk] = frame2posRotm(qH_ow2refLnk);
            
            % compute the hom. transformation matrix H from the new world
            % to the base:
            H_ow2refLnk = [R_ow2refLnk p_ow2refLnk; zeros(1,3) 1];
            H_nw2b = H_ow2refLnk \ H_ow2b;
            
            % extract the translation and the rotation values ...
            p_nw2b = H_nw2b(1:3,4);
            R_nw2b = H_nw2b(1:3,1:3);
        end

    end
end
