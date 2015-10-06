classdef WBMBasic < handle & matlab.mixin.Copyable
    properties(Access = private)
        wb_params@wbmBasicModelParams
    end
    
    properties(Access = private, Constant)
        wb_strWrongArgErr = 'Wrong number of input arguments!';
        wb_strDataTypeErr = 'Wrong data type!';
        wb_strVecSizeErr  = 'Wrong vector size!';
        wb_strWFrmCtrlErr = 'Unknown control parameter!';
    end
    
    methods(Access = public)
        % Constructor:
        function obj = WBMBasic(wbm_params, init_wf_ctrl)
            if ~exist('wbm_params', 'var')
                error('WBMBasic::WBMBasic: %s', obj.wb_strWrongArgErr);
            end
                        
            if ~exist('init_wf_ctrl', 'var')
                % set the WF from a given reference link as default ...
                init_wf_ctrl = 'wf_reflnk';
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
            %obj.wb_params = obj.getWBMParams();
            %newObj = WBMBasic(params);
        end
        
        function initModel(obj, urdf_robot_name)
            if ~exist('urdf_robot_name', 'var')
                % Optimized mode: use as default the URDF of the iCub-Robot
                %                 for the Gazebo simulator ...
                obj.wb_params.urdfRobotName = 'icubGazeboSim';
                wholeBodyModel('model-initialise');             
                return
            end
            % else, use the URDF of the given model of a floating base robot ...
            obj.wb_params.urdfRobotName = urdf_robot_name;
            wholeBodyModel('model-initialise', obj.wb_params.urdfRobotName);
        end
        
        function setWorldFrame(obj, R_rootlnk_wf, p_rootlnk_wf, g_wf)
            if (nargin ~= 3)
                error('WBMBasic::setWorldFrame: %s', obj.wb_strWrongArgErr);
            end
            obj.wb_params.R_rootlnk_wf = R_rootlnk_wf;
            obj.wb_params.p_rootlnk_wf = p_rootlnk_wf;
            obj.wb_params.g_wf = g_wf;

            % reshape the matrix into an 1-column array ...
            R_rlnk_wf_arr = reshape(obj.wb_params.R_rootlnk_wf, [], 1);
            
            wholeBodyModel('set-world-frame', R_rlnk_wf_arr, ...
                           obj.wb_params.p_rootlnk_w, obj.wb_params.g_wf);
        end
        
        function setWorldLink(obj, urdf_link_name, R_reflnk_wf, p_reflnk_wf, g_wf)
            if ( ~exist('R_reflnk_wf', 'var') && ~exist('p_reflnk_wf', 'var') )
                error('WBMBasic::setWorldLink: %s', obj.wb_strWrongArgErr);
            end
            obj.wb_params.R_reflnk_wf = R_reflnk_wf;
            obj.wb_params.p_reflnk_wf = p_reflnk_wf;

            R_rlnk_wf_arr = reshape(obj.wb_params.R_reflnk_wf, [], 1);

            switch nargin
                case 4
                    % Normal mode: set a specific URDF reference link ...
                    obj.wb_params.urdfRefLinkName = urdf_link_name;                    
                    obj.wb_params.g_wf = g_wf; % set the gravity ...

                    wholeBodyModel('set-world-link', obj.wb_params.urdfRefLinkName, ...
                                   R_rlnk_wf_arr, obj.wb_params.p_reflnk_wf, ...
                                   obj.wb_params.g_wf);
                case 2
                    % Optimized mode: use the default URDF reference link ...
                    obj.wb_params.urdfRefLinkName = 'l_sole';                       % check if it is really the default ref. link!!
                    wholeBodyModel('set-world-link', R_rlnk_wf_arr, obj.wb_params.p_reflnk_wf);
                otherwise
                    error('WBMBasic::setWorldLink: %s', obj.wb_strWrongArgErr);
            end
        end
        
        function setState(obj, q_j, dq_j, dx_b, omega_b)            
            if (nargin ~= 4)
                error('WBMBasic::setState: %s', obj.wb_strWrongArgErr);
            end
            %if ( (length(q_j) ~= length(dq_j)) || ...
            %     (length(dx_b) + length(omega_b) ~= 6) )
            %    error('WBMBasic::setState: %s', obj.wb_strVecSizeErr);
            %end
            
            v_b = [dx_b; omega_b]; % correct?!
            wholeBodyModel('update-state', q_j, dq_j, v_b);
        end
        
        function wbm_state = getState(varargin)
            % get the raw-data ...
            [q_j, xTb, dq_j, v_b] = wholeBodyModel('get-state');
           
            wbm_state = wbmState;
            wbm_state.q_j = q_j;
            wbm_state.dq_j = dq_j;
            wbm_state.x_b = xTb(1:3);
            wbm_state.qt_b = xTb(4:7);
            wbm_state.dx_b = v_b(1:3); % correct?!
            wbm_state.omega_b = v_b(4:6); % correct?!
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
                    error('WBMBasic::massMatrix: %s', obj.wb_strWrongArgErr);
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
                    error('WBMBasic::jacobian: %s', obj.wb_strWrongArgErr);
            end
        end
        
        function djdq = dJdq(obj, urdf_link_name, R_rootlnk_wf, p_rootlnk_wf, q_j, dq_j, v_xb)
            switch nargin
                case 6
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    djdq = wholeBodyModel('djdq', R_rlnk_wf_arr, p_rootlnk_wf, q_j, dq_j, v_xb, urdf_link_name);
                case 1
                    djdq = wholeBodyModel('djdq', urdf_link_name);
                otherwise
                    error('WBMBasic::dJdq: %s', obj.wb_strWrongArgErr);
            end
        end
        
        function H = centrodialMomentum(obj, R_rootlnk_wf, p_rootlnk_wf, q_j, dq_j, v_xb)
            switch nargin
                case 5
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    H = wholeBodyModel('centroidal-momentum', R_rlnk_wf_arr, p_rootlnk_wf, q_j, dq_j, v_xb);
                case 0 
                    H = wholeBodyModel('centroidal-momentum');
                otherwise
                    error('WBMBasic::centrodialMomentum: %s', obj.wb_strWrongArgErr);
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
                    error('WBMBasic::forwardKinematics: %s', obj.wb_strWrongArgErr);                    
            end            
        end
        
        function C_qv = genBiasForces(obj, R_rootlnk_wf, p_rootlnk_wf, q_j, dq_j, v_xb)
            switch nargin
                case 5
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    C_qv = wholeBodyModel('generalised-forces', R_rlnk_wf_arr, p_rootlnk_wf, q_j, dq_j, v_xb);
                case 0
                    C_qv = wholeBodyModel('generalised-forces');
                otherwise
                    error('WBMBasic::genBiasForces: %s', obj.wb_strWrongArgErr);
            end
        end       
  
        function wbm_params = getWBMParams(obj)
            wbm_params = obj.wb_params;
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
                                obj.wb_params.urdfRobotName, obj.wb_params.urdfLinkName, ...
                                mat2str(obj.wb_params.R_rootlnk_wf, precision), ...
                                mat2str(obj.wb_params.p_rootlnk_wf, precision), ...
                                mat2str(obj.wb_params.R_reflnk_wf, precision), ...
                                mat2str(obj.wb_params.p_reflnk_wf, precision), ...
                                mat2str(obj.wb_params.g_wf, precision));
           disp(strParams);
        end

    end
    
    methods(Access = private)
        function initWBM(obj, wbm_params, init_wf_ctrl)
            if ~isa(wbm_params, 'wbmBasicModelParams')
                error('WBMBasic::initWBM: %s', obj.wb_strDataTypeErr);
            end
            
            % Initialization:
            %
            obj.wb_params = wbmBasicModelParams;
            
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
            % chosen reference link:
            switch init_wf_ctrl
                case 'wf_reflnk'
                    % from a reference link ...
                    if isempty(wbm_params.urdfRefLinkName)
                        % Optimized mode:
                        setWorldLink(wbm_params.R_reflnk_wf, wbm_params.p_reflnk_wf);                
                    else
                        % Normal mode:
                        setWorldLink(wbm_params.urdfRefLinkName, wbm_params.R_reflnk_wf, ...
                                     wbm_params.p_reflnk_wf, wbm_params.g_wf);
                    end            
                case 'wf_rootlnk'
                    % from a root link ...
                    setWorldFrame(wbm_params.R_rootlnk_wf, wbm_params.p_rootlnk_wf, ...
                                  wbm_params.g_wf);
                otherwise
                    error('WBMBasic::initWBM: %s', obj.wb_strWFrmCtrlErr);
            end
        end

    end
end
