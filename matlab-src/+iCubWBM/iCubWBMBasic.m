classdef iCubWBMBasic < handle & matlab.mixin.Copyable
    properties(Access = private)
        wb_params@iCubWBMParams
        %wb_bOptDefault@logical
    end
    
    properties(Access = private, Constant)
        wb_strWrongArgErr = 'Wrong number of input arguments!';
        wb_strDataTypeErr = 'Wrong data type!';       
    end
    
    methods(Access = public)
        % Constructor:
        function obj = iCubWBMBasic(icub_wbm_params)
            initWBM(icub_wbm_params);
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
            %params = obj.getWBMParams();
            %newObj = iCubWBMBasic(params);
        end
        
        function initModel(urdf_robot_name)
            if exist('urdf_robot_name', 'var')
                obj.wb_params.urdfRobotName = urdf_robot_name;
                wholeBodyModel('model-initialise', obj.wb_params.urdfRobotName);
                return
            end
            % else, use the default "icubGazeboSim" ...
            obj.wb_params.urdfRobotName = 'icubGazeboSim';
            wholeBodyModel('model-initialise');             
        end
        
        function setState(q_j, dq_j, v_wb)            
            if (nargin ~= 3)
                error('iCubWBMBasic::setState: %s', obj.wb_strWrongArgErr);
            end 
            wholeBodyModel('update-state', q_j, dq_j, v_wb);
        end
        
        function [q_j, xTb, dq_j, v_xb] = getState(varargin)
            [q_j, xTbT, dq_j, v_xb] = wholeBodyModel('get-state');
            xTb = [xTbT(1:3); xTbT(7); xTbT(4:6)]; % position correction of the outputs
                                                   % (1. cart. position, 2. real part and
                                                   %  3. imag. part of the quaternion)                                                  
            %wb_state = iCubWBMState;
            %wb_state.q_j = q_j;
            %wb_state.dq_j = dq_j;
            %wb_state.x_b = xTb(1:3);
            %wb_state.qt_b = xTb(4:7);
            %wb_state.dx_b = v_xb(1:3); % ??? to check!!!
            %wb_state.omega_b = v_xb(4:6); % ??? to check!!!
        end
        
        function setWorldFrame(R_rootlnk_wf, p_rootlnk_wf, g_wf)
            if (nargin ~= 3)
                error('iCubWBMBasic::setWorldFrame: %s', obj.wb_strWrongArgErr);
            end
            R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1); % reshape the matrix to an 1-column array ...
            wholeBodyModel('set-world-frame', R_rlnk_wf_arr, p_rootlnk_wf, g_wf);
        end
        
        function setWorldLink(urdf_link_name, R_reflnk_wf, p_reflnk_wf, g_wf)
            R_rlnk_wf_arr = reshape(R_reflnk_wf, [], 1);

            switch nargin
                case 4
                    wholeBodyModel('set-world-link', urdf_link_name, R_rlnk_wf_arr, p_reflnk_wf, g_wf);
                case 2
                    wholeBodyModel('set-world-link', R_rlnk_wf_arr, p_reflnk_wf);
                otherwise
                    error('iCubWBMBasic::setWorldLink: %s', obj.wb_strWrongArgErr);
            end
        end    
        
        function M = massMatrix(R_rootlnk_wf, p_rootlnk_wf, q_j)
            switch nargin
                case 3
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    M = wholeBodyModel('mass-matrix', R_rlnk_wf_arr, p_rootlnk_wf, q_j);
                case 0
                    M = wholeBodyModel('mass-matrix');
                otherwise
                    error('iCubWBMBasic::massMatrix: %s', obj.wb_strWrongArgErr);
            end
        end       
        
        function [jl_lower, jl_upper] = getJointLimits(varargin)
            [jl_lower, jl_upper] = wholeBodyModel('joint-limits');
        end
        
        function J = jacobian(urdf_link_name, R_rootlnk_wf, p_rootlnk_wf, q_j)
            switch nargin
                case 4
                    J = wholeBodyModel('jacobian', R_rootlnk_wf, p_rootlnk_wf, q_j, urdf_link_name);                    
                case 1
                    J = wholeBodyModel('jacobian', urdf_link_name);
                otherwise
                    error('iCubWBMBasic::jacobian: %s', obj.wb_strWrongArgErr);
            end
        end
        
        function djdq = dJdq(urdf_link_name, R_rootlnk_wf, p_rootlnk_wf, q_j, dq_j, v_xb)
            switch nargin
                case 6
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    djdq = wholeBodyModel('djdq', R_rlnk_wf_arr, p_rootlnk_wf, q_j, dq_j, v_xb, urdf_link_name);
                case 1
                    djdq = wholeBodyModel('djdq', urdf_link_name);
                otherwise
                    error('iCubWBMBasic::dJdq: %s', obj.wb_strWrongArgErr);
            end
        end
        
        function H = centrodialMomentum(R_rootlnk_wf, p_rootlnk_wf, q_j, dq_j, v_xb)
            switch nargin
                case 5
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    H = wholeBodyModel('centroidal-momentum', R_rlnk_wf_arr, p_rootlnk_wf, q_j, dq_j, v_xb);
                case 0 
                    H = wholeBodyModel('centroidal-momentum');
                otherwise
                    error('iCubWBMBasic::centrodialMomentum: %s', obj.wb_strWrongArgErr);
            end
        end
        
        function p = forwardKinematics(urdf_link_name, R_rootlnk_wf, p_rootlnk_wf, q_j)
            switch nargin
                case 4
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    p = wholeBodyModel('forward-kinematics', R_rlnk_wf_arr, p_rootlnk_wf, q_j, urdf_link_name);
                case 1
                    p = wholeBodyModel('forward-kinematics', urdf_link_name);
                otherwise
                    error('iCubWBMBasic::forwardKinematics: %s', obj.wb_strWrongArgErr);                    
            end            
        end
        
        function C_qv = genBiasForces(R_rootlnk_wf, p_rootlnk_wf, q_j, dq_j, v_xb)
            switch nargin
                case 5
                    R_rlnk_wf_arr = reshape(R_rootlnk_wf, [], 1);
                    C_qv = wholeBodyModel('generalised-forces', R_rlnk_wf_arr, p_rootlnk_wf, q_j, dq_j, v_xb);
                case 0
                    C_qv = wholeBodyModel('generalised-forces');
                otherwise
                    error('iCubWBMBasic::genBiasForces: %s', obj.wb_strWrongArgErr);
            end
        end       
  
        function wbm_params = getWBMParams(obj)
            wbm_params = obj.wb_params;
        end
        
        function dispWBMParams(obj, precision)
            if ~exist('precision', 'var')
                precision = 2;
            end
            strParams = sprintf(['iCub-WBM parameters:\n\n'
                                 ' URDF robot name:     %s\n' ...
                                 ' URDF ref. link name: %s\n\n' ...
                                 ' R (root link to world frame):\n\n %s\n\n' ...
                                 ' p (root link to world frame):\n\n %s\n\n' ...
                                 ' R (ref. link to world frame):\n\n %s\n\n' ...
                                 ' p (ref. link to world frame):\n\n %s\n\n' ...
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
        function initWBM(icub_wbm_params)
            if ~isa(icub_wbm_params, 'iCubWBMParams')
                error(obj.wb_strDataTypeErr);
            end

            % Initialization:
            %
            obj.wb_params.R_rootlnk_wf = icub_wbm_params.R_rootlnk_wf;
            obj.wb_params.p_rootlnk_wf = icub_wbm_params.p_rootlnk_wf;
            obj.wb_params.R_reflnk_wf = icub_wbm_params.R_reflnk_wf;
            obj.wb_params.p_reflnk_wf = icub_wbm_params.p_reflnk_wf;
            obj.wb_params.g_wf = icub_wbm_params.g_wf;
            
            % initialize the mex-wholeBodyModel of the iCub-Robot
            % by using the Unified Robot Description Format (URDF):
            %
            if isempty(icub_wbm_params.urdfRobotName)
                % Optimized mode: use the default URDF for the Gazebo simulator ...
                obj.wb_params.urdfRobotName = 'icubGazeboSim';
                wholeBodyModel('model-initialise');                
            else
                % use an other URDF of a specific iCub model ...
                obj.wb_params.urdfRobotName = icub_wbm_params.urdfRobotName;
                wholeBodyModel('model-initialise', obj.wb_params.urdfRobotName);
            end

            % set the world frame to a given rototranslation from a chosen
            % reference link:
            %            
            R_rlnk_wf_arr = reshape(obj.wb_params.R_reflnk_wf, [], 1); % transform matrix to an 1-column array ...   
            if isempty(icub_wbm_params.urdfRefLinkName)
                % Optimized mode: use the default URDF reference link ...
                obj.wb_params.urdfRefLinkName = 'l_sole'; % check if it is really the default ref. link!!
                wholeBodyModel('set-world-link', R_rlnk_wf_arr, obj.wb_params.p_reflnk_wf);                
            else
                obj.wb_params.urdfRefLinkName = icub_wbm_params.urdfRefLinkName;           
                wholeBodyModel('set-world-link', obj.wb_params.urdfRefLinkName, R_rlnk_wf_arr, obj.wb_params.p_reflnk_wf, obj.wb_params.g_wf);
            end
        end

    end
end
