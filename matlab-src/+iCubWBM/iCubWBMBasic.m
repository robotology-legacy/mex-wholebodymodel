classdef iCubWBMBasic < handle & matlab.mixin.Copyable
    properties(Access = private)
        wb_params@iCubWBMParams
    end
    
    properties(Access = private, Constant)
        wb_strArgError = 'Wrong number of input arguments!';
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
                error('iCubWBMBasic::setState: %s', wb_strArgError);
            end 
            wholeBodyModel('update-state', q_j, dq_j, v_wb);
        end
        
        function [q_j, xTb, dq_j, v_xb] = getState(varargin)
            [q_j, xTbT, dq_j, v_xb] = wholeBodyModel('get-state');
            xTb = [xTbT(1:3); xTbT(7); xTbT(4:6)];
        end
        
        function setWorldFrame(R_rootlnk_wf, p_rootlnk_wf, g_wf)
            if (nargin ~= 3)
                error('iCubWBMBasic::setWorldFrame: %s', wb_strArgError);
            end
            R_rootlnk_wf_arr = reshape(R_rootlnk_wf, [], 1); % reshape the matrix to an 1-column array ...
            wholeBodyModel('set-world-frame', R_rootlnk_wf_arr, p_rootlnk_wf, g_wf);
        end
        
        function setWorldLink(urdf_link_name, R_reflnk_wf, p_reflnk_wf, g_wf)
            R_reflnk_wf_arr = reshape(R_reflnk_wf, [], 1);

            switch nargin
                case 4
                    wholeBodyModel('set-world-link', urdf_link_name, R_reflnk_wf_arr, p_reflnk_wf, g_wf); % check arguments here!!!
                case 2
                    wholeBodyModel('set-world-link', urdf_link_name, R_reflnk_wf_arr); % check arguments here!!!
                otherwise
                    error('iCubWBMBasic::setWorldLink: %s', wb_strArgError);
            end
        end
        
        function M = massMatrix(q_j)
            if ~exist('q_j', 'var')
                M = wholeBodyModel('mass-matrix');
                return
            end
            M = wholeBodyModel('mass-matrix', q_j);
        end
        
        function [jl_lower, jl_upper] = getJointLimits(varargin)
            [jl_lower, jl_upper] = wholeBodyModel('joint-limits');
        end
        
        function J = jacobian(urdf_link_name, q_j)
            switch nargin
                case 2
                    J = wholeBodyModel('jacobian', urdf_link_name, q_j);                    
                case 1
                    J = wholeBodyModel('jacobian', urdf_link_name);
                otherwise
                    error('iCubWBMBasic::jacobian: %s', wb_strArgError);
            end
        end     
                        
        function djdq = dJdq(urdf_link_name, q_j, dq_j, v_xb)
            switch nargin
                case 4
                    djdq = wholeBodyModel('djdq', urdf_link_name, q_j, dq_j, v_xb);
                case 1
                    djdq = wholeBodyModel('djdq', urdf_link_name);
                otherwise
                    error('iCubWBMBasic::dJdq: %s', wb_strArgError);
            end
        end
        
        function H = centrodialMomentum(q_j, dq_j, v_xb)
            switch nargin
                case 3
                    H = wholeBodyModel('centroidal-momentum', q_j, dq_j, v_xb);
                case 0 
                    H = wholeBodyModel('centroidal-momentum');
                otherwise
                    error('iCubWBMBasic::centrodialMomentum: %s', wb_strArgError);
            end
        end
        
        function p = forwardKinematics(urdf_link_name, q_j)
            switch nargin
                case 2
                    p = wholeBodyModel('forward-kinematics', urdf_link_name, q_j);
                case 1
                    p = wholeBodyModel('forward-kinematics', urdf_link_name);
                otherwise
                    error('iCubWBMBasic::forwardKinematics: %s', wb_strArgError);                    
            end            
        end
        
        function C_qv = genBiasForces(q_j, dq_j, v_xb)
            switch nargin
                case 3
                    C_qv = wholeBodyModel('generalised-forces',);
                case 0
                    C_qv = wholeBodyModel('generalised-forces', q_j, dq_j, v_xb);
                otherwise
                    error('iCubWBMBasic::genBiasForces: %s', wb_strArgError);
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
                error('Wrong data type!');
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
            if isempty(icub_wbm_params.urdfRobotName)
                % use the default URDF for the Gazebo simulator ...
                obj.wb_params.urdfRobotName = 'icubGazeboSim';
                wholeBodyModel('model-initialise');                
            else
                % use an other URDF of a specific iCub model ...
                obj.wb_params.urdfRobotName = icub_wbm_params.urdfRobotName;
                wholeBodyModel('model-initialise', obj.wb_params.urdfRobotName);
            end

            % set the the world frame to a given rototranslation from
            % a chosen reference link:            
            if isempty(icub_wbm_params.urdfRefLinkName)
                obj.wb_params.urdfRefLinkName = 'l_sole'; % default URDF reference link
            else
                obj.wb_params.urdfRefLinkName = icub_wbm_params.urdfRefLinkName;           
            end
            wbm_setWorldLink(obj.wb_params.urdfRefLinkName, obj.wb_params.R_reflnk_wf, obj.wb_params.p_reflnk_wf);            
        end

    end
end
