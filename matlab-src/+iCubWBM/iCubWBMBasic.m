classdef iCubWBMBasic < handle & matlab.mixin.Copyable
    properties(Access = private)
        wb_params@iCubWBMParams
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
                % note: works only for R2010b or newer.
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
                error('iCubWBMBasic::setState: Wrong number of input arguments!');
            end 
            wholeBodyModel('update-state', q_j, dq_j, v_wb);
        end
        
        function [q_j, xTb, dq_j, v_xb] = getState(varargin)
            [q_j, xTbT, dq_j, v_xb] = wholeBodyModel('get-state');
            xTb = [xTbT(1:3); xTbT(7); xTbT(4:6)];
        end
        
        function setWorldFrame(R_rootlnk_wf, p_rootlnk_wf, g_wf)
            if (nargin ~= 3)
                error('iCubWBMBasic::setWorldFrame: Wrong number of input arguments!');
            end
            R_rootlnk_wf_arr = reshape(R_rootlnk_wf, [], 1); % reshape the matrix to an 1-column array ...
            wholeBodyModel('set-world-frame', R_rootlnk_wf_arr, p_rootlnk_wf, g_wf);
        end
        
        function setWorldLink()
            
        end
        
        function massMatrix()
            
        end
        
        function jointLimits()
            
        end
        
        function jacobian()
            
        end
        
        function dJdq()
            
        end
        
        function centrodialMomentum()
            
        end
        
        function forwardKinematics()
            
        end
        
        function genBiasForces()
            
        end
        
        function getWBMParams()
            
        end
        
        function showWBMParams(obj)
            sprintf(1, ['iCub-parameters:\n robot (URDF): %s\n' ' NDOF: %i\n' ...
                        'link name (frame): %s\n'], obj.urdfName, obj.ndof, obj.linkName);
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
