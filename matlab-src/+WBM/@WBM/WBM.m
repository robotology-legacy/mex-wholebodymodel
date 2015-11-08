classdef WBM < WBMBasic
    properties(Access = private)
        wbm_config@wbmBasicRobotConfig
    end
    
    properties(Access = private, Constant)
       wbm_strWrongMatDimErr = 'Wrong matrix dimension!';
    end
    
    methods(Access = public)
        % Constructor:
        function obj = WBM(wbm_params, robot_config, init_wf_ctrl)
            args{1} = wbm_params;
            if exist('init_wf_ctrl', 'var')
                % set the given WF control value, else it will be used
                % inernally the default value ...
                args{2} = init_wf_ctrl;
            end
            % call the constructor of the superclass ...
            obj = obj@WBMBasic(args{:});
            
            if ~exist('robot_config', 'var')
                error('WBM::WBM: %s', obj.wbm_strWrongArgErr);
            end
 
            initConfig(robot_config);
            setState(obj.wbm_config.initState.q_j, obj.wbm_config.initState.dq_j, ...
                     obj.wbm_config.initState.dx_b, obj.wbm_config.initState.omega_b);            
        end
        
        function newObj = copy(obj)
            newObj = copy@WBMBasic(obj);
        end
                
        %function [xTb_init, chi_init] = getODEinitConditions(obj) % deprecated
        %    
        %end
        
        function T_b = getFrameRototranslation(obj)
            [T_b,~,~,~] = getState();
        end
        
        function updateWorldFrame(obj, q_j, dq_j, v_b, g_wf, urdf_link_name)
            
        end
        
        forwardDynamics(obj, t, ctrlTrqs, chi)
        
        %forwardDynamicsZeroExtForces(obj, t, chi) % deprecated
        
        visualizeForwardDynamics(obj, t, chi)
        
        function wbm_stParams = getStateParams(obj, stvChi)
            
        end
        
        function chi = getStateVector(obj, wbm_stParams)
            
        end

        function wbm_config = getWBMConfig(obj)
            wbm_config = obj.wbm_config;
        end
        
        function dispWBMConfig(obj, precision)
            if ~exist('precision', 'var')
                precision = 2;
            end
                        
            cellLnkNames = [num2cell(1:obj.wbm_config.nCstrs); obj.wbm_config.cstrLinkNames];
            strLnkNamesLst = sprintf('  %d  %s\n', cellLnkNames{:});
            
            cellInitSt{1} = sprintf('  q_j:      %s\n', mat2str(obj.wbm_config.initState.q_j, precision));
            cellInitSt{2} = sprintf('  dq_j:     %s\n', mat2str(obj.wbm_config.initState.dq_j, precision));
            cellInitSt{3} = sprintf('  x_b:      %s\n', mat2str(obj.wbm_config.initState.x_b, precision));
            cellInitSt{4} = sprintf('  qt_b:     %s\n', mat2str(obj.wbm_config.initState.qt_b, precision));
            cellInitSt{5} = sprintf('  dx_b:     %s\n', mat2str(obj.wbm_config.initState.dx_b, precision));
            cellInitSt{6} = sprintf('  omega_b:  %s\n', mat2str(obj.wbm_config.initState.omega_b, precision));
            strInitState = strcat(cellInitSt{1}, cellInitSt{2}, cellInitSt{3}, ...
                                  cellInitSt{4}, cellInitSt{5}, cellInitSt{6});
                              
            strConfig = sprintf(['Robot configuration:\n\n' ...
                                 ' NDOFs:         %d\n' ...
                                 ' # constraints: %d\n\n' ...
                                 ' Constraint link names:\n\n%s\n' ...
                                 ' damping coefficient: %f\n\n' ...
                                 ' initial state:\n\n%s\n'], ...
                                obj.wbm_config.ndof, obj.wbm_config.nCstrs, ...
                                strLnkNamesLst, obj.wbm_config.dampCoeff, ...
                                strInitState);
           disp(strConfig);
        end
        
    end
    
    methods(Access = private)
        function initConfig(obj, robot_config)
            % check if robot_config is an instance of a class that
            % is derived from the class "wbmBasicRobotConfig" ...
            if ~isa(robot_config, 'wbmBasicRobotConfig')
                error('WBM::initWBM: %s', obj.wbm_strDataTypeErr);
            end
            
            obj.wbm_config = wbmBasicRobotConfig;
            obj.wbm_config.ndof = robot_config.ndof;
            obj.wbm_config.nCstrs = robot_config.nCstrs;
            obj.wbm_config.cstrLinkNames = robot_config.cstrLinkNames;
            obj.wbm_config.dampCoeff = robot_config.dampCoeff;
            obj.wbm_config.initState = robot_config.initState;
        end
                        
    end
end
