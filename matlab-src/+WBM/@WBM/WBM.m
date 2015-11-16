classdef WBM < WBMBase
    properties(Access = private)
        wbm_config@wbmBaseRobotConfig
    end
        
    methods(Access = public)
        % Constructor:
        function obj = WBM(model_params, robot_config, wf2FixLnk)
            % call the constructor of the superclass ...
            obj = obj@WBMBase(model_params);
            
            if ~exist('robot_config', 'var')
                error('WBM::WBM: %s', wbmErrorMsg.WRONG_ARG);
            end
            if ~exist('wf2FixLnk', 'var')
                wf2FixLnk = false; % default value ...
            end
 
            initConfig(robot_config);
            if (wf2FixLnk == true)
                if isempty(obj.wbm_config.initStateParams.cstrLinkNames)
                    error('WBM::WBM: %s', wbmErrorMsg.CARRAY_IS_EMPTY);
                end
                % set the world frame (WF) at a given rototranslation from
                % a chosen fixed link (the first entry of the constraint list):
                obj.setWorldFrame2FixedLink(obj.wbm_config.initStateParams.q_j, obj.wbm_config.initStateParams.dq_j, ...
                                            obj.wbm_config.initStateParams.v_b, obj.wbm_config.initStateParams.g_wf, ...
                                            obj.wbm_config.initStateParams.cstrLinkNames{1});
            end        
        end
        
        % Copy-function:
        function newObj = copy(obj)
            newObj = copy@WBMBase(obj);
        end
        
        % Destructor:
        function delete(obj)
           delete@WBMBase(obj);
           clear obj.wbm_config.initStateParams obj.wbm_config;
        end
        
        function T_b = getWorldFrameRototranslation(obj)
            [T_b,~,~,~] = obj.getState();
        end
        
        function setWorldFrame2FixedLink(obj, q_j, dq_j, v_b, g_wf, urdf_link_name)
            if ~exist('urdf_link_name', 'var')
                % use the default link ...
                urdf_link_name = obj.wbm_params.urdfLinkName;
            else
                % replace the (old) default link with a new link ...
                obj.setLinkName(urdf_link_name);
            end
            
            switch nargin
                case 5
                case 4            
                    obj.setState(q_j, dq_j, v_b);
                    [p_w2b, R_w2b] = obj.getWorldFrameFromFixedLink(urdf_link_name, q_j);
                    obj.setWorldFrame(R_w2b, p_w2b, g_wf);
                otherwise
                    error('WBM::setWorldFrame2FixedLink: %s', wbmErrorMsg.WRONG_ARG);
            end
        end
        
        forwardDynamics(obj, t, chi, ctrlTrqs)
        
        visualizeForwardDynamics(obj, t, chi, ctrlTrqs)
        
        function stParams = getStateParams(obj, stvChi)
            stParams = wbmStateParams;
            ndof = obj.wbm_config.ndof;
            
            % get positions and orientation ...
            stParams.x_b  = stvChi(1:3,:);
            stParams.qt_b = stvChi(4:7,:);
            stParams.q_j  = stvChi(8:ndof+7,:);
            % the velocities ...
            stParams.dx_b    = stvChi(ndof+8:ndof+10,:);
            stParams.omega_b = stvChi(ndof+11:ndof+13,:);           
            stParams.dq_j    = stvChi(ndof+14:2*ndof+13,:);
        end
        
        function stvChi = getStateVector(stParams)
            stvChi = [stParams.x_b; stParams.qt_b; stParams.q_j; ...
                      stParams.dx_b; stParams.omega_b; stParams.dq_j];
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
            % is derived from the class "wbmBaseRobotConfig" ...
            if ~isa(robot_config, 'wbmBaseRobotConfig')
                error('WBM::initWBM: %s', wbmErrorMsg.WRONG_DATA_TYPE);
            end
            
            obj.wbm_config = wbmBaseRobotConfig;
            obj.wbm_config.ndof = robot_config.ndof;
            obj.wbm_config.nCstrs = robot_config.nCstrs;
            obj.wbm_config.cstrLinkNames = robot_config.cstrLinkNames;
            obj.wbm_config.dampCoeff = robot_config.dampCoeff;
            obj.wbm_config.initState = robot_config.initState;
        end
                        
    end
end
