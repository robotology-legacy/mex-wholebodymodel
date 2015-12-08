classdef WBM < WBM.WBMBase
    properties(Dependent)
        stvChiInit@double vector
        stvLen@uint16     scalar
        vqTInit@double    vector
        stvqT@double      vector
        wbm_config@WBM.wbmBaseRobotConfig
    end

    properties(Access = protected)
        iwbm_config@WBM.wbmBaseRobotConfig
    end
        
    methods%(Access = public)
        % Constructor:
        function obj = WBM(model_params, robot_config, wf2FixLnk)
            % call the constructor of the superclass ...
            obj = obj@WBM.WBMBase(model_params);
            
            if ~exist('robot_config', 'var')
                error('WBM::WBM: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
            if ~exist('wf2FixLnk', 'var')
                wf2FixLnk = false; % default value ...
            end
 
            obj.initConfig(robot_config);
            if wf2FixLnk
                % set the world frame (WF) at a given rototranslation from
                % a chosen fixed link (the first entry of the constraint list):
                v_b = vertcat(obj.iwbm_config.initStateParams.dx_b, ...
                              obj.iwbm_config.initStateParams.omega_b);
                obj.setWorldFrame2FixedLink(obj.iwbm_config.initStateParams.q_j, obj.iwbm_config.initStateParams.dq_j, ...
                                            v_b, obj.iwbm_params.g_wf, obj.iwbm_config.cstrLinkNames{1});
            end
            % retrieve and update the initial rototranslation (VQS-Transf.) of the robot base (world frame) ...
            obj.updateInitRototranslation();
        end
        
        % Copy-function:
        function newObj = copy(obj)
            newObj = copy@WBM.WBMBase(obj);
        end
        
        % Destructor:
        function delete(obj)
           delete@WBM.WBMBase(obj);
           clear obj.iwbm_config.initStateParams obj.iwbm_config;
        end
                
        function setWorldFrame2FixedLink(obj, q_j, dq_j, v_b, g_wf, urdf_link_name)
            if ~exist('urdf_link_name', 'var')
                % use the default link ...
                urdf_link_name = obj.iwbm_params.urdfLinkName;
            else
                % replace the (old) default link with a new link ...
                obj.urdfLinkName = urdf_link_name;
            end
            
            switch nargin
                case {5, 6}            
                    obj.setState(q_j, dq_j, v_b);
                    [w_p_b, w_R_b] = obj.getWorldFrameFromFixedLink(urdf_link_name, q_j);
                    obj.setWorldFrame(w_R_b, w_p_b, g_wf);
                otherwise
                    error('WBM::setWorldFrame2FixedLink: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end
        end

        function updateInitRototranslation(obj)
            vqT_init = obj.stvqT; % get the vector-quaternion transf. of the current state ...
            obj.iwbm_config.initStateParams.x_b  = vqT_init(1:3,1); % translation/position
            obj.iwbm_config.initStateParams.qt_b = vqT_init(4:7,1); % orientation (quaternion)
        end
        
        [dstvChi, h] = forwardDynamics(obj, t, stvChi, ctrlTrqs)

        % [] = visualizeForwardDynamics(obj, x_out, tspan, sim_config)

        [] = setupSimulation(~, sim_config)

        % function plotSimulationResults(@simFunc, x_out, tspan, sim_config)

        % end
        
        function stParams = getStateParams(obj, stvChi)
            if ~iscolumn(stvChi)
               error('WBM::getStateParams: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            ndof     = obj.iwbm_config.ndof;
            stvLen   = obj.iwbm_config.stvLen;
            %stParams = WBM.wbmStateParams;
            stParams = obj.initStateParams(); % useful ??

            % get the base/joint positions and the base orientation ...
            stParams.x_b  = stvChi(1:3,1);
            stParams.qt_b = stvChi(4:7,1);
            stParams.q_j  = stvChi(8:ndof+7,1);
            % the corresponding velocities ...
            stParams.dx_b    = stvChi(ndof+8:ndof+10,1);
            stParams.omega_b = stvChi(ndof+11:ndof+13,1);
            stParams.dq_j    = stvChi(ndof+14:stvLen,1);
        end

        function stParams = getStateParamsData(obj, chi)
            stvLen = obj.iwbm_config.stvLen;

            [m, n] = size(chi);
            if (n ~= stvLen) 
                error('WBM::getStateParamsData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            ndof     = obj.iwbm_config.ndof;
            %stParams = WBM.wbmStateParams;
            stParams = obj.initStateParamsMatrices(m); % useful ??

            % extract all values ...
            stParams.x_b  = chi(1:m,1:3);
            stParams.qt_b = chi(1:m,4:7);
            stParams.q_j  = chi(1:m,8:ndof+7);
            
            stParams.dx_b    = chi(1:m,ndof+8:ndof+10);
            stParams.omega_b = chi(1:m,ndof+11:ndof+13);           
            stParams.dq_j    = chi(1:m,ndof+14:stvLen);
        end

        function stvPos = getPositions(obj, stvChi)
            if ~iscolumn(stvChi)
               error('WBM::getPositions: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            
            % extract the base VQS-Transformation (without S)
            % and the joint positions ...
            cutp = obj.iwbm_config.ndof + 7;
            stvPos = stvChi(1:cutp,1); % [x_b; qt_b; q_j]
        end

        function stmPos = getPositionsData(obj, chi)
            stvLen = obj.iwbm_config.stvLen;

            [m, n] = size(chi);
            if (n ~= stvLen) 
                error('WBM::getPositionsData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            
            cutp = obj.iwbm_config.ndof + 7;
            stmPos = chi(1:m,1:cutp); % m -by- [x_b, qt_b, q_j]
        end

        function stvVel = getVelocities(obj, stvChi)
            if ~iscolumn(stvChi)
               error('WBM::getVelocities: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            stvLen = obj.iwbm_config.stvLen;
            cutp = obj.iwbm_config.ndof + 8;
            % extract the velocities ...
            stvVel = stvChi(cutp:stvLen,1); % [dx_b; omega_b; dq_j]
        end

        function stmVel = getVelocitiesData(obj, chi)
            stvLen = obj.iwbm_config.stvLen;

            [m, n] = size(chi);
            if (n ~= stvLen) 
                error('WBM::getVelocitiesData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            cutp = obj.iwbm_config.ndof + 8;
            stmVel = chi(1:m,cutp:stvLen); % m -by- [dx_b, omega_b, dq_j]
        end

        function stvVelb = getBaseVelocities(obj, stvChi)
            if ~iscolumn(stvChi)
               error('WBM::getBaseVelocities: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            cutp1 = obj.iwbm_config.ndof + 8;
            cutp2 = obj.iwbm_config.ndof + 13;
            stvVelb = stvChi(cutp1:cutp2,1); % [dx_b; omega_b]
        end
        
        function stmVelb = getBaseVelocitiesData(obj, chi)
            stvLen = obj.iwbm_config.stvLen;

            [m, n] = size(chi);
            if (n ~= stvLen) 
                error('WBM::getBaseVelocitiesData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            cutp1 = obj.iwbm_config.ndof + 8;
            cutp2 = obj.iwbm_config.ndof + 13;
            stmVelb = chi(1:m,cutp1:cutp2); % m -by- [dx_b, omega_b]
        end
        
        function vqT = getRototranslation(~, stParams)
            if isempty(stParams)
                error('WBM::getRototranslation: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end

            vqT = vertcat(stParams.x_b, stParams.qt_b);
        end

        function stvChi = toStateVector(~, stParams)
            if isempty(stParams)
                error('WBM::toStateVector: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end

            stvChi = vertcat(stParams.x_b, stParams.qt_b, stParams.q_j, ...
                             stParams.dx_b, stParams.omega_b, stParams.dq_j);
        end

        function stvChiInit = get.stvChiInit(obj)
            stInit = obj.iwbm_config.initStateParams;
            stvChiInit = vertcat(stInit.x_b, stInit.qt_b, stInit.q_j, ...
                                 stInit.dx_b, stInit.omega_b, stInit.dq_j);
        end

        function stvLen = get.stvLen(obj)
            stvLen = obj.iwbm_config.stvLen;
        end

        function vqTInit = get.vqTInit(obj)
            stInit = obj.iwbm_config.initStateParams;
            vqTInit = vertcat(stInit.x_b, stInit.qt_b);
        end

        function stvqT = get.stvqT(obj)
            [stvqT,~,~,~] = obj.getState();
        end

        function wbm_config = get.wbm_config(obj)
            wbm_config = obj.iwbm_config;
        end
        
        function dispWBMConfig(obj, prec)
            if ~exist('prec', 'var')
                prec = 2;
            end
                        
            cellLnkNames = [num2cell(1:obj.iwbm_config.nCstrs); obj.iwbm_config.cstrLinkNames];
            strLnkNamesLst = sprintf('  %d  %s\n', cellLnkNames{:});
            stInit = obj.iwbm_config.initStateParams;
            
            cellInitSt{1} = sprintf('  q_j:      %s', mat2str(stInit.q_j, prec));
            cellInitSt{2} = sprintf('  dq_j:     %s', mat2str(stInit.dq_j, prec));
            cellInitSt{3} = sprintf('  x_b:      %s', mat2str(stInit.x_b, prec));
            cellInitSt{4} = sprintf('  qt_b:     %s', mat2str(stInit.qt_b, prec));
            cellInitSt{5} = sprintf('  dx_b:     %s', mat2str(stInit.dx_b, prec));
            cellInitSt{6} = sprintf('  omega_b:  %s', mat2str(stInit.omega_b, prec));
            strInitState  = sprintf('%s\n%s\n%s\n%s\n%s\n%s', cellInitSt{1}, cellInitSt{2}, ...
                                    cellInitSt{3}, cellInitSt{4}, cellInitSt{5}, cellInitSt{6});
                              
            strConfig = sprintf(['Robot configuration:\n\n' ...
                                 ' NDOFs:         %d\n' ...
                                 ' # constraints: %d\n\n' ...
                                 ' constraint link names:\n\n%s\n' ...
                                 ' damping coefficient: %f\n\n' ...
                                 ' initial state:\n\n%s\n'], ...
                                obj.iwbm_config.ndof, obj.iwbm_config.nCstrs, ...
                                strLnkNamesLst, obj.iwbm_config.dampCoeff, ...
                                strInitState);
           disp(strConfig);
        end
        
    end
    
    methods(Access = private)
        function initConfig(obj, robot_config)
            % check if robot_config is an instance of a class that
            % is derived from "wbmBaseRobotConfig" ...
            if ~isa(robot_config, 'WBM.wbmBaseRobotConfig')
                error('WBM::initWBM: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.iwbm_config = WBM.wbmBaseRobotConfig;
            
            if (robot_config.ndof == 0)
                error('WBM::initWBM: %s', WBM.wbmErrorMsg.VALUE_IS_ZERO);
            end            
            obj.iwbm_config.ndof          = robot_config.ndof;
            obj.iwbm_config.stvLen        = 2*obj.iwbm_config.ndof + 13;

            if (length(robot_config.cstrLinkNames) ~= robot_config.nCstrs)
                error('WBM::initWBM: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            obj.iwbm_config.cstrLinkNames = robot_config.cstrLinkNames;
            obj.iwbm_config.nCstrs        = robot_config.nCstrs;
            obj.iwbm_config.dampCoeff     = robot_config.dampCoeff;

            if isempty(robot_config.initStateParams)
                error('WBM::initWBM: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end            
            % if 'initStateParams' is not empty, check the dimensions ...
            stInit = robot_config.initStateParams;
            len = size(stInit.x_b,1) + size(stInit.qt_b,1) + size(stInit.q_j,1) + ...
                  size(stInit.dx_b,1) + size(stInit.omega_b,1) + size(stInit.dq_j,1);
            if ( (len ~= 0) && (len ~= obj.iwbm_config.stvLen) )
                if (len ~= (obj.iwbm_config.stvLen - 7)) % length without x_b & qt_b (they will be updated afterwards)
                    error('WBM::initWBM: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
                end
            end
            obj.iwbm_config.initStateParams = robot_config.initStateParams;
        end

        function stParams = initStateParams(obj)
            stParams = WBM.wbmStateParams;
            % allocate memory for each variable ...
            stParams.x_b  = zeros(3,1);
            stParams.qt_b = zeros(4,1);
            stParams.q_j  = zeros(obj.iwbm_config.ndof,1);

            stParams.dx_b    = zeros(3,1);
            stParams.omega_b = zeros(3,1);
            stParams.dq_j    = zeros(obj.iwbm_config.ndof,1);
        end

        function stParams = initStateParamsMatrices(obj, m)
            stParams = WBM.wbmStateParams;

            stParams.x_b  = zeros(m,3);
            stParams.qt_b = zeros(m,4);
            stParams.q_j  = zeros(m,obj.iwbm_config.ndof);
            
            stParams.dx_b    = zeros(m,3);
            stParams.omega_b = zeros(m,3);
            stParams.dq_j    = zeros(m,obj.iwbm_config.ndof);
        end
                        
    end
end
