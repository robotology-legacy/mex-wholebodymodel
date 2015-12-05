classdef WBM < WBM.WBMBase
    properties(Dependent)
        stvChiInit@double vector
        vqTInit@double vector
        stvqT@double vector
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
                v_b = zeros(6,1);
                v_b(1:3,1) = obj.iwbm_config.initStateParams.dx_b;
                v_b(4:6,1) = obj.iwbm_config.initStateParams.omega_b;
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
        
        forwardDynamics(obj, t, chi, ctrlTrqs)

        function [dchi, h] = forwardDynamics_test(obj, t, chi, ctrlTrqs)
            ndof = obj.iwbm_config.ndof;
            nCstrs = obj.iwbm_config.nCstrs;
            dampCoeff = obj.iwbm_config.dampCoeff;

            % get the current state parameters from the run-time variable "chi" ...
            stp = obj.getStateParams(chi);
            omega_w = stp.omega_b;
            v_bw = [stp.dx_b; omega_w];
            %v = [stp.dx_b; omega_w; stp.dq_j];

            % mex-WBM calls:
            %
            obj.setState(stp.q_j, stp.dq_j, v_bw);

            % reconstruct the rotation of the 'root link' to the 'world'
            % from the quaternion part of the transformation vector vqT_b:
            vqT_b = obj.stvqT;
            [~,R_b] = WBM.utilities.frame2posRotm(vqT_b);

            M = obj.massMatrix();
            h = obj.generalBiasForces();

            % compute the Jacobian and the corresponding derivative Jacobian for
            % each contact constraint:
            Jc = zeros(6*nCstrs,6+ndof);
            dJcDq = zeros(6*nCstrs,1);
            for i = 1:nCstrs
                Jc(6*i-5:6*i,:)    = obj.jacobian(obj.iwbm_config.cstrLinkNames{i}); % 6*(i-1)+1 == 6*i-5
                dJcDq(6*i-5:6*i,:) = obj.dJdq(obj.iwbm_config.cstrLinkNames{i});
            end

            % get the current control torque vector ...
            tau = ctrlTrqs.tau(t);

            % contact force computations:
            JcMinv = Jc/M;
            JcMinvJct = JcMinv * Jc';
            tauDamp = -dampCoeff * stp.dq_j;

            % calculate the contact (constraint) force ...
            f_c = JcMinvJct \ (JcMinv * (h - [zeros(6,1); (tau + tauDamp)]) - dJcDq);

            % need to apply root-to-world rotation to the spatial angular velocity omega_w to
            % obtain angular velocity in body frame omega_b. This is then used in the
            % quaternion derivative computation:
            omega_b = R_b * omega_w;
            dqt_b = WBM.utilities.quatDerivative(stp.qt_b, omega_b);

            dx = [stp.dx_b; dqt_b; stp.dq_j];
            dv = M \ (Jc'*f_c + [zeros(6,1); (tau + tauDamp)] - h);
            dchi = [dx; dv];
            %kinEnergy = 0.5*v'*M*v;
        end

        function [dchi, h] = forwardDynamics_test2(obj, t, chi, ctrlTrqs)
            %% extraction of state
            %ndof = param.ndof;
            ndof = obj.iwbm_config.ndof;

            x_b = chi(1:3,:);
            qt_b = chi(4:7,:);
            qj = chi(8:ndof+7,:);
            %x = [x_b;qt_b;qj];

            dx_b = chi(ndof+8:ndof+10,:);
            omega_W = chi(ndof+11:ndof+13,:);
            dqj = chi(ndof+14:2*ndof+13,:);

            v = [dx_b;omega_W;dqj];

            %% MexWholeBodyModel calls


            %wbm_updateState(qj,dqj,[dx_b;omega_W]);
            obj.setState(qj, dqj, [dx_b;omega_W]);


            %reconstructing rotation of root to world from the quaternion
            [~,T_b,~,~] = wbm_getState();
            %T_b = obj.stvqT;

            qt_b_mod_s = T_b(4);
            qt_b_mod_r = T_b(5:end);
            R_b = eye(3) - 2*qt_b_mod_s*skew(qt_b_mod_r) + 2 * skew(qt_b_mod_r)^2;
            p_b = T_b(1:3);
            %[p_b, R_b] = WBM.utilities.frame2posRotm(T_b);


            M = wbm_massMatrix();
            h = wbm_generalisedBiasForces();
            % M = obj.massMatrix();
            % h = obj.generalBiasForces();

            hDash = wbm_generalisedBiasForces(R_b,p_b,qj,dqj,[dx_b;omega_W]);
            g = wbm_generalisedBiasForces(R_b,p_b,qj,zeros(size(qj)),zeros(6,1));
            % hDash = obj.generalBiasForces(R_b, p_b, qj, dqj, [dx_b;omega_W]);
            % g = obj.generalBiasForces(R_b, p_b, qj, zeros(size(qj)), zeros(6,1));

            %% Building up contraints jacobian and djdq
            %numConstraints = length(param.constraintLinkNames);
            nCstrs = obj.iwbm_config.nCstrs;
            Jc = zeros(6*nCstrs,6+ndof);
            dJcDq = zeros(6*nCstrs,1);
            for i=1:nCstrs
                Jc(6*(i-1)+1:6*i,:) = wbm_jacobian(obj.iwbm_config.cstrLinkNames{i});
                dJcDq(6*(i-1)+1:6*i,:) = wbm_djdq(obj.iwbm_config.cstrLinkNames{i});
                % Jc(6*(i-1)+1:6*i,:) = obj.jacobian(obj.iwbm_config.cstrLinkNames{i});
                % dJcDq(6*(i-1)+1:6*i,:) = obj.dJdq(obj.iwbm_config.cstrLinkNames{i});
            end


            %% control torque
            %tau = param.tau(t);
            tau = ctrlTrqs.tau(t);


            %% Contact forces computation
            JcMinv = Jc/M;
            JcMinvJct = JcMinv * Jc';   

            %tauDamp = -param.dampingCoeff*dqj;
            dampCoeff = obj.iwbm_config.dampCoeff;
            tauDamp = -dampCoeff * dqj;
  
            temp = JcMinv*h;
            temp2 = JcMinvJct\(JcMinv*h);

            fc = (JcMinvJct)\(JcMinv*(h-[zeros(6,1);tau+tauDamp])-dJcDq);

            % need to apply root-to-world rotation to the spatial angular velocity omega_W to
            % obtain angular velocity in body frame omega_b. This is then used in the
            % quaternion derivative computation.

            omega_b = R_b*omega_W;% R_b*omega_W;
            dqt_b = quaternionDerivative(omega_b, qt_b);%,param.QuaternionDerivativeParam);

            dx = [dx_b;dqt_b;dqj];
            dv = M\(Jc'*fc + [zeros(6,1); tau+tauDamp]-h);
            dchi = [dx;dv];  
            %kinEnergy = 0.5*v'*M*v;
            %dchi = zeros(size(dchi));
        end

        visualizeForwardDynamics(obj, x_out, tspan, sim_config)

        setupSimulation(sim_config)

        % function plotSimulationResults(@simFunc, x_out, tspan, sim_config)

        % end
        
        function stParams = getStateParams(obj, stvChi)
            if ~iscolumn(stvChi)
               error('WBM::getStateParams: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            ndof = obj.iwbm_config.ndof;
            stvLen = obj.iwbm_config.stvLen;
            stParams = obj.initStateParams();

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

            ndof = obj.iwbm_config.ndof;
            stParams = obj.initStateParamsMatrices(m);

            % extract all values ...
            stParams.x_b  = chi(1:m,1:3);
            stParams.qt_b = chi(1:m,4:7);
            stParams.q_j  = chi(1:m,8:ndof+7);
            
            stParams.dx_b    = chi(1:m,ndof+8:ndof+10);
            stParams.omega_b = chi(1:m,ndof+11:ndof+13);           
            stParams.dq_j    = chi(1:m,ndof+14:stvLen);
        end

        function stvPos = getStatePositions(obj, stvChi)
            if ~iscolumn(stvChi)
               error('WBM::getStatePositions: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            cutp = obj.iwbm_config.ndof + 7;

            % extract the base VQS-Transformation (without S)
            % and the joint positions ...
            stvPos = zeros(cutp,1);
            stvPos = stvChi(1:cutp,1); % [x_b; qt_b; q_j]
        end

        function stmPos = getStatePositionsData(obj, chi)
            stvLen = obj.iwbm_config.stvLen;

            [m, n] = size(chi);
            if (n ~= stvLen) 
                error('WBM::getStatePositionsData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            cutp = obj.iwbm_config.ndof + 7;

            stmPos = zeros(m,cutp);
            stmPos = chi(1:m,1:cutp); % m -by- [x_b, qt_b, q_j]
        end

        function stvVel = getStateVelocities(obj, stvChi)
            if ~iscolumn(stvChi)
               error('WBM::getStateVelocities: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end

            stvLen = obj.iwbm_config.stvLen;
            cutp = obj.iwbm_config.ndof + 8;
            len = stvLen - cutp;

            % extract the velocites ...
            stvVel = zeros(len,1);
            stvVel = stvChi(cutp:stvLen,1); % [dx_b; omega_b; dq_j]
        end

        function stmVel = getStateVelocitiesData(obj, chi)
            stvLen = obj.iwbm_config.stvLen;

            [m, n] = size(chi);
            if (n ~= stvLen) 
                error('WBM::getStateVelocitiesData: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end

            cutp = obj.iwbm_config.ndof + 8;
            len = stvLen - cutp;

            stmVel = zeros(m,len);
            stmVel = chi(1:m,cutp:stvLen); % m -by- [dx_b, omega_b, dq_j]
        end
        
        function vqT = getRototranslation(obj, stParams)
            if isempty(stParams)
                error('WBM::getRototranslation: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end

            vqT = zeros(7,1);
            vqT(1:3,1) = stParams.x_b;
            vqT(4:7,1) = stParams.qt_b;
        end

        function stvChi = toStateVector(obj, stParams)
            if isempty(stParams)
                error('WBM::toStateVector: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end

            vqT_b  = [stParams.x_b; stParams.qt_b];
            stvChi = [vqT_b; stParams.q_j; stParams.dx_b; stParams.omega_b; stParams.dq_j]; % optimieren??
        end

        function stvChiInit = get.stvChiInit(obj)
            stInit     = obj.iwbm_config.initStateParams;
            vqT_b      = [stInit.x_b; stInit.qt_b];
            stvChiInit = [vqT_b; stInit.q_j; stInit.dx_b; stInit.omega_b; stInit.dq_j]; % optimieren??
        end

        function vqTInit = get.vqTInit(obj)
            stInit = obj.iwbm_config.initStateParams;

            vqTInit = zeros(7,1);
            vqTInit(1:3,1) = stInit.x_b;
            vqTInit(4:7,1) = stInit.qt_b;
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
            
            cellInitSt{1} = sprintf('  q_j:      %s\n', mat2str(obj.iwbm_config.initState.q_j, prec));
            cellInitSt{2} = sprintf('  dq_j:     %s\n', mat2str(obj.iwbm_config.initState.dq_j, prec));
            cellInitSt{3} = sprintf('  x_b:      %s\n', mat2str(obj.iwbm_config.initState.x_b, prec));
            cellInitSt{4} = sprintf('  qt_b:     %s\n', mat2str(obj.iwbm_config.initState.qt_b, prec));
            cellInitSt{5} = sprintf('  dx_b:     %s\n', mat2str(obj.iwbm_config.initState.dx_b, prec));
            cellInitSt{6} = sprintf('  omega_b:  %s\n', mat2str(obj.iwbm_config.initState.omega_b, prec));
            strInitState = strcat(cellInitSt{1}, cellInitSt{2}, cellInitSt{3}, ...
                                  cellInitSt{4}, cellInitSt{5}, cellInitSt{6});
                              
            strConfig = sprintf(['Robot configuration:\n\n' ...
                                 ' NDOFs:         %d\n' ...
                                 ' # constraints: %d\n\n' ...
                                 ' Constraint link names:\n\n%s\n' ...
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
            if (len ~= obj.iwbm_config.stvLen)
                if (len ~= (obj.iwbm_config.stvLen - 7)) % length without x_b & qt_b (they will be updated)
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
            stParams = wbmStateParams;

            stParams.x_b  = zeros(1:m,3);
            stParams.qt_b = zeros(1:m,4);
            stParams.q_j  = zeros(1:m,obj.iwbm_config.ndof);
            
            stParams.dx_b    = zeros(1:m,3);
            stParams.omega_b = zeros(1:m,3);
            stParams.dq_j    = zeros(1:m,obj.iwbm_config.ndof);
        end
                        
    end
end
