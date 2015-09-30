classdef WBM < WBMBasic
    properties(Access = private)
        wb_config@wbmBasicRobotConfig
    end
    
    properties(Access = private, Constant)
       wb_strWrongMatDimErr = 'Wrong matrix dimension!';
    end
    
    methods(Access = public)
        % Constructor:
        function obj = WBM(wbm_params, robot_config, init_wf_ctrl)
            if (nargin < 2)
                error('WBM::WBM: %s', obj.wb_strWrongArgErr);
            end
            % call the constructor of the superclass ...
            if ~exist('init_wf_ctrl', 'var')
                % use the default WF control value ...
                obj = obj@WBMBasic(wbm_params);
            else
                obj = obj@WBMBasic(wbm_params, init_wf_ctrl);
            end
             
            initConfig(robot_config);
            setState(obj.wb_config.initState.q_j, obj.wb_config.initState.dq_j, ...
                     obj.wb_config.initState.dx_b, obj.wb_config.initState.omega_b);            
        end
        
        function newObj = copy(obj)
            newObj = copy@WBMBasic(obj);
        end
        
        function u_quat = axisAngle2UnitQuat(axang)
            if (size(axang) ~= 4)
                error('WBM::axisAngle2UnitQuat: %s', obj.wb_strVecSizeErr);
            end
            theta = axang(4);
            q = [cos(theta/2); sin(theta/2).*axang(1:3)];
            u_quat = q./norm(q);
        end
        
        forwardDynamics()
        
        forwardDynamicsZeroExtForces()
        
        visualizeForwardDynamics()

        function wbm_config = getWBMConfig(obj)
            wbm_config = obj.wb_config;
        end
        
        function dispWBMConfig(obj, precision)
            if ~exist('precision', 'var')
                precision = 2;
            end
            %strConfig = sprintf();

        end
        
    end
    
    methods(Access = private)
        function initConfig(obj, robot_config)
            % check robot_config is an instance of a class that
            % is derived from the class "wbmBasicRobotConfig" ...
            if ~isa(robot_config, 'wbmBasicRobotConfig')
                error('WBM::initWBM: %s', obj.wb_strDataTypeErr);
            end
            
            obj.wb_config = wbmBasicRobotConfig;
            obj.wb_config.ndof = robot_config.ndof;
            obj.wb_config.nCstrs = robot_config.nCstrs;
            obj.wb_config.cstrLinkNames = robot_config.cstrLinkNames;
            obj.wb_config.dampCoeff = robot_config.dampCoeff;
            obj.wb_config.initState = robot_config.initState;
            
            % convert all angle-values of the joint positions
            % from degrees to radians ... 
            obj.wb_config.initState.q_j = obj.wb_config.initState.q_j * (pi/180.0);
        end
        
        function dquat = quatDerivative(q, omega)
            K = 1;
            omegaCross = [0 -omega'; omega -skew(omega)];
            dquat = 0.5*omegaCross*q + K*(1 - norm(q))*q;
        end
        
        plotQuat(q)
                        
        function [pos, R] = frame2posRot(qT)
            if (size(qT) ~= 7)
               error('WBM::frame2posRot: %s', obj.wb_strVecSizeErr);
            end
            pos = qT(1:3); % cartesian postion
            qt_b_mod_s = qT(4); % scalar/real part
            qt_b_mod_r = qT(5:end); % (imaginary) vector part
            
            % calculate the Direction Cosine Matrix (DCM):
            %R = zeros(3);
            R = eye(3) - 2*qt_b_mod_s*skew(qt_b_mod_r) + 2*skew(qt_b_mod_r)^2;            
            %R = quat2dcm(qt_b_mod_r)   % maybe it is better when we use
                                        % there the buildin-method of matlab ...            
        end
        
        function R = quat2rot(q)
           R = quat2dcm(q);
        end
                
        function X = skew(x)
            X = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];            
        end
        
    end
end
