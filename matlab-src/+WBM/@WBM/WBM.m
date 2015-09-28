classdef WBM < WBMBasic
    properties(Access = private)
        wb_config@wbmBasicRobotConfig
    end
    
    properties(Access = private, Constant)
       wb_strVecSizeErr  = 'Wrong vector size!';
       wb_strWrongDimErr = 'Wrong matrix dimension!';
    end
    
    methods(Access = public)
        % Constructor:
        function obj = WBM(wbm_params, robot_config)
            initWBM(wbm_params, robot_config);            
        end
        
        function newObj = copy()
            
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

        function wb_config = getWBMConfig(obj)
            wb_config = obj.wb_config;
        end
        
        function dispWBMConfig(obj, precision)
            if ~exist('precision', 'var')
                precision = 2;
            end
            %strConfig = sprintf();

        end
        
    end
    
    methods(Access = private)
        function initWBM(wbm_params, robot_config)
            obj.wb_config.ndof = 25;
            
            
        end
        
        function dquat = quatDerivative(q, omega)
            K = 1;
            omegaCross  = [0 -omega'; omega -skew(omega)];
            dquat = 0.5*omegaCross*q + K*(1 - norm(q))*q;
        end
        
        plotQuat(q)
                        
        function [pos, dcm] = frame2posRot(qT)
            if (size(qT) ~= 7)
               error('WBM::frame2posRot: %s', obj.wb_strVecSizeErr);
            end
            pos = qT(1:3); % cartesian postion
            qt_b_mod_s = qT(4); % real/scalar part
            qt_b_mod_r = qT(5:end); % imaginary part
            
            % calculate the Direction Cosine Matrix (DCM):
            dcm = zeros(3, 3);
            dcm = eye(3) - 2*qt_b_mod_s*skew(qt_b_mod_r) + 2*skew(qt_b_mod_r)^2;   
            
            %dcm = angle2dcm(yaw, pitch, roll, 'ZYX') % maybe it is better when
            %we use there the buildin-method of matlab ...
        end
                
        function X = skew(x)
            X = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
        end
        
    end
end
