function [gainsPCOM, gainsDCOM, gainMomentum, impedances, dampings, referenceParams, directionOfOscillation, noOscillationTime,...
          forceFrictionCoefficient, numberOfPoints, torsionalFrictionCoefficient,...
          footSize, fZmin, increasingRatesImp, qTildeMax] = gains (DOF, LEFT_RIGHT_FOOT_IN_CONTACT, DEMO_LEFT_AND_RIGHT)


directionOfOscillation            = [0; 0; 0];
referenceParams                   = [0.0  0.0];  %referenceParams(1) = amplitude of ascillations in meters 
                                                 %referenceParams(2) = frequency of ascillations in hertz

noOscillationTime                 = 0;     % If DEMO_LEFT_AND_RIGHT = 1, the variable noOscillationTime is the time, in seconds, 
                                           % that the robot waits before starting the left-and-righ
                                           
qTildeMax                         = 20*pi/180;

%% PARAMETERS FOR TWO FEET ON THE GROUND
if LEFT_RIGHT_FOOT_IN_CONTACT == 2
    
    gainsPCOM                 = diag([ 50   50  50]);
    gainsDCOM                 = 2*sqrt(gainsPCOM);

    gainMomentum              = 1;

% Impadances acting in the null space of the desired contact forces 
     
%      impTorso            = [   60    60   10
%                                 0     0    0];
% 
%      impArms             = [8    8    8   12  5  
%                             0   0    0    0   0];
%                                                 
%      impLeftLeg          = [ 35   20    30   350  550   0
%                               0    0     0     0    0   0]; 
% 
%      impRightLeg         = [35   20    30    350   550   0
%                              0    0     0      0    0   0];                             

       impTorso            = [ 50    50   50
                                0     0    0]; 
                           
       impArms             = [ 10    10    10   10  5  
                                0     0    0    0   0];
                        
       impLeftLeg          = [ 35   50    0.1   30   2   10
                                0    0     0     0   0    0]; 

       impRightLeg         = [35   50    0.1    30   2   10
                               0    0     0      0   0    0]; 
                        
                         
    if (DEMO_LEFT_AND_RIGHT == 1)
        
        directionOfOscillation = [0;1;0];
        referenceParams        = [0.035 0.35];     %referenceParams(1) = amplitude of ascillations in meters
    
    end
    
end

%% PARAMETERS FOR ONLY ONE FOOT ON THE GROUND
if  LEFT_RIGHT_FOOT_IN_CONTACT == 1
    
    gainsPCOM                 = diag([50 50 50]);
    gainsDCOM                 = diag([  1    1   1]);

    gainMomentum              = 1 ;

% Impedances acting in the null space of the desired contact forces 
    
%     impTorso            = [  20    20   20
%                               0     0    0]; 
% 
%     impArms             = [ 13  13   13   5   5
%                              0    0    0   0   0 ];
% 
%     impLeftLeg          = [ 70   70  65  30   0   0
%                              0    0   0   0   0   0]; 
% 
%     impRightLeg         = [ 20   20  20  10   0  0
%                              0    0   0   0   0  0];    


      impTorso            = [  20    20   20
                                0     0    0]; 

      impArms             = [ 13  13   13   5   5
                               0    0    0   0   0 ];

      impLeftLeg          = [ 70   70  65  30  10  10
                               0    0   0   0   0   0]; 

      impRightLeg         = [ 20   20  20  10  10  10
                               0    0   0   0   0   0];
                           
                           
    if (DEMO_LEFT_AND_RIGHT == 1)
        
        directionOfOscillation = [1;1;1];
        referenceParams        = [0.00025 0.2];     %referenceParams(1) = amplitude of ascillations in meters
    
    end
                           

end

%% other calculations
  impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
   
  dampings            = 1*ones(1,DOF);

  increasingRatesImp  = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];

if (size(impedances,2) ~= DOF)
    
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
    
end

%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)
% Friction cone parameters
numberOfPoints               = 4;  % The friction cone is approximated by using linear interpolation of the circle. 
                                   % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;                %1/3;  
torsionalFrictionCoefficient = 2/150;

footSize                     = [ -0.1 0.1   ;    % xMin, xMax
                                 -0.1 0.1  ];    % yMin, yMax    

fZmin                        = 10;

% The QP solver will search a solution fo that 
% satisfies the inequality Aineq_f F(fo) < bineq_f 

end
