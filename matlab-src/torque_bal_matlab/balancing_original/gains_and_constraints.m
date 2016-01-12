function [gainParam, constraintParam, trajParam] = gains_and_constraints(param)
%% gains_and_constraints
%  Generates the desired gains for both the joint and the CoM
%  dynamics. It also generates the friction cones at feet. 
ndof                       = param.ndof;
Contact_constraints        = param.numConstraints;
Left_and_Right_Demo        = param.demo_left_and_right;

trajParam.directionOfOscillation     = [0;0;0];
trajParam.referenceParams            = [0.0 0.0];   %referenceParams(1) = amplitude of ascillations in meters 
                                                    %referenceParams(2) = frequency of ascillations in Hertz

trajParam.noOscillationTime          = 0;           % If DEMO_LEFT_AND_RIGHT = 1, the variable noOscillationTime is the time, in seconds, 
                                                    % that the robot waits before starting the left-and-righ
                                           
qTildeMax                  = 20*pi/180;

%% PARAMETERS FOR TWO FEET ON THE GROUND
if Contact_constraints == 2
    
   gainParam.gainsPCOM               = diag([ 50   50  50]);
   gainParam.gainsDCOM               = 2*sqrt(gainParam.gainsPCOM);
   gainParam.gainMomentum            = 1;

% impedances acting in the null space of the desired contact forces 
    impTorso            = [ 50  50  50
                             0   0   0]; 
                           
    impArms             = [ 10  10  10  10  5  
                             0   0   0   0  0];
                        
    impLeftLeg          = [ 35  50   1  30   2  10
                             0   0   0   0   0   0]; 

    impRightLeg         = [35  50    1   30   2  10
                            0   0    0    0   0   0]; 
                        
    if (Left_and_Right_Demo == 1)
        
        trajParam.directionOfOscillation = [0;1;0];
        trajParam.referenceParams        = [0.035 0.35];     %referenceParams(1) = amplitude of ascillations in meters
    
    end
    
end

%% PARAMETERS FOR ONLY ONE FOOT ON THE GROUND
if  Contact_constraints == 1
    
    gainParam.gainsPCOM                 = diag([120  140 120])/3;
    gainParam.gainsDCOM                 = diag([  1    1   1]);
    gainParam.gainMomentum              = 1 ;

% impedances acting in the null space of the desired contact forces 
     impTorso            = [ 20   20   20
                              0    0    0]; 

     impArms             = [ 13  13   13   5   5
                              0   0    0   0   0 ];

     impLeftLeg          = [ 70   70  65  30  10  10
                              0    0   0   0   0   0]; 

     impRightLeg         = [ 20   20  20  10  10  10
                              0    0   0   0   0   0];
                          
end

%% Definition of impedances and dampings vectors
  impedances             = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
   
  gainParam.dampings     = 0.5*ones(1,ndof);

  increasingRatesImp     = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];

if (size(impedances,2) ~= ndof)
    
  error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
    
end

%% Constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)
% Friction cone parameters
numberOfPoints               = 4;   % The friction cone is approximated by using linear interpolation of the circle. 
                                    % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;                %1/3;  
torsionalFrictionCoefficient = 2/150;

footSize                     = [ -0.1 0.1   ;    % xMin, xMax
                                 -0.1 0.1  ];    % yMin, yMax    

fZmin                        = 10;

% The QP solver will search a solution f0 that 
% satisfies the inequality Aineq_f F(fo) < bineq_f 
[constraintParam.ConstraintsMatrix,constraintParam.bVectorConstraints] = constraints(forceFrictionCoefficient,numberOfPoints,...
                                                                                     torsionalFrictionCoefficient,footSize,fZmin);
constraintParam.footSize = footSize;

%% Impedances correction with a nonlinear term
gainParam.impedances = nonLinImp(param.qjInit,param.qj,impedances,increasingRatesImp,qTildeMax);
 
end
