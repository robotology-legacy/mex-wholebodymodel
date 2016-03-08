function [gains, constraints, trajectory] = gainsAndConstraints_SoT(param)
%% gainsAndConstraints_SoT
%  Generates the desired gains for both the joint and the CoM
%  dynamics. It also generates the friction cones at feet.
%  The output are:
%
%  gains            this is a structure which contains both CoM and joints gains
%
%  constraints      this is also a structure and contains the constraints for QP
%                   solver
%
%  trajectory       this structure contains all the parameters needed for
%                   CoM trajectory definition
%
%% General parameters
ndof                       = param.ndof;
Contact_constraints        = param.numConstraints;
demo_movements             = param.demo_movements;

trajectory.directionOfOscillation     = [0;0;0];
trajectory.referenceParams            = [0.0 0.0];   %referenceParams(1) = amplitude of ascillations in meters 
                                                     %referenceParams(2) = frequency of ascillations in Hertz

trajectory.noOscillationTime          = 0;           % If DEMO_LEFT_AND_RIGHT = 1, the variable noOscillationTime is the time, in seconds, 
                                                     % that the robot waits before starting the left-and-righ
                                            
qTildeMax                             = 20*pi/180;

%% Pameters for 2 feet on the ground
if Contact_constraints == 2
    
   gains.gainsPCoM               = diag([ 50  50  50]);
   gains.gainsDCoM               = 2*sqrt(gains.gainsPCoM);
   gains.gainPhi                 = 5;
   gains.gainMomentum            = 2*sqrt(gains.gainPhi);

% impedances acting in the null space of the desired contact forces 
    impTorso            = [ 50  50  50
                             0   0   0]; 
                           
    impArms             = [ 10  10  10  10  5  
                             0   0   0   0  0];
                        
    impLeftLeg          = [ 35  50   10  30   5  10
                             0   0   0    0   0   0]; 

    impRightLeg         = [35  50    10  30   5  10
                            0   0    0    0   0   0]; 
                        
    if (demo_movements == 1)
        
        trajectory.directionOfOscillation = [0;1;0];
        trajectory.referenceParams        = [0.035 0.35];     %referenceParams(1) = amplitude of ascillations in meters

    end
    
end

%% Parameters for 1 foot on the ground
if  Contact_constraints == 1
    
    gains.gainsPCoM                 = diag([40  45 40]);
    gains.gainsDCoM                 = 2*sqrt(gains.gainsPCoM);
    gains.gainPhi                   = 5;
    gains.gainMomentum              = 2*sqrt(gains.gainPhi);
   
% impedances acting in the null space of the desired contact forces 
     impTorso            = [ 20   20   20
                              0    0    0]; 

     impArms             = [ 15  15   15   5   5
                              0   0    0   0   0 ];

if param.feet_on_ground(1) == 1
    
     impLeftLeg          = [ 70   70  65  30  10  10
                              0    0   0   0   0   0]; 

     impRightLeg         = [ 20   20  20  10  10  10
                              0    0   0   0   0   0];
    
else
   
     impRightLeg         = [ 70   70  65  30  10  10
                              0    0   0   0   0   0]; 

     impLeftLeg          = [ 20   20  20  10  10  10
                              0    0   0   0   0   0];

end

    if (demo_movements == 1)
        
        trajectory.directionOfOscillation = [0;1;0];
        trajectory.referenceParams        = [0.05 0.3];     %referenceParams(1) = amplitude of ascillations in meters
    
    end

end

%% Definition of impedances and dampings vectors

  impedances             = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)]/20;
   
  gains.dampings         = 0.5*ones(1,ndof)/4;

  increasingRatesImp     = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];

if (size(impedances,2) ~= ndof)
    
  error('Dimension mismatch between ndof and dimension of the variable impedences. Check these variables in the file gains.m');
    
end

%% Constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0)
% Friction cone parameters
numberOfPoints               = 4;   % The friction cone is approximated by using linear interpolation of the circle. 
                                    % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;              
torsionalFrictionCoefficient = 2/150;

footSize                     = [ -0.1 0.1;       % xMin, xMax
                                 -0.1 0.1  ];    % yMin, yMax    

fZmin                        = 10;

% The QP solver will search a solution f0 that satisfies the inequality Aineq_f F(fo) < bineq_f 
[constraints.ConstraintsMatrix,constraints.bVectorConstraints] = frictionCones(forceFrictionCoefficient,numberOfPoints,...
                                                                               torsionalFrictionCoefficient,footSize,fZmin);
constraints.footSize = footSize;

%% Impedances correction with a nonlinear term
gains.impedances     = nonLinImp(param.qjInit,param.qj,impedances,increasingRatesImp,qTildeMax);

%% New postural task 

% old postural
gains.impedances_old = gains.impedances;
gains.dampings_old   = gains.dampings;

% parameters definition
M         = param.M0;
Jc        = param.Jc0;
pinv_tol  = 1e-8;

S         = [ zeros(6,ndof);
              eye(ndof,ndof)];

JcMinv         = Jc/M;
JcMinvS        = JcMinv*S;

% damped null space
Mbar                = M(7:end,7:end)-M(7:end,1:6)/M(1:6,1:6)*M(1:6,7:end);
reg                 = 5e-6;
Pinv_JcMinvS_damp   = JcMinvS'/(JcMinvS*JcMinvS' + reg*eye(size(JcMinvS,1)));
NullLambda_damp     = eye(ndof)-Pinv_JcMinvS_damp*JcMinvS;

% new postural task
impedances = diag(gains.impedances)*pinv(NullLambda_damp*Mbar,pinv_tol);
dampings   = diag(gains.dampings)*pinv(NullLambda_damp*Mbar,pinv_tol);

gains.impedances = impedances +0.1*eye(ndof);
gains.dampings   = dampings   +0.1*eye(ndof);

gains.GainCorr_imp     = NullLambda_damp*Mbar;
gains.GainCorr_damp    = NullLambda_damp*Mbar;

% Added for IROS
% save('gains','gains')

end
