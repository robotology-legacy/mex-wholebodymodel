%% gainsAndConstraints
% generates the desired initial gains for both the joint and the CoM
% dynamics. It also generates the friction cones at feet to define the constraints
% for the QP solver.
% The output are:
%
%  gains            this is a structure which contains both CoM and joints gains
%
%  constraints      this is also a structure and contains the constraints for QP
%                   solver
%
%  trajectory       this structure contains all the parameters needed for
%                   CoM trajectory definition
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function [gains, constraints, trajectory] = gainsAndConstraints(params)
%% General parameters
ndof                                  = params.ndof;
trajectory.directionOfOscillation     = [0;0;0];
trajectory.referenceParams            = [0.0 0.0];   %referenceParams(1) = amplitude of ascillations in meters 
                                                     %referenceParams(2) = frequency of ascillations in Hertz

trajectory.noOscillationTime          = 0;           % If params.demo_movements = 1, the variable noOscillationTime is the time, in seconds, 
                                                     % that the robot waits before starting the left-and-right

%% Pameters for 2 feet on the ground
if sum(params.feet_on_ground) == 2
    
   gains.gainsPCoM               = diag([ 40  40  40]);
   gains.gainsDCoM               = 2*sqrt(gains.gainsPCoM);
   gains.gainsPAngMom            = diag([5 5 5]);
   gains.gainsDAngMom            = 2*sqrt(gains.gainsPAngMom);

% impedances acting in the null space of the desired contact forces 
    impTorso            = [ 40  40  40]; 
    impArms             = [ 10  10  10   5  5];
    impLeftLeg          = [ 35  40   10  30   5  10]; 
    impRightLeg         = [ 35  40   10  30   5  10]; 
    
 % desired CoM trajectory   
    if (params.demo_movements == 1)
        
        trajectory.directionOfOscillation = [0;1;0];
        trajectory.referenceParams        = [0.035 0.35];     %referenceParams(1) = amplitude of ascillations in meters
    end
    
end

%% Parameters for 1 foot on the ground
if  sum(params.feet_on_ground) == 1
 
    gains.gainsPCoM                 = diag([40 45 40]);
    gains.gainsDCoM                 = 2*sqrt(gains.gainsPCoM);
    gains.gainsPAngMom              = diag([5 5 5]);
    gains.gainsDAngMom              = 2*sqrt(gains.gainsPAngMom);
   
% impedances acting in the null space of the desired contact forces 
     impTorso            = [ 20   20   20]; 
     impArms             = [ 15  15   45   5   5];

if params.feet_on_ground(1) == 1
    
     impLeftLeg          = [ 70   70  65  30  10  10];  
     impRightLeg         = [ 20   20  20  10  10  10];   
else
     impRightLeg         = [ 70   70  65  30  10  10]; 
     impLeftLeg          = [ 20   20  20  10  10  10];
end

% desired CoM trajectory
    if (params.demo_movements == 1)
        
        trajectory.directionOfOscillation = [0;1;0];
        trajectory.referenceParams        = [0.0075 0.1];     %referenceParams(1) = amplitude of ascillations in meters
    end

end

%% Definition of the impedances and dampings vectors 
gains.impedances    = [impTorso,impArms,impArms,impLeftLeg,impRightLeg];
gains.dampings      = 2*sqrt(gains.impedances);

if (size(gains.impedances,2) ~= ndof)
    
  error('Dimension mismatch between ndof and dimension of the variable impedences. Check these variables in the file gains.m');    
end

%% Desired shape for the state matrix of the linearized system, for gains tuning
gains.KSdes = diag(gains.impedances);
gains.KDdes = diag(gains.dampings);

%% Constraints for QP for balancing on both feet -friction cone -z-moment -in terms of fc (not f0)
% friction cone parameters
numberOfPoints               = 4;   % The friction cone is approximated by using linear interpolation of the circle. 
                                    % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 
forceFrictionCoefficient     = 1;              
torsionalFrictionCoefficient = 2/150;
constraints.footSize         = [-0.1 0.1;       % xMin, xMax
                                -0.1 0.1];      % yMin, yMax    
fZmin                        = 10;

% the QP solver will search a solution f0 that satisfies the inequality Aineq_f F(fo) < bineq_f 
[constraints.ConstraintsMatrix,constraints.bVectorConstraints] = frictionCones(forceFrictionCoefficient,numberOfPoints,...
                                                                               torsionalFrictionCoefficient,constraints.footSize,fZmin);

%% Generate a new postural task to assure system's stability
% parameters definition
M         = params.MInit;
Jc        = params.JcInit;
S         = [zeros(6,ndof);
             eye(ndof,ndof)];
JcMinv    = Jc/M;
JcMinvS   = JcMinv*S;

% damped null space
Mbar                = M(7:end,7:end)-M(7:end,1:6)/M(1:6,1:6)*M(1:6,7:end);
pinvJcMinvS_damp    = JcMinvS'/(JcMinvS*JcMinvS' + params.pinv_damp*eye(size(JcMinvS,1)));
NullLambda_damp     = eye(ndof)-pinvJcMinvS_damp*JcMinvS;

% new postural gains. They are define to match as close as possible the
% previous dynamics
impedances = diag(gains.impedances)*pinv(NullLambda_damp*Mbar,params.pinv_tol);
dampings   = diag(gains.dampings)*pinv(NullLambda_damp*Mbar,params.pinv_tol);

% add a regulation term to both the impedance and damping, to assure the matrices
% are positive definite
gains.impedances       = impedances + 0.1.*eye(ndof);
gains.dampings         = dampings   + 0.1.*eye(ndof);

% this is the term added to the postural task to assure the dynamics to be
% asymptotically stable
gains.posturalCorr     = NullLambda_damp*Mbar;

end
