function [CFoot,dConstraints] = frictionCones(staticFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,footSize,fZmin)
%% frictionCones
%  Computes friction cones contraints. Approximation with straight lines.
%
%  outputs: constraints matrix and constraints vector for QP programming
%
%% Setup parameters
% split the pi/2 angle into numberOfPoints - 1;
segmentAngle = pi/2 / (numberOfPoints - 1);

% define angle
angle  = 0 : segmentAngle : (2 * pi - segmentAngle);
points = [cos(angle); sin(angle)];

numberOfEquations       = size(points, 2);
assert(size(points, 2) == (4 * (numberOfPoints - 2) + 4));

% Aineq*x <= b, with b is all zeros.
Aineq = zeros(numberOfEquations, 6);

% define equations
for i = 1 : numberOfEquations
   firstPoint  = points(:, i);
   secondPoint = points(:, rem(i, numberOfEquations) + 1);
   
   % define line passing through the above points
   angularCoefficients = (secondPoint(2) - firstPoint(2)) / (secondPoint(1) - firstPoint(1));
   offsets = firstPoint(2) - angularCoefficients * firstPoint(1);
   inequalityFactor = +1;
   
   % if any of the two points are between pi and 2pi, then the inequality is
   % in the form of y >= m*x + q, and I need to change the sign of it.

   if (angle(i) > pi || angle(rem(i, numberOfEquations) + 1) > pi)
       inequalityFactor = -1;
   end
   
   % a force is 6 dimensional f = [fx, fy, fz, mux, muy, muz]'
   % I have constraints on fy and fz, and the offset will be multiplied by
   % mu * fx
   Aineq(i,:) = inequalityFactor .*  [-angularCoefficients, 1, -offsets * staticFrictionCoefficient, 0, 0, 0];

end

CFoot = [ 0         , 0, -torsionalFrictionCoefficient,               0,               0, 1; 
          0         , 0, -torsionalFrictionCoefficient,               0,               0,-1; 
          0         , 0,                              -1,             0,               0, 0;
          0         , 0,                   footSize(1,1),             0,               1, 0; 
          0         , 0,                  -footSize(1,2),             0,              -1, 0; 
          0         , 0,                   footSize(2,1),            -1,               0, 0; 
          0         , 0,                  -footSize(2,2),             1,               0, 0];

CFoot       =  [ Aineq ;
                 CFoot ];     
      
dConstraints = [zeros(size(Aineq,1), 1); zeros(7,1)];

dConstraints(3 + size(Aineq,1)) = -fZmin; 

end
