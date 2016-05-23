function centroidal = centroidalConversion(dynamics,forKinematics)
%CENTROIDALCONVERSION applies the centroidal tranformation to the system,
%                     i.e. the base dynamics will be coincident with the
%                     CoM dynamics. 
%           
%           centroidal = CENTROIDALCONVERSION(dynamics,forKinematics) takes 
%           as an input the structure DYNAMICS, which contains the robot 
%           dynamics and FORKINEMATICS, which contains the robot forward
%           kinematics.
%           The output is the structure CENTROIDAL which contains the robot
%           dynamics converted to the centroidal coordinates.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% Dynamics parameters
M       = dynamics.M;
h       = dynamics.h;
g       = dynamics.g;
Jc      = dynamics.Jc;
dJcNu   = dynamics.dJcNu;

% ForKin parameters
xCoM     = forKinematics.xCoM;
VelBase  = forKinematics.VelBase;
PosBase  = forKinematics.PosBase;
dxCoM    = forKinematics.dxCoM;
Nu       = forKinematics.Nu;

%% CENTROIDAL TRANSFORMATION
[T,dT]                                 = centroidalTransformationT_TDot(xCoM,PosBase,dxCoM,VelBase,M);
[MJS, CNuJS, gJS, JcJS, dJcNuJS, NuJS] = fromFloatingToCentroidalDynamics(M, h, g, Jc, dJcNu, Nu, T, dT);

centroidal.M                           = MJS;
centroidal.g                           = gJS;
centroidal.CNu                         = CNuJS;
centroidal.dJcNu                       = dJcNuJS;
centroidal.Nu                          = NuJS;
centroidal.Jc                          = JcJS;

end

