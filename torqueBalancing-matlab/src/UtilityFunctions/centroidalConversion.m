function centroidalDyn = centroidalConversion(DYNAMICS,FORKINEMATICS,STATE)
%CENTROIDALCONVERSION applies the centroidal coordinates tranformation to the 
%                     system, i.e. the base dynamics now coincides with the
%                     CoM dynamics. 
%           
%           centroidal = CENTROIDALCONVERSION(dynamics,forKinematics,state) 
%           takes as an input the structures containing the robot dynamics,
%           forward kinematics and state.
%           The output is the structure CENTROIDALDYN which contains the robot
%           dynamics converted into the centroidal coordinates.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
%% Dynamics
M       = DYNAMICS.M;
h       = DYNAMICS.h;
g       = DYNAMICS.g;
Jc      = DYNAMICS.Jc;
dJcNu   = DYNAMICS.dJcNu;

%% Forward kinematics
xCoM     = FORKINEMATICS.xCoM;
dxCoM    = FORKINEMATICS.dxCoM;

%% Robot state
Nu       = STATE.Nu;
VelBase  = STATE.VelBase;
PosBase  = STATE.PosBase;

%% CENTROIDAL TRANSFORMATION
[T,dT]                                 = centroidalTransformationT_TDot(xCoM,PosBase,dxCoM,VelBase,M);
[M_c, CNu_c, g_c, Jc_c, dJcNu_c, Nu_c] = fromFloatingToCentroidalDynamics(M, h, g, Jc, dJcNu, Nu, T, dT);

centroidalDyn.M                           = M_c;
centroidalDyn.g                           = g_c;
centroidalDyn.CNu                         = CNu_c;
centroidalDyn.dJcNu                       = dJcNu_c;
centroidalDyn.Nu                          = Nu_c;
centroidalDyn.Jc                          = Jc_c;

end

