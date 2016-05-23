function dynamics = robotDynamics(state,config)
%ROBOTDYNAMICS defines the robot dynamics from the current state and the user
%         desired configuration. It is a wrapper for the WBM functions.
%
%         dynamics = ROBOTDYNAMICS(state,config) takes as an input the current 
%         state of the robot, which is defined in the structure STATE, and
%         the structure CONFIG which contains all the user-defined
%         parameters. The output is the structure DYNAMICS which contains
%         the system mass matrix, Coriolis forces, gravity vector, and so
%         on.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% Config parameters
ndof           = config.ndof;
MassCorr       = config.MassCorr;
% State parameters
RotBase        = state.RotBase;
PosBase        = state.PosBase;
qj             = state.qj;
dqj            = state.dqj;
VelBase        = state.VelBase;
omegaBaseWorld = state.omegaBaseWorld;

%% ROBOT DYNAMICS
% mass matrix
M                                = wbm_massMatrix(RotBase,PosBase,qj);
% correction to make the mass matrix positive definite 
M(7:end,7:end)                   = M(7:end,7:end) + MassCorr.*eye(ndof);

% generalized bias forces, Coriolis and gravity
h                                = wbm_generalisedBiasForces(RotBase,PosBase,qj,dqj,[VelBase;omegaBaseWorld]);
g                                = wbm_generalisedBiasForces(RotBase,PosBase,qj,zeros(ndof,1),zeros(6,1));
CNu                              = h-g;

% centroidal momentum
H                                = wbm_centroidalMomentum(RotBase,PosBase,qj,dqj,[VelBase;omegaBaseWorld]);

% Jacobians and dJNu
JCoM                             = wbm_jacobian(RotBase,PosBase,qj,'com');
dJCoMNu                          = wbm_djdq(RotBase,PosBase,qj,dqj,[VelBase;omegaBaseWorld],'com');
Jc                               = zeros(6*config.numConstraints,6+ndof);
dJcNu                            = zeros(6*config.numConstraints,1);

for i=1:config.numConstraints
    
    Jc(6*(i-1)+1:6*i,:)          = wbm_jacobian(RotBase,PosBase,qj,config.constraintLinkNames{i});
    dJcNu(6*(i-1)+1:6*i,:)       = wbm_djdq(RotBase,PosBase,qj,dqj,[VelBase;omegaBaseWorld],config.constraintLinkNames{i});    
end

% centroidal momentum Jacobian
JHBase                           = zeros(6,6);
JHJoint                          = zeros(6,ndof);

for ii = 1:6
    
NuBaseJH                         = zeros(6,1);
NuBaseJH(ii)                     = 1;
JHBase(:,ii)                     = wbm_centroidalMomentum(RotBase, PosBase, qj, zeros(ndof,1), NuBaseJH);
end

for ii = 1:ndof

dqjJH                            = zeros(ndof,1);
dqjJH(ii)                        = 1;
JHJoint(:,ii)                    = wbm_centroidalMomentum(RotBase, PosBase, qj, dqjJH, zeros(6,1));
end

JH                               = [JHBase JHJoint]; 
dJHNu                            = [M(1,1)*dJCoMNu(1:3); zeros(3,1)];

%% Generate the output
dynamics.M                       = M;
dynamics.h                       = h;
dynamics.g                       = g;
dynamics.H                       = H;
dynamics.CNu                     = CNu;
dynamics.JCoM                    = JCoM;
dynamics.dJCoMNu                 = dJCoMNu;
dynamics.Jc                      = Jc;
dynamics.dJcNu                   = dJcNu;
dynamics.JH                      = JH;
dynamics.dJHNu                   = dJHNu;

end
