function DYNAMICS = robotDynamics(STATE,CONFIG)
%ROBOTDYNAMICS defines the robot dynamics given the current state and the user
%         desired configuration. It is a wrapper for the mex-WBM functions.
%
% DYNAMICS = ROBOTDYNAMICS(STATE,CONFIG) takes as an input the current
% state of the robot, which is defined in the structure STATE, and
% the structure CONFIG which contains all the user-defined parameters. The 
% output is the structure DYNAMICS which contains the system mass matrix, 
% Coriolis forces, gravity vector, and so on.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% Config parameters
ndof            = CONFIG.ndof;
massCorr        = CONFIG.massCorr;

% State parameters
w_R_b           = STATE.w_R_b;
x_b             = STATE.x_b;
qj              = STATE.qj;
dqj             = STATE.dqj;
dx_b            = STATE.dx_b;
w_omega_b       = STATE.w_omega_b;

%% ROBOT DYNAMICS
% mass matrix
M                                = wbm_massMatrix(w_R_b,x_b,qj);

% correction to ensure the mass matrix to be positive definite. It is used
% only when the system is integrated using a fixed step integrator,
% otherwise it won't converge to a solution.
M(7:end,7:end)                   = M(7:end,7:end) + massCorr.*eye(ndof);

% generalized bias forces, Coriolis and gravity
h                                = wbm_generalizedBiasForces(w_R_b,x_b,qj,dqj,[dx_b;w_omega_b]);
g                                = wbm_generalizedBiasForces(w_R_b,x_b,qj,zeros(ndof,1),zeros(6,1));
C_nu                             = h-g;

% centroidal momentum
H                                = wbm_centroidalMomentum(w_R_b,x_b,qj,dqj,[dx_b;w_omega_b]);

% Jacobians and dJ_nu
JCoM                             = wbm_jacobian(w_R_b,x_b,qj,'com');
dJCoM_nu                         = wbm_dJdq(w_R_b,x_b,qj,dqj,[dx_b;w_omega_b],'com');
Jc                               = zeros(6*CONFIG.numConstraints,6+ndof);
dJc_nu                           = zeros(6*CONFIG.numConstraints,1);

for i=1:CONFIG.numConstraints
    
    Jc(6*(i-1)+1:6*i,:)          = wbm_jacobian(w_R_b,x_b,qj,CONFIG.constraintLinkNames{i});
    dJc_nu(6*(i-1)+1:6*i,:)      = wbm_dJdq(w_R_b,x_b,qj,dqj,[dx_b;w_omega_b],CONFIG.constraintLinkNames{i});
end

% centroidal momentum Jacobian
JHBase                           = zeros(6,6);
JHJoint                          = zeros(6,ndof);

for ii = 1:6
    
    v_bJH                        = zeros(6,1);
    v_bJH(ii)                    = 1;
    JHBase(:,ii)                 = wbm_centroidalMomentum(w_R_b,x_b,qj,zeros(ndof,1),v_bJH);
end

for ii = 1:ndof
    
    dqjJH                        = zeros(ndof,1);
    dqjJH(ii)                    = 1;
    JHJoint(:,ii)                = wbm_centroidalMomentum(w_R_b,x_b,qj,dqjJH,zeros(6,1));
end

JH                               = [JHBase JHJoint];
dJH_nu                           = [M(1,1)*dJCoM_nu(1:3); zeros(3,1)];

%% Generate the output
DYNAMICS.M                       = M;
DYNAMICS.h                       = h;
DYNAMICS.g                       = g;
DYNAMICS.H                       = H;
DYNAMICS.C_nu                    = C_nu;
DYNAMICS.JCoM                    = JCoM;
DYNAMICS.dJCoM_nu                = dJCoM_nu;
DYNAMICS.Jc                      = Jc;
DYNAMICS.dJc_nu                  = dJc_nu;
DYNAMICS.JH                      = JH;
DYNAMICS.dJH_nu                  = dJH_nu;

end
