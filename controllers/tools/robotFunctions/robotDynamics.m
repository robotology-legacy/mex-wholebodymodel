function DYNAMICS = robotDynamics(STATE,MODEL)
%ROBOTDYNAMICS defines the robot dynamics given the current state and the
%              robot model. It is a wrapper for the mex-WBM functions.
%
% Format: DYNAMICS = ROBOTDYNAMICS(STATE,MODEL)
%
% Inputs:  - STATE contains the current system state;
%          - MODEL is a structure defining the robot model;
%
% Output:  - STATE which is a structure containing the following variables:
%
% M          mass matrix (R^(n+6 x n+6))
% h          generalized bias forces (R^(n+6))
% g          gravity forces (R^(n+6))
% H          centroidal momentum (R^6)
% C_nu       Coriolis and centrifugal effects (R^(n+6))
% JCoM       CoM jacobian (R^(6 x n+6))
% dJCoM_nu   COM jacobian derivative times state velocity (R^6)
% Jc         contact jacobian (R^(6*numConstraints x n+6))
% dJc_nu     contact jacobian derivative times state velocity (R^6)
% JH         centroidal momentum matrix (R^(6 x n+6))
% dJH_nu     centroidal momentum matrix derivative times state velocity (R^6)
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% Config parameters
ndof            = MODEL.ndof;
numConstraints  = sum(MODEL.CONFIG.feet_on_ground);

%% State parameters
w_R_b           = STATE.w_R_b;
x_b             = STATE.x_b;
qj              = STATE.qj;
dqj             = STATE.dqj;
dx_b            = STATE.dx_b;
w_omega_b       = STATE.w_omega_b;

%% ROBOT DYNAMICS
% mass matrix
M                           = wbm_massMatrix(w_R_b,x_b,qj);
M(7:end,7:end)              = M(7:end,7:end) + 0.1.*eye(ndof);

% generalized bias forces, Coriolis and gravity
h                           = wbm_generalizedBiasForces(w_R_b,x_b,qj,dqj,[dx_b;w_omega_b]);
g                           = wbm_generalizedBiasForces(w_R_b,x_b,qj,zeros(ndof,1),zeros(6,1));
C_nu                        = h-g;
% centroidal momentum
H                           = wbm_centroidalMomentum(w_R_b,x_b,qj,dqj,[dx_b;w_omega_b]);
% Jacobians and dJ_nu
JCoM                        = wbm_jacobian(w_R_b,x_b,qj,'com');
dJCoM_nu                    = wbm_dJdq(w_R_b,x_b,qj,dqj,[dx_b;w_omega_b],'com');
Jc                          = zeros(6*numConstraints,6+ndof);
dJc_nu                      = zeros(6*numConstraints,1);

for i=1:numConstraints
    Jc(6*(i-1)+1:6*i,:)     = wbm_jacobian(w_R_b,x_b,qj,MODEL.constraintLinkNames{i});
    dJc_nu(6*(i-1)+1:6*i,:) = wbm_dJdq(w_R_b,x_b,qj,dqj,[dx_b;w_omega_b],MODEL.constraintLinkNames{i});
end
% centroidal momentum Jacobian
JHBase                      = zeros(6,6);
JHJoint                     = zeros(6,ndof);
for ii = 1:6
    
    v_bJH                   = zeros(6,1);
    v_bJH(ii)               = 1;
    JHBase(:,ii)            = wbm_centroidalMomentum(w_R_b,x_b,qj,zeros(ndof,1),v_bJH);
end
for ii = 1:ndof
    
    dqjJH                   = zeros(ndof,1);
    dqjJH(ii)               = 1;
    JHJoint(:,ii)           = wbm_centroidalMomentum(w_R_b,x_b,qj,dqjJH,zeros(6,1));
end
JH                          = [JHBase JHJoint];
dJH_nu                      = [M(1,1)*dJCoM_nu(1:3); zeros(3,1)];

%% Output structure
DYNAMICS.M                  = M;
DYNAMICS.h                  = h;
DYNAMICS.g                  = g;
DYNAMICS.H                  = H;
DYNAMICS.C_nu               = C_nu;
DYNAMICS.JCoM               = JCoM;
DYNAMICS.dJCoM_nu           = dJCoM_nu;
DYNAMICS.Jc                 = Jc;
DYNAMICS.dJc_nu             = dJc_nu;
DYNAMICS.JH                 = JH;
DYNAMICS.dJH_nu             = dJH_nu;

end
