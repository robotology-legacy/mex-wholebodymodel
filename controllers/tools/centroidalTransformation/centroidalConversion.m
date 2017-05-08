function [STATE,DYNAMICS] = centroidalConversion(DYNAMICS_OLD,FORKINEMATICS,STATE_OLD)
%CENTROIDALCONVERSION applies the centroidal coordinates tranformation to the
%                     system, i.e. the floating base dynamics now coincides
%                     with the centroidal momentum dynamics.
%
% Format: [STATE,DYNAMICS] = CENTROIDALCONVERSION(DYNAMICS_OLD,FORKINEMATICS,STATE_OLD)
%
% Inputs:  - DYNAMICS_OLD contains current robot dynamics;
%          - FORKINEMATICS stores position, orientation and velocity of
%            points in cartesian space;
%          - STATE_OLD contains the current system state;
%
% Output:  - DYNAMICS contains current robot dynamics;
%          - STATE contains the current system state;
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
DYNAMICS = DYNAMICS_OLD;
STATE    = STATE_OLD;

%% Dynamics 
M        = DYNAMICS_OLD.M;
h        = DYNAMICS_OLD.h;
g        = DYNAMICS_OLD.g;
Jc       = DYNAMICS_OLD.Jc;
dJc_nu   = DYNAMICS_OLD.dJc_nu;
Mbj      = M(1:6,7:end);
Mb       = M(1:6,1:6);

%% Forward kinematics
xCoM     = FORKINEMATICS.xCoM;
dxCoM    = FORKINEMATICS.dxCoM;

%% Robot state
nu       = STATE_OLD.nu;
dx_b     = STATE_OLD.dx_b;
x_b      = STATE_OLD.x_b;

%% Compute transformation matrix T and its derivative
ndof     = size(Mbj,2);
r        = xCoM -x_b;
X        = [eye(3),skewm(r)';
            zeros(3),eye(3)]; 
Js       = X*(Mb\Mbj);
% time derivatives
dr       = dxCoM - dx_b;
mdr      = M(1,1)*dr;
dMb      = [zeros(3),skewm(mdr)';
            skewm(mdr),zeros(3)]; 
inv_dMb  = -Mb\dMb/Mb;
dX       = [zeros(3),skewm(dr)';
            zeros(3),zeros(3)];  
dJs      = dX*(Mb\Mbj) + X*inv_dMb*Mbj;
% compute transformation (matrices T and dT)
T        = [X,Js;
            zeros(ndof,6),eye(ndof)];
dT       = [dX,dJs;
            zeros(ndof,6),zeros(ndof)];
% matrix inverse        
invT     = eye(ndof+6)/T;
invTt    = eye(ndof+6)/(transpose(T));

%% State and dynamics conversion to centroidal coordinates
% mass matrix
M_c            = invTt*M*invT;
M_c(1:6,7:end) = zeros(6,ndof);
M_c(7:end,1:6) = zeros(ndof,6);
M_c(1:3,1:3)   = M(1,1)*eye(3);
M_c(1:3,4:6)   = zeros(3);
M_c(4:6,1:3)   = zeros(3);
Mb             = M(1:6,1:6);
Mbj            = M(1:6,7:end);
% velocity and gravity
nu_c           = T*nu;
gravAcc        = norm(invTt*g)/M(1,1);
e3             = zeros(ndof+6,1);
e3(3)          = 1;
g_c            = M(1,1)*gravAcc*e3;
% Coriolis terms
C_nu           = h - g;
C_nu_j         = C_nu(7:end);
C_nu_b         = C_nu(1:6);
C_nu_c_dT      = invTt*C_nu - M_c*dT*nu;
C_nu_c         = [ zeros(3,1);
                  C_nu_c_dT(4:6);
                  C_nu_j-(transpose(Mbj))*(Mb\C_nu_b)];
% dT*nu computation for Jacobian
dT_nu          = M_c\(C_nu-C_nu_c);
Jc_c           = Jc*invT;
dJc_nu_c       = dJc_nu - Jc*invT*dT_nu;
   
%% Output structure
STATE.nu               = nu_c;
DYNAMICS.M             = M_c;
DYNAMICS.g             = g_c;
DYNAMICS.C_nu          = C_nu_c;
DYNAMICS.dJc_nu        = dJc_nu_c;
DYNAMICS.Jc            = Jc_c;

end

