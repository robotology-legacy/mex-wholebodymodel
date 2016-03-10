function [M_c,C_cNu_c,g_c,Jc_c,dJcNu_c,Nu_c] = fromFloatingToCentroidalDynamics(M, h, g, Jc, dJcNu, Nu, T, dT)
% fromFloatingToCentroidalDynamics
% converts dynamic equation parameters to the
% corresponding values in centroidal frame of reference
ndof   = size(g,1)-6;

invT   = eye(ndof+6)/T;
invTt  = eye(ndof+6)/(T');

%% control terms conversion
M_c = invTt*M*invT;

M_c(1:6,7:end) = zeros(6,ndof);
M_c(7:end,1:6) = zeros(ndof,6);

M_c(1:3,1:3)   = M(1,1)*eye(3);
M_c(1:3,4:6)   = zeros(3);
M_c(4:6,1:3)   = zeros(3);

Nu_c         = T*Nu;
gravAcc      = norm(invTt*g)/M(1,1);

e3           = zeros(ndof+6,1);
e3(3)        = 1;
g_c          = M(1,1)*gravAcc*e3;

%coriolis terms
CNu           = h - g;

CNu_j         = CNu(7:end);
CNu_b         = CNu(1:6);

Mb            =   M(1:6,1:6);
Mbj           =   M(1:6,7:end);

C_cNu_c_dT    = invTt*CNu - M_c*dT*Nu;

C_cNu_c       = [ zeros(3,1); 
                  C_cNu_c_dT(4:6); 
                  CNu_j-(Mbj')*(Mb\CNu_b)];

%new dT*Nu computation for Jacobian
dTNu          = M_c\(CNu-C_cNu_c);  

Jc_c          = Jc*invT;
dJcNu_c       = dJcNu - Jc*invT*dTNu;

end