function [Mc,C_cNu_c, gc, Jcc, dJcDq_c, Nu_c] = centroidalConversion(M, CNu, g, Jc, dJcDq, Nu, T, dT)
% centroidalConversion 
% converts dynamic equation parameters to the
% corresponding values in centroidal frame of reference
ndof   = size(g,1)-6;

invT   = eye(ndof+6)/T;
invTt  = eye(ndof+6)/(T');

%% control terms conversion
Mc = invTt*M*invT;

Mc(1:6,7:end) = zeros(6,ndof);
Mc(7:end,1:6) = zeros(ndof,6);

Mc(1:3,1:3)   = M(1,1)*eye(3);
Mc(1:3,4:6)   = zeros(3);
Mc(4:6,1:3)   = zeros(3);

Nu_c         = T*Nu;
gravAcc      = norm(invTt*g)/M(1,1);

e3         = zeros(ndof+6,1);
e3(3)      = 1;
gc         = M(1,1)*gravAcc*e3;

C_cNu_c_ini  = invTt*CNu - Mc*dT*Nu;

CNu_j = CNu(7:end);
CNu_b = CNu(1:6);

Mb   =   M(1:6,1:6);
Mbj  =   M(1:6,7:end);

C_cNuc_b = [zeros(3,1); C_cNu_c_ini(4:6)];
C_cNuc_j = CNu_j - (Mbj')*(Mb\CNu_b);

C_cNu_c  = [C_cNuc_b; C_cNuc_j];

% % for verify the formula
% g_j  = g(7:end);
% gc_j = g_j - (Mbj')*(Mb\g_b);

%new dT*Nu computation for Jacobian
invMc        = eye(ndof+6)/Mc;
dTNu         = invMc*(CNu-C_cNu_c);  

Jcc       = Jc*invT;
dJcDq_c   = dJcDq - Jc*invT*dTNu;

end