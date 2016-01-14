function tau_delta = add_delta(rotation, position, qjoints, lparam)

g_bar       = lparam.g_bar;
e3          = lparam.e3;
St          = lparam.St;
S6          = lparam.S6;
S6t         = lparam.S6t;
S7          = lparam.S7;
grav        = lparam.grav;
n_tot       = lparam.n_tot;
n_base      = lparam.n_base;
n_joints    = lparam.n_joints;
toll        = lparam.toll;

%% Redefinition of all terms
%Feet jacobian
Jc_new = wbm_jacobian(rotation, position, qjoints, 'l_sole');

%Mass matrix
M_new = wbm_massMatrix(rotation, position, qjoints);
 
%Matrix A at CoM
x_lsole_new = wbm_forwardKinematics(rotation, position, qjoints,'l_sole');
com_new     = wbm_forwardKinematics(rotation, position, qjoints, 'com');

pos_leftFoot_new   = x_lsole_new(1:3);
xcom_new           = com_new(1:3); 

Pl_new             = pos_leftFoot_new  - xcom_new;

A_new = [ eye(3)   zeros(3);
         Sf(Pl_new)  eye(3)];

%composed variables
Minv_new  = eye(n_tot)/M_new;
pinvA_new = eye(n_base)/A_new;
Jct_new   = Jc_new.';

invS6MS6t_new = eye(n_base)/(S6*M_new*S6t);
D_new         = S7 - (S7*M_new*S6t)*invS6MS6t_new*S6;

Lambda_new = Jc_new*Minv_new*St;

%pinvL_new  = pinv(Lambda_new, toll);
 pinvL_new  = Lambda_new.'/(Lambda_new*Lambda_new.' + toll*eye(size(Lambda_new,1)));

NL_new     = eye(n_joints) - pinvL_new*Lambda_new;

P_new      = Jc_new*g_bar*e3 + Jc_new*Minv_new*Jct_new*pinvA_new*grav;

P0_new     = D_new*(M_new*g_bar*e3 + Jct_new*pinvA_new*grav);

%new torques
tau_delta  = pinvL_new*P_new + NL_new*P0_new;

end















