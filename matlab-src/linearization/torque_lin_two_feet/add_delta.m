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
PINV_TOL    = lparam.PINV_TOL;
link_name   = lparam.link_name;
n_constr    = lparam.n_constr;
num_constr  = lparam.num_constr;

%% Redefinition of all terms
%Feet Jacobian
Jc_new        = zeros(n_constr,n_tot);

for ii=1:num_constr
    
   Jc_new(6*(ii-1)+1:6*ii,:) = wbm_jacobian(rotation,position,qjoints,link_name{ii});

end

%Mass matrix
M_new = wbm_massMatrix(rotation,position,qjoints);

%Matrix A at CoM
x_lsole_new  = wbm_forwardKinematics(rotation,position,qjoints,'l_sole');
x_rsole_new  = wbm_forwardKinematics(rotation,position,qjoints,'r_sole');
com_new      = wbm_forwardKinematics(rotation,position,qjoints,'com');

pos_leftFoot_new     = x_lsole_new(1:3);
pos_rightFoot_new    = x_rsole_new(1:3);
xcom_new             = com_new(1:3);

Pr_new               = pos_rightFoot_new  - xcom_new ;  
Pl_new               = pos_leftFoot_new   - xcom_new ;

AL_new  = [ eye(3),  zeros(3);
           Sf(Pl_new),  eye(3)];
AR_new  = [ eye(3),  zeros(3);
           Sf(Pr_new),  eye(3) ];
   
A_new   = [AL_new , AR_new];

%% Fixed parameters
invS6MS6t_new  = eye(n_base)/(S6*M_new*S6t);

Minv_new    = eye(n_tot)/M_new ;
pinvA_new   = pinv(A_new , PINV_TOL);
NA_new      = eye(n_constr) - pinvA_new*A_new ;
Jct_new     = Jc_new .';
D_new       = S7 - (S7*M_new*S6t)*invS6MS6t_new*S6;

%% Composed variables
Lambda_new  = Jc_new*Minv_new*St;
pinvL_new   = pinv(Lambda_new , PINV_TOL);
NL_new      = eye(n_joints) - pinvL_new*Lambda_new ;

Sigma_new         = -pinvL_new*(Jc_new*Minv_new*Jct_new ) -NL_new*D_new*Jct_new ;
pinvSigmaNA_new   =  pinv(Sigma_new*NA_new , PINV_TOL);

f12_new    =  pinvA_new  - NA_new*pinvSigmaNA_new*(Sigma_new*pinvA_new);
f1_new     = -f12_new*grav -NA_new*pinvSigmaNA_new*(pinvL_new*Jc_new*g_bar*e3 + NL_new*D_new*M_new*g_bar*e3);

P_new      = Jc_new*g_bar*e3 - Jc_new*Minv_new*Jct_new*f1_new;
P0_new     = D_new*(M_new*g_bar*e3 - Jct_new*f1_new);

%Torques at equilibrium
tau_delta  = pinvL_new*P_new + NL_new*P0_new;

end















