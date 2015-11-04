clear all
close all
clc

%% Linearization around a given joints position
addpath('./../../whole_body_model_functions/');
addpath('./../../../../../build/');
addpath('./../../worker_functions');
addpath('./../../');

%% Initialize the model
wbm_modelInitialise('icubGazeboSim');
wbm_setWorldLink('l_sole',eye(3),[0 0 0]',[0,0,-9.81]');

%% Define the desired joints position
% initial conditions             
 torsoInit    = [ -10.0   0.0    0.0]';
 leftArmInit  = [ -19.7   29.7   0.0   44.9   0.0]';          
 rightArmInit = [ -19.7   29.7   0.0   44.9   0.0]';         
 leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
 rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1]'; 
 
qjDes  = [torsoInit; leftArmInit; rightArmInit; leftLegInit; rightLegInit]*(pi/180);

%% Update the position
n_tot    = 31;
n_joints = 25;
n_base   = 6;

wbm_updateState(qjDes,zeros(n_joints,1),zeros(n_base,1));

[~,T_b,~,~] = wbm_getState();
[pos,rot]   = frame2posrot(T_b);

%% Steady-state torques definition
rot_tr  = rot.';

%Feet Jacobian
Jc      = wbm_jacobian(rot_tr, pos, qjDes, 'l_sole');
%Mass matrix
M       = wbm_massMatrix(rot_tr, pos, qjDes);
%Matrix A at CoM
com     = wbm_forwardKinematics(rot_tr, pos, qjDes, 'com');
xcom    = com(1:3);
A       = eye(3);

%Constant values
toll = 1e-10;

g_bar = 9.81;
m     = M(1,1);

e3    = zeros(n_tot,1);
e3(3) = 1;

S     = [zeros(n_joints,n_base) eye(n_joints)];
St    = S.';
S6    = [eye(n_base)  zeros(n_base, n_joints)];
S6t   = S6.';
S7    = [zeros(n_joints,n_base) eye(n_joints)];

grav  = [0; 0; -m*g_bar];

%Composed variables
Minv  = eye(n_tot)/M;
pinvA = eye(3)/A;
Jct   = Jc.';

invS6MS6t = eye(n_base)/(S6*M*S6t);
D         = S7 - (S7*M*S6t)*invS6MS6t*S6;

Lambda    = Jc*Minv*St;

%pinvL    = pinv(Lambda, toll);
 pinvL    = Lambda.'/(Lambda*Lambda.' + toll*eye(size(Lambda,1)));

NL        = eye(n_joints) - pinvL*Lambda;

%% Final terms
Jcf =  Jc(1:3,:);
Jcft = Jcf.';
Jcm =  Jc(4:6,:);
Jcmt = Jcm.';

Gamma       = -pinvL*(Jc*Minv*Jcmt);
NL_tilde    =  [Gamma NL];
NL_tilde2   =  [(Gamma+D*Jcmt) NL];
pinvNL_t2   =  pinv(NL_tilde2, toll);

tau_base   =  pinvL*(Jc*g_bar*e3 + Jc*Minv*Jcft*pinvA*grav);

post       =  D*(M*g_bar*e3 +Jcft*pinvA*grav);

tau0_tilde =  pinvNL_t2*(post-tau_base);

tau_reg = tau_base + NL_tilde*tau0_tilde;

R1      = pinvL*(Jc*Minv*Jcft*pinvA) -NL_tilde*pinvNL_t2*(-D*Jcft*pinvA +pinvL*(Jc*Minv*Jcft*pinvA)); 

NL_tot  = NL_tilde*pinvNL_t2;
save('tau_reg','tau_reg')
 
%% Gains
gainsPCOM    = diag([50 50 50]);
gainsDCOM    = diag([1  1  1]);
gainMomentum = 1;

impTorso            = [  20    20   20
                          0     0    0]; 

impArms             = [ 13  13   13   5   5
                         0    0    0   0   0 ];

impLeftLeg          = [ 70  70  65  40  10  10
                         0   0   0   0   0   0]; 
                         
impRightLeg         = [ 20  20  20  10  10  10
                         0   0   0   0   0   0];
                         
impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)]; 
dampings            = 1*ones(1,n_joints);

Kimp = diag(impedances);
Kder = diag(dampings);
Kp   = gainsPCOM;
Kd   = gainsDCOM;
Kg   = gainMomentum*eye(3);

%% Floating base transformations
[T_bar, ~] = parametrization(rot_tr);

T_tilde    = [  eye(3)  zeros(3);
              zeros(3)   T_bar ];

T          = [    T_tilde            zeros(n_base,n_joints);
              zeros(n_joints,n_base)         eye(n_joints)];

%CoM Jacobian 
Jcom       = wbm_jacobian(rot_tr, pos, qjDes, 'com');

Jcom_lin   = Jcom(1:3,:);
Jcom_lin_b = Jcom_lin(:,1:n_base);
Jcom_lin_q = Jcom_lin(:,n_base+1:end);

%Jw for angular momentum
Jwb = zeros(3,n_base);
Jwq = zeros(3,n_joints);

for ii = 1:n_base

vb     = zeros(n_base,1);
vb(ii) = 1;

H = wbm_centroidalMomentum(rot_tr, pos, qjDes, zeros(n_joints,1), vb);

Jwb(:,ii) = H(4:end);

end

for ii = 1:n_joints

dqj     = zeros(n_joints,1);
dqj(ii) = 1;

H = wbm_centroidalMomentum(rot_tr, pos, qjDes, dqj, zeros(n_base,1));

Jwq(:,ii) = H(4:end);

end

%% Transformation between vb and dqj
Jc_b = Jc(1:n_base, 1:n_base);
Jc_q = Jc(1:n_base, n_base+1:end);

conv_vb = -(eye(n_base)/Jc_b)*Jc_q;

%% Variables needed for others functions
lparam.delta        = 0.00001;
lparam.qjDes        = qjDes;

lparam.m            = m;
lparam.g_bar        = g_bar;
lparam.e3           = e3;
lparam.St           = St;
lparam.S6           = S6;
lparam.S6t          = S6t;
lparam.S7           = S7;
lparam.grav         = grav;
lparam.n_tot        = n_tot;
lparam.n_base       = n_base;
lparam.n_joints     = n_joints;
lparam.toll         = toll;

%% Numerical derivative of tau_no_gains
dtau_ng = zeros(n_joints);

for jj = 1:n_joints

numerical_tau = num_der(jj, lparam);
dtau_ng(:,jj) = numerical_tau;
 
end

%% Other derivatives with respect of position
Jcom_a      = Jcom*T;

Jcom_a_linb = Jcom_a(1:3,1:n_base);
Jcom_a_linq = Jcom_a(1:3,n_base+1:end);

total_Jcom  = Jcom_a_linb*(eye(n_base)/T_tilde)*conv_vb + Jcom_a_linq;
 
dqHdot      = -m*Kp*total_Jcom;

%total stiffness
KS = dtau_ng -R1*dqHdot -NL_tot*Kimp;
save('KS','KS')

%% Other derivatives with respect of velocity
xD_der = Jcom_lin_b*conv_vb + Jcom_lin_q;

dqDHdot     =    -m*Kd*xD_der;

%total damping
KD = -R1*dqDHdot -NL_tot*Kder;
save('KD','KD')

%% Positive definite verify 
eig_A = eig(-KS);
flag  = 0;

for i = 1:length(eig_A)
   
    if eig_A(i) <= 0 
	flag = 1;
    end
    
end

if flag == 1
    
	disp('the stiffness matrix is not positive definite')
	else
	disp('the stiffness matrix is positive definite')
    
end
 
eig_B = eig(-KD);
flag  = 0;

for i = 1:length(eig_B)
   
    if eig_B(i) <= 0 
	flag = 1;
    end
    
end

if flag == 1
    
	disp('the damping matrix is not positive definite')
	else
	disp('the damping matrix is positive definite')
    
end

%% Gains tuning using kronecher product
% stiffness matrix
desired_KS = -dtau_ng -25*eye(n_joints);

R11  = -R1*(-m*eye(3));
R12  =  total_Jcom;

S11  = -NL_tot;
S12  =  eye(n_joints);

[Kp_n, Kimp_n] = kronecher_prod(desired_KS,R11,R12,S11,S12);

KS_new = dtau_ng -R1*(-m*Kp_n*total_Jcom) -NL_tot*Kimp_n;

% damping matrix
desired_KD = -5*eye(n_joints);

R21 = -R1*(-m*eye(3));
R22 =  xD_der;

S21 = -NL_tot;
S22 =  eye(n_joints);

[Kd_n, Kder_n] = kronecher_prod(desired_KD,R21,R22,S21,S22);

KD_new = -R1*(-m*Kd_n*xD_der) -NL_tot*Kder_n;

%% Eigenvalues evaluation
disp('eigenvalues of KS_new and KD_new')
disp([eig(KS_new) eig(KD_new)])

disp('eigenvalues of Kp_n, Kd_n')
disp([eig(Kp_n) eig(Kd_n)])

disp('eigenvalues of Kimp and Kder')
disp([eig(Kimp_n) eig(Kder_n)])

gains_tun.Kimp_n = Kimp_n;
gains_tun.Kder_n = Kder_n;
gains_tun.Kp_n   = Kp_n;
gains_tun.Kd_n   = Kd_n;

save('gains_tun','gains_tun')

