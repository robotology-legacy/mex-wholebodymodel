clear all
close all
clc

%% Linearization around a given joints position
addpath('./../../mex-wholebodymodel/matlab/utilities');
addpath('./../../mex-wholebodymodel/matlab/wrappers');
addpath('./../../../../build/');
addpath('./../');

%% Initialize the model
wbm_modelInitialise('icubGazeboSim');

%% Define the desired joints position
torsoInit    = [ -10.0   0.0    0.0]';
leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1]'; 
leftArmInit  = [ -19.7   29.7   0.0   44.9   0.0]';          
rightArmInit = [ -19.7   29.7   0.0   44.9   0.0]';
 
qjDes  = [torsoInit; leftArmInit; rightArmInit; leftLegInit; rightLegInit]*(pi/180);

%% Update the position
n_tot    = 31;
n_joints = 25;
n_base   = 6;

wbm_updateState(qjDes,zeros(n_joints,1),zeros(n_base,1));

[rot,pos]   = wbm_getWorldFrameFromFixedLink('l_sole',qjDes);

%% Steady-state torques definition
rot_tr = rot.';

%Feet Jacobian
Jc  = wbm_jacobian(rot_tr, pos, qjDes, 'l_sole');

%Mass matrix
M   = wbm_massMatrix(rot_tr, pos, qjDes);

%Matrix A at CoM
x_lsole = wbm_forwardKinematics(rot_tr, pos, qjDes, 'l_sole');
com     = wbm_forwardKinematics(rot_tr, pos, qjDes, 'com');

pos_leftFoot = x_lsole(1:3);
xcom         = com(1:3);
Pl           = pos_leftFoot - xcom;

A            = [eye(3)   zeros(3);
                Sf(Pl)     eye(3)];

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

grav  = [0; 0; -m*g_bar; zeros(3,1)];

%Composed variables
Minv  = eye(n_tot)/M;
pinvA = eye(n_base)/A;
Jct   = Jc.';

invS6MS6t = eye(n_base)/(S6*M*S6t);
D         = S7 - (S7*M*S6t)*invS6MS6t*S6;

Lambda    = Jc*Minv*St;

%pinvL     = pinv(Lambda, toll);
 pinvL     = Lambda.'/(Lambda*Lambda.' + toll*eye(size(Lambda,1)));

NL         = eye(n_joints) - pinvL*Lambda;

%% Final terms
P  = Jc*g_bar*e3 + Jc*Minv*Jct*pinvA*grav;

P0 = D*(M*g_bar*e3 + Jct*pinvA*grav);

R1 = pinvL*Jc*Minv*Jct*pinvA + NL*D*Jct*pinvA;

tau_reg = pinvL*P + NL*P0;
save('tau_reg','tau_reg')

%% Gains
gainsPCOM    = diag([50 50 50]);
gainsDCOM    = diag([1  1  1]);
gainMomentum = 1;

impTorso            = [  20    20   20
                          0     0    0]; 

impArms             = [ 13  13   13   5   5
                         0    0    0   0   0 ];

impLeftLeg          = [ 70  70  65  30  10  10
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
Kg2  = 15*eye(3);

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

% torso Jacobian
Jt         = wbm_jacobian(rot_tr, pos, qjDes, 'torso');

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
Jcom_a = Jcom*T;
Jt_a   = Jt*T;

Jcom_a_linb = Jcom_a(1:3,1:n_base);
Jcom_a_linq = Jcom_a(1:3,n_base+1:end);

Jt_a_b = Jt_a(4:6,1:n_base);
Jt_a_q = Jt_a(4:6,n_base+1:end);

total_Jcom  = Jcom_a_linb*(eye(n_base)/T_tilde)*conv_vb + Jcom_a_linq;
total_Jt    = Jt_a_b*(eye(n_base)/T_tilde)*conv_vb + Jt_a_q;
 
dqHdot      = [-m*Kp*total_Jcom; -Kg2*total_Jt];

%total stiffness
KS = dtau_ng -R1*dqHdot -NL*Kimp;
save('KS','KS')

%% Other derivatives with respect of velocity
xD_der = Jcom_lin_b*conv_vb + Jcom_lin_q;
Hw_der = Jwb*conv_vb + Jwq;

dqDHdot     = [-m*Kd*xD_der; -Kg*Hw_der];

%total damping
KD = -R1*dqDHdot -NL*Kder;
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

R11  = -R1;
R12  =  [total_Jcom; total_Jt];

S11  = -NL;
S12  =  eye(n_joints);

[Kp_n, Kg2_n, Kimp_n] = kronecher_prod(desired_KS,R11,R12,S11,S12,lparam);

KS_new = dtau_ng -R1*[-m*Kp_n*total_Jcom; -Kg2_n*total_Jt] -NL*Kimp_n;

% damping matrix
desired_KD = -5*eye(n_joints);

R21 = -R1;
R22 =  [xD_der ; Hw_der];

S21 = -NL;
S22 =  eye(n_joints);

[Kd_n, Kg_n, Kder_n] = kronecher_prod(desired_KD,R21,R22,S21,S22,lparam);

KD_new = -R1*[-m*Kd_n*xD_der; -Kg_n*Hw_der] -NL*Kder_n;

%% Eigenvalues evaluation
disp('eigenvalues of KS_new and KD_new')
disp([eig(KS_new) eig(KD_new)])

disp('eigenvalues of Kp_n, Kg2_n, Kd_n, Kg_n')
disp([eig(Kp_n) eig(Kg2_n) eig(Kd_n) eig(Kg_n)])

% disp('eigenvalues of Kimp and Kder')
% disp([eig(Kimp_n) eig(Kder_n)])

gains_tun.Kimp_n = Kimp_n;
gains_tun.Kder_n = Kder_n;
gains_tun.Kp_n   = Kp_n;
gains_tun.Kd_n   = Kd_n;
gains_tun.Kg_n   = Kg_n;
gains_tun.Kg2_n  = Kg2_n;

save('gains_tun','gains_tun')

