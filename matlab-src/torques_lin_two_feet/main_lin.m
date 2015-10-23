clear all
close all
clc

%% Paths
addpath('./../whole_body_model_functions/');
addpath('./../../../../build/');
addpath('./../worker_functions');
addpath('./../');

%% Initialize the model
wbm_modelInitialise('icubGazeboSim');
wbm_setWorldLink('l_sole',eye(3),[0 0 0]',[0,0,-9.81]');

%% Define the initial position 
torsoInit    = [ -10.0   0.0   0.0]';                   
leftArmInit  = [ -19.7  29.7   0.0  44.9  0.0]';          
rightArmInit = [ -19.7  29.7   0.0  44.9  0.0]';         
leftLegInit  = [  25.5   0.1   0.0 -18.5 -5.5  -0.1]';     
rightLegInit = [  25.5   0.1   0.0 -18.5 -5.5  -0.1]';
 
qjDes  = [torsoInit; leftArmInit; rightArmInit; leftLegInit; rightLegInit]*(pi/180);

%% Update to the initial position
n_tot    = 31;
n_joints = 25;
n_base   = 6;

wbm_updateState(qjDes, zeros(n_joints,1), zeros(n_base,1));

[~,T_b,~,~]     = wbm_getState();
[pos,rot]       = frame2posrot(T_b);

%% Variables definition
PINV_TOL    = 1e-10;
inv_rot     = eye(3)/rot;

%Feet Jacobian
link_name   = {'l_sole', 'r_sole'};
n_constr    = 12;
num_constr  = 2;

Jc          = zeros(n_constr,n_tot);

for ii=1:num_constr
    
   Jc(6*(ii-1)+1:6*ii,:) = wbm_jacobian(inv_rot,pos,qjDes,link_name{ii});

end

%Mass matrix
M = wbm_massMatrix(inv_rot,pos,qjDes);

%Matrix A at CoM
x_lsole = wbm_forwardKinematics(inv_rot,pos,qjDes,'l_sole');
x_rsole = wbm_forwardKinematics(inv_rot,pos,qjDes,'r_sole');
com     = wbm_forwardKinematics(inv_rot,pos,qjDes,'com');

pos_leftFoot    = x_lsole(1:3);
pos_rightFoot   = x_rsole(1:3);
xcom            = com(1:3);

Pr              = pos_rightFoot - xcom;  
Pl              = pos_leftFoot  - xcom;

AL = [ eye(3),  zeros(3);
       Sf(Pl),  eye(3)];
AR = [ eye(3),  zeros(3);
       Sf(Pr),  eye(3) ];
   
A  = [AL, AR];

%% Fixed parameters
g_bar  = 9.81;
m      = M(1,1);
e3     = zeros(n_tot,1);
e3(3)  = 1;

S      = [zeros(n_joints,n_base) eye(n_joints)];
S6     = [eye(n_base) zeros(n_base,n_joints)];
S6t    =  S6.';
S7     = [zeros(n_joints,n_base) eye(n_joints)];

invS6MS6t = eye(n_base)/(S6*M*S6t);
grav      = [0; 0; -m*g_bar; zeros(3,1)];

Minv   = eye(n_tot)/M;
St     = S.';
pinvA  = pinv(A, PINV_TOL);
NA     = eye(n_constr) - pinvA*A;
Jct    = Jc.';
D      = S7 - (S7*M*S6t)*invS6MS6t*S6;

%% Composed variables
Lambda = Jc*Minv*St;
pinvL  = pinv(Lambda, PINV_TOL);
NL     = eye(n_joints) - pinvL*Lambda;

Sigma        = -pinvL*(Jc*Minv*Jct) -NL*D*Jct;
pinvSigmaNA  =  pinv(Sigma*NA, PINV_TOL);

f12   =  pinvA - NA*pinvSigmaNA*(Sigma*pinvA);
f1    = -f12*grav -NA*pinvSigmaNA*(pinvL*Jc*g_bar*e3 + NL*D*M*g_bar*e3);

F     =  pinvL*Jc*Minv + NL*D; 
f22   =  NA*pinvSigmaNA;

P     =  Jc*g_bar*e3 - Jc*Minv*Jct*f1;
P0    =  D*(M*g_bar*e3 - Jct*f1);

R1    =  F*Jct*f12; 
R2    =  (eye(n_joints) + F*Jct*f22)*NL; 

% Torques at equilibrium
tau_reg = pinvL*P + NL*P0;
save('tau_reg','tau_reg')

%% Initial gains
gainsPCOM           = diag([ 50   50  50]);
gainsDCOM           = 2*sqrt(gainsPCOM);
gainMomentum        = 1;

impTorso            = [ 50    50   50
                         0     0    0]; 
                           
impArms             = [ 10    10    10   10  5  
                         0     0    0    0   0];
                        
impLeftLeg          = [ 35   50    0.1   30   2   10
                         0    0     0     0   0    0]; 

impRightLeg         = [35   50    0.1    30   2   10
                        0    0     0      0   0    0]; 

dampings            = 1*ones(1,n_joints);
impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
     
Kimp                = diag(impedances);  
Kder                = diag(dampings);
 
Kp = gainsPCOM;
Kd = gainsDCOM;
Kg = gainMomentum*eye(3);

%% Terms for analitical derivatives
% Transformation to euler angles
[T_bar,~]  = parametrization(rot);

T_tilde    = [eye(3) zeros(3);
              zeros(3) T_bar];
          
T          = [T_tilde         zeros(n_base,n_joints);
              zeros(n_joints,n_base), eye(n_joints)];
          
% vb conversion in dqj
Jc_b       =  Jc(1:n_base, 1:n_base);
Jc_q       =  Jc(1:n_base, n_base+1:end);

conv_vb    = -(eye(n_base)/Jc_b)*Jc_q;

% CoM jacobian
Jcom       = wbm_jacobian(inv_rot,pos,qjDes,'com');

% Hw jacobian
Jwb        = zeros(3,n_base);
Jwqj       = zeros(3,n_joints);

for ii = 1:n_joints

    dot_qj     = zeros(n_joints,1);
    dot_qj(ii) = 1;
    
    H = wbm_centroidalMomentum(inv_rot,pos,qjDes,dot_qj,zeros(n_base,1));
     
    Jwqj(:,ii) = H(4:6);
    
end

for ii = 1:6

    vb     = zeros(n_base,1);
    vb(ii) = 1;
    
    H = wbm_centroidalMomentum(inv_rot,pos,qjDes,zeros(n_joints,1),vb);
    
    Jwb(:,ii) = H(4:6);
    
end

%% Needed parameters
lparam.delta    = 0.00001;
lparam.qjDes    = qjDes;
lparam.n_joints = n_joints;
lparam.n_base   = n_base;

lparam.g_bar       = g_bar;
lparam.e3          = e3;
lparam.St          = St;
lparam.S6          = S6;
lparam.S6t         = S6t;
lparam.S7          = S7;
lparam.grav        = grav;
lparam.n_tot       = n_tot;
lparam.n_base      = n_base;
lparam.n_joints    = n_joints;
lparam.PINV_TOL    = PINV_TOL;
lparam.link_name   = link_name;
lparam.n_constr    = n_constr;
lparam.num_constr  = num_constr;
lparam.m           = m;

%% Numerical derivative of tau_reg
dtau_ng        = zeros(n_joints);

for jj = 1:n_joints

numerical_tau  = num_der(jj, lparam);
dtau_ng(:,jj)  = numerical_tau;
 
end

%% Others derivatives with respect of position
Jcom_a      = Jcom*T;

Jcom_a_linb = Jcom_a(1:3,1:n_base);
Jcom_a_linq = Jcom_a(1:3,n_base+1:end);

total_Jcom  = Jcom_a_linb*(eye(n_base)/T_tilde)*conv_vb + Jcom_a_linq;
 
dqHdot      = [-m*Kp*total_Jcom; zeros(3,n_joints)];
    
%total stiffness
KS = dtau_ng -R1*dqHdot -R2*Kimp;
save('KS','KS')

%% Others derivatives with respect of velocity
Jcom_lin   = Jcom(1:3,:);
Jcom_lin_b = Jcom_lin(:,1:n_base);
Jcom_lin_q = Jcom_lin(:,n_base+1:end);

xD_der   = Jcom_lin_b*conv_vb + Jcom_lin_q;
Hw_der   = Jwb*conv_vb + Jwqj;

dqDHdot  = [-m*Kd*xD_der; -Kg*Hw_der];

%total damping
KD = -R1*dqDHdot -R2*Kder;
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

R11  = -R1*[-m*eye(3); zeros(3)];
R12  =  total_Jcom;

S11  = -R2;
S12  =  eye(n_joints);

[Kp_n, ~, Kimp_n] = kronecher_prod(desired_KS,R11,R12,S11,S12,lparam,'position');

KS_new = dtau_ng -R1*[-m*Kp_n*total_Jcom; zeros(3,n_joints)] -R2*Kimp_n;

% damping matrix
desired_KD = -5*eye(n_joints);

R21 = -R1;
R22 =  [xD_der ; Hw_der];

S21 = -R2;
S22 =  eye(n_joints);

[Kd_n, Kg_n, Kder_n] = kronecher_prod(desired_KD,R21,R22,S21,S22,lparam,'velocity');

KD_new = -R1*[-m*Kd_n*xD_der; -Kg_n*Hw_der] -R2*Kder_n;

%% Eigenvalues evaluation
disp('eigenvalues of KS_new and KD_new')
disp([eig(KS_new) eig(KD_new)])

disp('eigenvalues of Kp_n, Kd_n, Kg_n')
disp([eig(Kp_n) eig(Kd_n) eig(Kg_n)])

% disp('eigenvalues of Kimp and Kder')
% disp([eig(Kimp_n) eig(Kder_n)])

gains_tun.Kimp_n = Kimp_n;
gains_tun.Kder_n = Kder_n;
gains_tun.Kp_n   = Kp_n;
gains_tun.Kd_n   = Kd_n;
gains_tun.Kg_n   = Kg_n;

save('gains_tun','gains_tun')



