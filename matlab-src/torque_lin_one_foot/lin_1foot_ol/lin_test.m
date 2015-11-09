clear all
close all
clc

%% Linearization around a given joints position
addpath('./../../mex-wholebodymodel/matlab/utilities');
addpath('./../../mex-wholebodymodel/matlab/wrappers');
addpath('./../../../../build/');
addpath('./../');

%% fixed values and initial conditions
n_tot    = 31;
n_joints = 25;
n_base   = 6;

wbm_modelInitialise('icubGazeboSim');

torsoInit    = [ -10.0   0.0    0.0]';
leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  -0.1]';
rightLegInit = [  25.5   5.0    0.0  -40    -5.5  -0.1]'; 
leftArmInit  = [ -19.7   29.7   0.0   44.9   0.0]';          
rightArmInit = [ -19.7   29.7   0.0   44.9   0.0]';
 
qj0  = [torsoInit; leftArmInit; rightArmInit; leftLegInit; rightLegInit] * (pi/180);
dqj0 = zeros(n_joints,1);
vb0  = zeros(n_base,1);

%update the state to the initial condition
wbm_updateState(qj0,zeros(n_joints,1),zeros(n_base,1));
 
[rot0,pos0]   = wbm_getWorldFrameFromFixedLink('l_sole',qj0);

toll_lambda  = 1e-10;

g_bar    = 9.81;
 
e3       = zeros(n_tot,1);
e3(3)    = 1;
 
S        = [zeros(n_joints,n_base) eye(n_joints)];
St       = S.';
S6       = [eye(n_base)   zeros(n_base,n_joints)];
S6t      =  S6.';
S7       = [zeros(n_joints,n_base) eye(n_joints)];

%% gains
gainsPCOM           = diag([50  50 50]);
gainsDCOM           = diag([  1    1   1]);
gainMomentum        = 1 ; 

impTorso            = [  20    20   20
                           0     0    0]; 

impArms             = [ 13  13   13   5   5
                         0    0    0   0   0 ];

impLeftLeg          = [ 70   70  65  30  10  10
                          0    0   0   0   0   0]; 
                         
impRightLeg         = [ 20   20  20  10  10  10
                          0    0   0   0   0   0];
                         
impedances          = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)]; 
dampings            = 1*ones(1,n_joints);

Kimp = diag(impedances);
Kder = diag(dampings);
 
Kp = gainsPCOM;
Kd = gainsDCOM;
Kg = gainMomentum*eye(3);

%% Position and velocity variation form steady state
rot_tr0  = rot0.';

com0     = wbm_forwardKinematics(rot_tr0,pos0,qj0,'com');
xcom0    = com0(1:3);

space = 500;
toll  = 1.5*pi/180;

q2  = zeros(n_joints,space);
dq2 = q2;

for pp = 1:25

q2(pp,:)  = linspace(qj0(pp)-toll,qj0(pp)+toll,space);
dq2(pp,:) = linspace(dqj0(pp)-toll,dqj0(pp)+toll,space);

end

load('KS')
load('KD')
load('tau_reg')

tau_q     = zeros(n_joints, space);
tau_q_lin = tau_q;

%% Torques equation varying position
kk = 1;

for q = q2
    
qj = q;

wbm_updateState(qj,zeros(n_joints,1),zeros(n_base,1));
 
[rot,pos]   = wbm_getWorldFrameFromFixedLink('l_sole',qj);
     
%% Base variables
rot_tr = rot.';

%Jacobian at feet    
Jc = wbm_jacobian(rot_tr,pos,qj,'l_sole');

%Mass matrix
M  = wbm_massMatrix(rot_tr,pos,qj);

%JcDv
JcDv = wbm_djdq(rot_tr,pos,qj,dqj0,vb0,'l_sole');

%Generalised bias forces
h    = wbm_generalisedBiasForces(rot_tr,pos,qj,dqj0,vb0);

%Centroidal momentum
H    = wbm_centroidalMomentum(rot_tr,pos,qj,dqj0,vb0);

%Matrix A at CoM
x_lsole = wbm_forwardKinematics(rot_tr,pos,qj,'l_sole');
com     = wbm_forwardKinematics(rot_tr,pos,qj,'com');

pos_leftFoot    = x_lsole(1:3);
xcom            = com(1:3);
  
Pl              = pos_leftFoot  - xcom;

A = eye(3);

%% Others parameters 
m        = M(1,1);
grav     = [0; 0; -m*g_bar];

%% Composed variables
Minv   = eye(n_tot)/M;
pinvA  = eye(3)/A;
Jct    = Jc.';

invS6MS6t = eye(n_base)/(S6*M*S6t);
D         = S7 - (S7*M*S6t)*invS6MS6t*S6;

Lambda = Jc*Minv*St;
pinvL  = pinv(Lambda, toll_lambda); 

NL     = eye(n_joints) - pinvL*Lambda;
 
Jcom           = wbm_jacobian(rot_tr,pos,qj,'com');

Jcom_lin       = Jcom(1:3,:);

Jcom_lin_base  = Jcom_lin(:,1:n_base);  
Jcom_lin_qj    = Jcom_lin(:,n_base+1:end);

Jc_base        = Jc(:,1:n_base);
Jc_qj          = Jc(:,n_base+1:end);

conv_vb        = -(eye(n_base)/Jc_base)*Jc_qj;

xDcom    = (Jcom_lin_base*conv_vb + Jcom_lin_qj)*dqj0;
lin_dyn  = -Kp*(xcom-xcom0) -Kd*xDcom;

HDotDes =  m*lin_dyn;

%% nonlinear torques
fl               = pinvA*(HDotDes-grav);

 m               = M(1,1);
 Mb              = M(1:6,1:6);
 Mbj             = M(1:6,7:end);
 qTilde          = qj-qj0;
 qD              = dqj0;

%%
Jcf = Jc([1 2 3],:);
Jcm = Jc([4 5 6],:);

Jcft = Jcf.';
Jcmt = Jcm.';

JcMinv          = Jc/M;
JcMinvSt        = JcMinv*St;
JcMinvJct       = JcMinv*transpose(Jc);

 PInv_JcMinvSt  = pinv(JcMinvSt,toll_lambda);
 
tau_b  =  PInv_JcMinvSt*(JcMinv*h - JcDv);
tau_fl = -PInv_JcMinvSt*(JcMinv*Jcft*fl);
tau_f = tau_b+tau_fl;
Gamma  = -PInv_JcMinvSt*(JcMinv*Jcmt);

NL_tilde = [Gamma NL];

post = (h(7:end) - Mbj'/Mb*h(1:6) - diag(impedances)*qTilde - diag(dampings)*qD) - (Jcft(7:end,:) - Mbj'/Mb*(Jcft(1:6,:)))*fl;

NL_tilde2 = [(Gamma+(Jcmt(7:end,:) - Mbj'/Mb*(Jcmt(1:6,:)))) NL];
pinvNLt2  = pinv(NL_tilde2,toll_lambda);

tau0_tilde = pinvNLt2*(post-tau_f);

tau_q(:,kk) = tau_f + NL_tilde*tau0_tilde;

%% linear torques
tau_q_lin(:,kk) = tau_reg +KS*(qj-qj0) +KD*(dqj0);

kk=kk+1;

wbm_updateState(qj0,zeros(n_joints,1),zeros(n_base,1));

end

%% graphics of linearization with respect of qj
graphics(q2*180/pi,tau_q,tau_q_lin,qj0*180/pi,tau_reg);

%% velocity
%definition of vb
wbm_updateState(qj0,zeros(n_joints,1),zeros(n_base,1));

[rot,pos]   = wbm_getWorldFrameFromFixedLink('l_sole',qj0);

rot_tr    = rot.';
Jc        = wbm_jacobian(rot_tr,pos,qj0,'l_sole');
Jc_base   = Jc(:,1:n_base);
Jc_qj     = Jc(:,n_base+1:end);

conv_vb   = -(eye(n_base)/Jc_base)*Jc_qj;

tau_dq      = zeros(n_joints, space);
tau_dq_lin  = tau_dq;

kk = 1;

for dq = dq2
    
qj  = qj0;

dqj = dq;
vb  = conv_vb*dqj;

%update the state
wbm_updateState(qj,dqj,vb);
 
[rot,pos]   = wbm_getWorldFrameFromFixedLink('l_sole',qj);
     
%% Base variables
rot_tr = rot.';

%Jacobian at feet    
Jc = wbm_jacobian(rot_tr,pos,qj,'l_sole');

%Mass matrix
M  = wbm_massMatrix(rot_tr,pos,qj);

%JcDv
JcDv = wbm_djdq(rot_tr,pos,qj,dqj,vb,'l_sole');

%gen. bias forces
h    = wbm_generalisedBiasForces(rot_tr,pos,qj,dqj,vb);

%centroidal momentum
H    = wbm_centroidalMomentum(rot_tr,pos,qj,dqj,vb);

%Matrix A at CoM
% x_lsole = wbm_forwardKinematics(rot_inv,pos,qj,'l_sole');
% com     = wbm_forwardKinematics(rot_inv,pos,qj,'com');

% pos_leftFoot    = x_lsole(1:3);
  xcom            = com(1:3);
  
% Pl              = pos_leftFoot  - xcom;

% A = [ eye(3),  zeros(3);
%        Sf(Pl),  eye(3)];

A = eye(3);

%% Others parameters 
m        = M(1,1);
grav     = [0; 0; -m*g_bar];

%% Composed variables
Minv   = eye(n_tot)/M;
pinvA  = eye(3)/A;
Jct    = Jc.';

invS6MS6t = eye(n_base)/(S6*M*S6t);
D         = S7 - (S7*M*S6t)*invS6MS6t*S6;

Lambda = Jc*Minv*St;
pinvL  = pinv(Lambda, toll_lambda); 

NL     = eye(n_joints) - pinvL*Lambda;
 
Jcom           = wbm_jacobian(rot_tr,pos,qj,'com');

Jcom_lin       = Jcom(1:3,:);

Jcom_lin_base  = Jcom_lin(:,1:n_base);  
Jcom_lin_qj    = Jcom_lin(:,n_base+1:end);

Jc_base        = Jc(:,1:n_base);
Jc_qj          = Jc(:,n_base+1:end);

conv_vb        = -(eye(n_base)/Jc_base)*Jc_qj;

xDcom    = (Jcom_lin_base*conv_vb + Jcom_lin_qj)*dqj;
lin_dyn  = -Kp*(xcom-xcom0) -Kd*xDcom;

HDotDes  =  m*lin_dyn;

%% nonlinear torques
fl               = pinvA*(HDotDes-grav);

 m               = M(1,1);
 Mb              = M(1:6,1:6);
 Mbj             = M(1:6,7:end);
 qTilde          = qj-qj0;
 qD              = dqj;

%%
Jcf = Jc([1 2 3],:);
Jcm = Jc([4 5 6],:);

Jcft = Jcf.';
Jcmt = Jcm.';

JcMinv          = Jc/M;
JcMinvSt        = JcMinv*St;
JcMinvJct       = JcMinv*transpose(Jc);

 PInv_JcMinvSt  = pinv(JcMinvSt,toll_lambda);
 
tau_b  =  PInv_JcMinvSt*(JcMinv*h - JcDv);
tau_fl = -PInv_JcMinvSt*(JcMinv*Jcft*fl);
tau_f  =  tau_b+tau_fl;
Gamma  = -PInv_JcMinvSt*(JcMinv*Jcmt);

NL_tilde = [Gamma NL];

post = (h(7:end) - Mbj'/Mb*h(1:6) - diag(impedances)*qTilde - diag(dampings)*qD) - (Jcft(7:end,:) - Mbj'/Mb*(Jcft(1:6,:)))*fl;

NL_tilde2 = [(Gamma+(Jcmt(7:end,:) - Mbj'/Mb*(Jcmt(1:6,:)))) NL];
pinvNLt2  = pinv(NL_tilde2,toll_lambda);

tau0_tilde = pinvNLt2*(post-tau_f);

tau_dq(:,kk) = tau_f + NL_tilde*tau0_tilde;

%% linear torques
tau_dq_lin(:,kk) = tau_reg +KS*(qj-qj0) +KD*(dqj);

kk=kk+1;

wbm_updateState(qj0,zeros(n_joints,1),zeros(n_base,1));

end

%% graphics of linearization with respect of dqj
% graphics(dq2*180/pi,tau_dq,tau_dq_lin,dqj0*180/pi,tau_reg);

