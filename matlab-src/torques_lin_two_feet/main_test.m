clear all
close all
clc

%% fixed values and initial conditions
n_tot    = 31;
n_joints = 25;
n_base   = 6;

wbm_modelInitialise('icubGazeboSim');
wbm_setWorldLink('l_sole',eye(3),[0 0 0]',[0,0,-9.81]');

torsoInit    = [ -10.0   0.0   0.0]';                   
leftArmInit  = [ -19.7  29.7   0.0  44.9  0.0]';          
rightArmInit = [ -19.7  29.7   0.0  44.9  0.0]';         
leftLegInit  = [  25.5   0.1   0.0 -18.5 -5.5  -0.1]';     
rightLegInit = [  25.5   0.1   0.0 -18.5 -5.5  -0.1]';
 
qj0    = [torsoInit; leftArmInit; rightArmInit; leftLegInit; rightLegInit]*(pi/180);
dqj0   = zeros(n_joints,1);
vb0    = zeros(n_base,1);

% Update the state to the initial condition
wbm_updateState(qj0,zeros(n_joints,1),zeros(n_base,1));
 
[~,T_b0,~,~]     = wbm_getState();
[pos0,rot0]      = frame2posrot(T_b0);

PINV_TOL = 1e-10;
g_bar    = 9.81;
e3       = zeros(n_tot,1);
e3(3)    = 1;
 
S        = [zeros(n_joints,n_base) eye(n_joints)];
St       = S.';
S6       = [eye(n_base)   zeros(n_base,n_joints)];
S6t      = S6.';
S7       = [zeros(n_joints,n_base) eye(n_joints)];

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

%% Initial position of CoM
rot_inv0   = eye(3)/rot0;
link_name  = {'l_sole', 'r_sole'};
n_constr   = 12;
num_constr = 2;

com0     = wbm_forwardKinematics(rot_inv0,pos0,qj0,'com');
xcom0    = com0(1:3);

space = 500;
toll  = 0.5*pi/180;

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
 
[~,T_b,~,~]     = wbm_getState();
[pos,rot]       = frame2posrot(T_b);
     
%% Base variables
inv_rot     = eye(3)/rot;

%Feet Jacobian
Jc          = zeros(n_constr,n_tot);

for ii=1:num_constr
    
   Jc(6*(ii-1)+1:6*ii,:)   = wbm_jacobian(inv_rot,pos,qj,link_name{ii});
   %dJcdv
   JcDv(6*(ii-1)+1:6*ii,:) = wbm_djdq(inv_rot,pos,qj,dqj0,vb0,'l_sole');

end

%Mass matrix
M  =  wbm_massMatrix(inv_rot,pos,qj);

%Matrix A at CoM
x_lsole = wbm_forwardKinematics(inv_rot,pos,qj,'l_sole');
x_rsole = wbm_forwardKinematics(inv_rot,pos,qj,'r_sole');
com     = wbm_forwardKinematics(inv_rot,pos,qj,'com');

%Generalised bias forces
h    = wbm_generalisedBiasForces(inv_rot,pos,qj,dqj0,vb0);

%Centroidal momentum
H    = wbm_centroidalMomentum(inv_rot,pos,qj,dqj0,vb0);

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
m         = M(1,1);
invS6MS6t = eye(n_base)/(S6*M*S6t);
grav      = [0; 0; -m*g_bar; zeros(3,1)];

Minv   = eye(n_tot)/M;
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

Jc_b         = Jc(1:n_base, 1:n_base);
Jc_q         = Jc(1:n_base, n_base+1:end);
conv_vb      = -(eye(n_base)/Jc_b)*Jc_q;
          
% CoM jacobian
Jcom         = wbm_jacobian(inv_rot,pos,qj,'com');  
Jcom_bar     = Jcom(1:3,1:n_base)*conv_vb + Jcom(1:3,n_base+1:end);

xDcom        =  Jcom_bar*dqj0;
des_x        = -Kp*(xcom-xcom0) -Kd*xDcom;
HDotDes      =  [m*des_x; -Kg*H(4:end)];

f_ini  =  pinvA*(HDotDes-grav); 
f0     = -pinvSigmaNA*(pinvL*(Jc*Minv*(h-Jct*f_ini)-JcDv) +NL*(D*(h-Jct*f_ini)-Kimp*(qj-qj0)-Kder*(dqj0)));
f_c    =  pinvA*(HDotDes-grav) +NA*f0;

% Torques at equilibrium
tau_q(:,kk) = pinvL*(Jc*Minv*(h-Jct*f_c)-JcDv) +NL*(D*(h-Jct*f_c)-Kimp*(qj-qj0)-Kder*(dqj0)) ;

%% linear torques
tau_q_lin(:,kk) = tau_reg +KS*(qj-qj0) +KD*(dqj0);

kk=kk+1;

wbm_updateState(qj0,zeros(n_joints,1),zeros(n_base,1));

end

%% graphics of linearization with respect of qj
graphics(q2*180/pi,tau_q,tau_q_lin,qj0*180/pi,tau_reg);

%%
for q = q2
    
qj = q;

wbm_updateState(qj,zeros(n_joints,1),zeros(n_base,1));
 
[~,T_b,~,~]     = wbm_getState();
[pos,rot]       = frame2posrot(T_b);
     
%% Base variables
inv_rot     = eye(3)/rot;

%Feet Jacobian
Jc          = zeros(n_constr,n_tot);

for ii=1:num_constr
    
   Jc(6*(ii-1)+1:6*ii,:)   = wbm_jacobian(inv_rot,pos,qj,link_name{ii});
   %dJcdv
   JcDv(6*(ii-1)+1:6*ii,:) = wbm_djdq(inv_rot,pos,qj,dqj0,vb0,'l_sole');

end

%Mass matrix
M  =  wbm_massMatrix(inv_rot,pos,qj);

%Matrix A at CoM
x_lsole = wbm_forwardKinematics(inv_rot,pos,qj,'l_sole');
x_rsole = wbm_forwardKinematics(inv_rot,pos,qj,'r_sole');
com     = wbm_forwardKinematics(inv_rot,pos,qj,'com');

%Generalised bias forces
h    = wbm_generalisedBiasForces(inv_rot,pos,qj,dqj0,vb0);

%Centroidal momentum
H    = wbm_centroidalMomentum(inv_rot,pos,qj,dqj0,vb0);

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
m         = M(1,1);
invS6MS6t = eye(n_base)/(S6*M*S6t);
grav      = [0; 0; -m*g_bar; zeros(3,1)];

Minv   = eye(n_tot)/M;
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

Jc_b         = Jc(1:n_base, 1:n_base);
Jc_q         = Jc(1:n_base, n_base+1:end);
conv_vb      = -(eye(n_base)/Jc_b)*Jc_q;
          
% CoM jacobian
Jcom         = wbm_jacobian(inv_rot,pos,qj,'com');  
Jcom_bar     = Jcom(1:3,1:n_base)*conv_vb + Jcom(1:3,n_base+1:end);

xDcom        =  Jcom_bar*dqj0;
des_x        = -Kp*(xcom-xcom0) -Kd*xDcom;
HDotDes      =  [m*des_x; -Kg*H(4:end)];

f_ini  =  pinvA*(HDotDes-grav); 
f0     = -pinvSigmaNA*(pinvL*(Jc*Minv*(h-Jct*f_ini)-JcDv) +NL*(D*(h-Jct*f_ini)-Kimp*(qj-qj0)-Kder*(dqj0)));
f_c    =  pinvA*(HDotDes-grav) +NA*f0;

% Torques at equilibrium
tau_q(:,kk) = pinvL*(Jc*Minv*(h-Jct*f_c)-JcDv) +NL*(D*(h-Jct*f_c)-Kimp*(qj-qj0)-Kder*(dqj0)) ;

%% linear torques
tau_q_lin(:,kk) = tau_reg +KS*(qj-qj0) +KD*(dqj0);

kk=kk+1;

wbm_updateState(qj0,zeros(n_joints,1),zeros(n_base,1));

end

%% graphics of linearization with respect of qj
graphics(q2*180/pi,tau_q,tau_q_lin,qj0*180/pi,tau_reg);



