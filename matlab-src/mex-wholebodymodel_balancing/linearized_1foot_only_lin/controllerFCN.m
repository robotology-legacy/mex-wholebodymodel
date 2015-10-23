function [errorCoM, tau_ol,f0, tau2]   =  ...
          controllerFCN (LEFT_RIGHT_FOOT_IN_CONTACT, DOF, ...
          q, qDes, v, M, h, Jc, JcDv, xcom, J_CoM, desired_x_dx_ddx_CoM,...
          gainsPCOM, gainsDCOM, impedances, dampings, qDes0, xcom0)

% this is the function that computes the desired contact forces and torques
% at joints. There's also the possibility to use QP program to calculate f0 
ROBOT_DOF       = DOF;
constraints     = LEFT_RIGHT_FOOT_IN_CONTACT;

%% tolerances for pseudoinverse and QP
 PINV_TOL        = 1e-10;
%reg             = 0.01;

%% others variables
gravAcc          = 9.81;

m               = M(1,1);
n_joints        = DOF;
n_base          = 6;

S     = [zeros(n_joints,n_base) eye(n_joints)];
St    = S.';
S6    = [eye(n_base)  zeros(n_base, n_joints)];
S6t   = S6.';
S7    = [zeros(n_joints,n_base) eye(n_joints)];

invS6MS6t = eye(n_base)/(S6*M*S6t);
D         = S7 - (S7*M*S6t)*invS6MS6t*S6;

grav            = [ zeros(2,1);
                   -m*gravAcc];
               
xDcom           = J_CoM(1:3,:)*v;
qD              = v(7:end);
qTilde          = q-qDes;

xDDcomStar      = desired_x_dx_ddx_CoM(:,3) - gainsPCOM*(xcom  - desired_x_dx_ddx_CoM(:,1))...
                  - gainsDCOM*(xDcom - desired_x_dx_ddx_CoM(:,2));

%% corrections for only one foot on the ground
if     constraints == 1  
    
    A      = eye(3);
    
    pinvA  = eye(3);

elseif constraints == 2

    A      = [eye(3) eye(3)];
   
    pinvA  = pinv( A, PINV_TOL);
    
end

%% terms from the constraint equation
JcMinv          = Jc/M;
JcMinvSt        = JcMinv*St;

 PInv_JcMinvSt  = pinv(JcMinvSt, PINV_TOL);
%PInv_JcMinvSt  = JcMinvSt'/(JcMinvSt*JcMinvSt' + reg*eye(size(JcMinvSt,1)));

if    constraints == 1
    
    Jcf =  Jc(1:3,:);
    Jcft = Jcf.';
    
    Jcm =  Jc(4:6,:);
    Jcmt = Jcm.';
    
elseif constraints == 2
    
    Jcf  =  Jc([1 2 3 7 8 9],:);
    Jcft =  Jcf.';
    
    Jcm  =  Jc([4 5 6 10 11 12],:);
    Jcmt =  Jcm.';
    
end

HDotDes         = m*xDDcomStar;
NL              = eye(ROBOT_DOF) - PInv_JcMinvSt*JcMinvSt;
f_HDot          = pinvA*(HDotDes - grav);

%% corrections for only one foot on the ground
NA              =  zeros(3);

if     constraints == 1
     
fl_ini     =  f_HDot; 
    
elseif constraints == 2
    
NA         =  eye(6,6)-pinvA*A;
    
fl_ini     =  f_HDot;
   
end

%% construction of torques
Gamma       = -PInv_JcMinvSt*(JcMinv*Jcmt);
NL_tilde    =  [Gamma NL];
NL_tilde2   =  [(Gamma+D*Jcmt) NL];
pinvNL_t2   =  pinv(NL_tilde2, PINV_TOL);

if     constraints == 1

f0          =  zeros(3,1);

elseif constraints == 2
    
G           = -PInv_JcMinvSt*JcMinv*Jcft;
tau_f0      =  PInv_JcMinvSt*(JcMinv*h -JcDv) -PInv_JcMinvSt*(JcMinv*Jcft*fl_ini);
post_f0     =  D*(h-Jcft*fl_ini) -diag(impedances)*qTilde -diag(dampings)*qD;
R           = -D*Jcmt*NA;
H           =  G + NL_tilde*pinvNL_t2*(R-G);

tau_f0_fin  = tau_f0 + NL_tilde*pinvNL_t2*(post_f0 - tau_f0);       
f0          = pinv(H,PINV_TOL)*tau_f0_fin;

end

fl         =  fl_ini + NA*f0;

tau_base   =  PInv_JcMinvSt*(JcMinv*h -JcDv) -PInv_JcMinvSt*(JcMinv*Jcft*fl);

post       =  D*(h-Jcft*fl) -diag(impedances)*qTilde -diag(dampings)*qD;

tau0_tilde =  pinvNL_t2*(post-tau_base);

%% Definition of torques and contact forces

 tau_ol    = tau_base + NL_tilde*tau0_tilde;
 
 errorCoM  = xcom - desired_x_dx_ddx_CoM(:,1);
 
 
%% Linearized system
load('KS')
load('KD')
load('tau_reg')

tau_lin       = tau_reg + KS*(q-qDes0) + KD*qD;

xDDcomStar2   = - gainsPCOM*(xcom  - xcom0)- gainsDCOM*(xDcom);

HDotDes2   =  m*xDDcomStar2;
fl_ini2    =  pinvA*(HDotDes2 - grav);
fl         =  fl_ini2 + NA*f0;
tau_base   =  PInv_JcMinvSt*(JcMinv*h -JcDv) -PInv_JcMinvSt*(JcMinv*Jcft*fl);

post       =  D*(h-Jcft*fl) -diag(impedances)*(q-qDes0) -diag(dampings)*qD;

tau0_tilde =  pinvNL_t2*(post-tau_base);

tau_nonl   = tau_base + NL_tilde*tau0_tilde;

tau2       = [tau_nonl tau_lin];
 
 end