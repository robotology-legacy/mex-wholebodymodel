function [tauModel, Sigma, NA, fHdotDesC1C2, errorCoM, f0,tau_c,f_c,dqj_des]   =  ...
          controllerFCN (LEFT_RIGHT_FOOT_IN_CONTACT, DOF, USE_QP_SOLVER, ConstraintsMatrix, bVectorConstraints,...
          q, qDes, v, M, h, H, posLeftFoot, posRightFoot, footSize, Jc, JcDv, xcom, J_CoM, desired_x_dx_ddx_CoM,...
          gainsPCOM, gainsDCOM, gainMomentum, impedances, dampings, pos_feet, lfoot_ini, rfoot_ini,qDes2,gen)

% this is the function that computes the desired contact forces and torques
% at joints. There's also the possibility to use QP program to calculate f0 
e1              = [1;0;0];
e2              = [0;1;0];
e3              = [0;0;1];

ROBOT_DOF       = DOF;
constraints     = LEFT_RIGHT_FOOT_IN_CONTACT;

%% tolerances for pseudoinverse and QP
 PINV_TOL        = 1e-10;
 regHessiansQP   = 1e-3;
%reg             = 0.01;

%% others variables
gravAcc          = 9.81;

 m               = M(1,1);
 Mb              = M(1:6,1:6);
 Mbj             = M(1:6,7:end);
%Mj              = M(7:end,7:end);

St              = [ zeros(6,ROBOT_DOF);
                    eye(ROBOT_DOF,ROBOT_DOF)];

grav            = [ zeros(2,1);
                   -m*gravAcc;
                    zeros(3,1)];
  
pos_leftFoot    = posLeftFoot(1:3);
rotMatLeftFoot  = Rf(posLeftFoot(4:7));

pos_rightFoot   = posRightFoot(1:3);
rotMatRightFoot = Rf(posRightFoot(4:7));

xDcom           = J_CoM(1:3,:)*v;
qD              = v(7:end);

xDDcomStar      = desired_x_dx_ddx_CoM(:,3) - gainsPCOM*(xcom  - desired_x_dx_ddx_CoM(:,1))...
                  - gainsDCOM*(xDcom - desired_x_dx_ddx_CoM(:,2));

Pr              = pos_rightFoot - xcom;   % Application point of the contact force on the right foot w.r.t. CoM
Pl              = pos_leftFoot  - xcom;   % Application point of the contact force on the left foot w.r.t. CoM

AL = [ eye(3),  zeros(3);
       Sf(Pl),  eye(3)];
AR = [ eye(3),  zeros(3);
       Sf(Pr),  eye(3) ];

%% corrections for only one foot on the ground
if     constraints == 1  
    
    A      = AL;
    
    pinvA  = eye(6)/A;

elseif constraints == 2

    A      = [AL, AR];
   
    pinvA  = pinv( A, PINV_TOL);
    
end

%% terms from the constraint equation
JcMinv          = Jc/M;
JcMinvSt        = JcMinv*St;
JcMinvJct       = JcMinv*transpose(Jc);

 PInv_JcMinvSt  = pinv(JcMinvSt, PINV_TOL);
%PInv_JcMinvSt  = JcMinvSt'/(JcMinvSt*JcMinvSt' + reg*eye(size(JcMinvSt,1)));

%% CoM controller and desired contact forces
HDotDes_w       = -gainMomentum*H(4:end);

HDotDes         = [  m*xDDcomStar;
                     HDotDes_w  ];
                      
NL              = eye(ROBOT_DOF) - PInv_JcMinvSt*JcMinvSt;

f_HDot          = pinvA*(HDotDes - grav);

%% corrections for only one foot on the ground
NA               =  zeros(6);

if     constraints == 1 && USE_QP_SOLVER == 0
     
fHdotDesC1C2     =  f_HDot; 
    
elseif constraints == 2
    
NA               =  eye(12,12)-pinvA*A;
    
fHdotDesC1C2     =  f_HDot;
   
end

%%
JBar             =  transpose(Jc(:,7:end)) - Mbj'/Mb*transpose(Jc(:,1:6));   % multiplier of f in tau0
qTilde           =  q-qDes;

Sigma            = -(PInv_JcMinvSt*JcMinvJct + NL*JBar);
SigmaNA          =  Sigma*NA;

%% adding a correction term in the costraint equation
% this is necessary to reduce the numerical errors in the costraint
% equation. 
 k_corr_pos = 5;
 k_corr_vel = 2*sqrt(k_corr_pos);
 
 if     constraints == 1
     
 pos_feet_delta = [(pos_leftFoot-lfoot_ini(1:3)); (pos_feet(4:6))];
 
 elseif constraints == 2
     
 pos_feet_delta = [(pos_leftFoot-lfoot_ini(1:3)); (pos_feet(4:6));...
                   (pos_rightFoot-rfoot_ini(1:3));(pos_feet(10:12))];
               
 end
 
 tauModel   = PInv_JcMinvSt*(JcMinv*h - JcDv -k_corr_vel.*Jc*v -k_corr_pos.*pos_feet_delta) + ... 
              NL*(h(7:end) - Mbj'/Mb*h(1:6) - diag(impedances)*qTilde - diag(dampings)*qD);

%tauModel   = PInv_JcMinvSt*(JcMinv*h - JcDv) + NL*(h(7:end) - Mbj'/Mb*h(1:6) - diag(impedances)*qTilde - diag(dampings)*qD);

%% QP solver
CL               = ConstraintsMatrix; 
CL(end-4,1:3)    = -e3'*rotMatLeftFoot;                
CL(end-3,:)      = [ footSize(1,1)*e3'*rotMatLeftFoot', e2'*rotMatLeftFoot'];
CL(end-2,:)      = [-footSize(1,2)*e3'*rotMatLeftFoot',-e2'*rotMatLeftFoot'];

CL(end-1,:)      = [ footSize(2,1)*e3'*rotMatLeftFoot',-e1'*rotMatLeftFoot'];
CL(end  ,:)      = [-footSize(2,2)*e3'*rotMatLeftFoot', e1'*rotMatLeftFoot'];

CR               = ConstraintsMatrix;
CR(end-4,1:3)    = -e3'*rotMatRightFoot;  
CR(end-3,:)      = [ footSize(1,1)*e3'*rotMatRightFoot', e2'*rotMatRightFoot'];
CR(end-2,:)      = [-footSize(1,2)*e3'*rotMatRightFoot',-e2'*rotMatRightFoot'];

CR(end-1,:)      = [ footSize(2,1)*e3'*rotMatRightFoot',-e1'*rotMatRightFoot'];
CR(end  ,:)      = [-footSize(2,2)*e3'*rotMatRightFoot', e1'*rotMatRightFoot'];


ConstraintsMatrix2Feet    = blkdiag(CL,CR);
bVectorConstraints2Feet   = [bVectorConstraints;bVectorConstraints];

ConstraintsMatrixQP1Foot  = CL;
bVectorConstraintsQP1Foot = bVectorConstraints;

%% corrections for only one foot on the ground
if  constraints == 1 && USE_QP_SOLVER == 1
    
A1Foot                     =  AL;
HessianMatrixQP1Foot       =  A1Foot'*A1Foot;
gradientQP1Foot            = -A1Foot'*(HDotDes - grav);

[fHdotDesC1C2, ~, exitFlag, iter, lambda, auxOutput] = qpOASES (HessianMatrixQP1Foot, gradientQP1Foot, ConstraintsMatrixQP1Foot,...
                                                                [], [], [], bVectorConstraintsQP1Foot);           

  if exitFlag ~= 0
      
  disp('QP failed with');
  disp( exitFlag)
  disp( iter)
  disp( auxOutput)
  disp( lambda)
  error('qp_failed')
      
  end
 
elseif constraints == 2 && USE_QP_SOLVER == 1

ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*NA;
bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*f_HDot;

HessianMatrixQP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*regHessiansQP;
gradientQP2Feet           = SigmaNA'*(tauModel + Sigma*f_HDot);

[f0, ~, exitFlag, iter, lambda, auxOutput] = qpOASES (HessianMatrixQP2Feet, gradientQP2Feet, ConstraintsMatrixQP2Feet,...
                                                           [], [], [], bVectorConstraintsQp2Feet);           
            
 if exitFlag ~= 0

   disp('QP failed');
   disp(exitFlag);
   disp(iter);
   disp(auxOutput);
   disp(lambda);
   error('qp_failed')
    
 end

end

%% desired f0 no QP
if     constraints == 1

f0                    =  zeros(6,1);

elseif constraints == 2 && USE_QP_SOLVER == 0 

f0                    = -pinv(SigmaNA, PINV_TOL)*(tauModel+Sigma*f_HDot);

end

%f                    = f_HDot + NA*f0;
errorCoM              = xcom- desired_x_dx_ddx_CoM(:,1);

%% Joint space control
Jct = Jc.';

% inv_base =  (eye(6)/Jc(1:6,1:6));
% J_total  = -J_CoM(1:3,1:6)*inv_base*Jc(1:6,7:end) + J_CoM(1:3,7:end);

  J_total  =  J_CoM(1:3,7:end);
%    J_total  =  J_CoM(1:3,:);

%   NJ = eye(25) - pinv(J_total,PINV_TOL)*J_total;
  
  dqj_des  =  pinv(J_total,PINV_TOL)*(desired_x_dx_ddx_CoM(:,2));
  
%   dqj_des = t_dqj_des(7:end) ;
  
% qddj_des =  pinv(J_total,PINV_TOL)*desired_x_dx_ddx_CoM(:,3);

% qTilde2 = q - qDes2;

qDesini = q;

qDes3  = qDes + pinv(J_total,PINV_TOL)*(errorCoM);

qTilde2 = q -qDes3;


qD2     = v(7:end); %;- dqj_des;
 
[errorCoM (xDcom - desired_x_dx_ddx_CoM(:,2))]

% Mb    = M(1:6,1:6);
% Mjb   = M(7:end,1:6);
% M_bar = M(7:end,7:end)-Mjb*(eye(6)/Mb)*Mbj;

%Version 1

% tau_nf   = PInv_JcMinvSt*(JcMinv*(h-Jct*pinvA*(-grav))-JcDv -k_corr_vel.*Jc*v -k_corr_pos.*pos_feet_delta);
% 
% NL_tilde = [(-PInv_JcMinvSt*(JcMinvJct)+JBar) NL];
% 
% reg      = 1e-10;
% 
% % PIN_NL_tilde  = NL_tilde'/(NL_tilde*NL_tilde' + reg*eye(size(NL_tilde,1)));
%   PIN_NL_tilde  = pinv(NL_tilde, reg);
% 
% % NLL           = eye(25+6*constraints) - PIN_NL_tilde*NL_tilde;
% 
% % tau00         = [pinvA*(HDotDes); zeros(25,1)];
% 
% post =  (h(7:end) - Mbj'/Mb*h(1:6) - diag(impedances)*qTilde2 - diag(dampings)*qD2)-JBar*pinvA*(-grav);
% 
% 
% tau0_tilde  = PIN_NL_tilde*(post-tau_nf); % +NLL*tau00;
% 
% f_c2        = tau0_tilde(1:6*constraints);
% f_c         = f_c2 - pinvA*grav;
% tau_c       = tau_nf + NL*tau0_tilde(6*constraints+1:end) + (-PInv_JcMinvSt*(JcMinvJct))*f_c2;

%Version 2

 h_new = (eye(31) -Jct*(eye(6*constraints)/(JcMinvJct))*JcMinv)*h;
 v_new = Jct*(eye(12)/(JcMinvJct))*JcDv;
 B     = (eye(31) -Jct*(eye(6*constraints)/(JcMinvJct))*JcMinv)*St;
 
 gen_new = (eye(31) -Jct*(eye(6*constraints)/(JcMinvJct))*JcMinv)*gen; 
 
 Bj    = B(7:end,:);
 Bbj   = B(1:6,:);
 
 Mb    = M(1:6,1:6);
 Mjb   = M(7:end,1:6);
 
 B_bar = Bj -Mjb*(eye(6)/Mb)*Bbj;
 
% tau_c  =  pinv(B_bar,PINV_TOL)*((h_new(7:end)+v_new(7:end)-Mjb*(eye(6)/Mb)*(h_new(1:6)+v_new(1:6)))+ (-diag(impedances)*qTilde2 -diag(dampings)*qD2));
tau_c  =  pinv(B_bar,PINV_TOL)*(gen_new(7:end)-Mjb*(eye(6)/Mb)*(gen_new(1:6))+(-diag(impedances)*10*qTilde2 -diag(dampings)*qD2));
f_c    = (eye(6*constraints)/(JcMinvJct))*(JcMinv*h - JcMinv*St*tau_c -JcDv);
 

