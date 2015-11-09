function [tauModel, Sigma, NA, fHdotDesC1C2, errorCoM, f0]   =  ...
          controllerFCN (LEFT_RIGHT_FOOT_IN_CONTACT, DOF, USE_QP_SOLVER, ConstraintsMatrix, bVectorConstraints,...
          q, qDes, v, M, h, H, posLeftFoot, posRightFoot, footSize, Jc, JcDv, xcom, J_CoM, desired_x_dx_ddx_CoM,...
          gainsPCOM, gainsDCOM, gainMomentum, impedances, dampings)

% this is the function that computes the desired contact forces and torques
% at joints. There's also the possibility to use QP program to calculate f0 
e1              = [1;0;0];
e2              = [0;1;0];
e3              = [0;0;1];

ROBOT_DOF       = DOF;
constraints     = LEFT_RIGHT_FOOT_IN_CONTACT;

%% tolerances for pseudoinverse and QP
 PINV_TOL        = 1e-10;
 regHessiansQP   = 1e-5;
%reg             = 0.0001;

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

%% 
tauModel         = PInv_JcMinvSt*(JcMinv*h - JcDv) + NL*(h(7:end) - Mbj'/Mb*h(1:6) - diag(impedances)*qTilde - diag(dampings)*qD);

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

  f0                  =  zeros(6,1);

elseif constraints == 2 && USE_QP_SOLVER == 0 

f0                    = -pinv(SigmaNA, PINV_TOL)*(tauModel+Sigma*f_HDot);

end

%f                         = f_HDot + NA*f0;
errorCoM                   = xcom - desired_x_dx_ddx_CoM(:,1);

