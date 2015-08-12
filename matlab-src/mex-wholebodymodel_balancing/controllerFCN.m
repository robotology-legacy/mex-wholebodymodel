function [tauModel,Sigma,NA,fHdotDesC1C2, ...
          HessianMatrixQP1Foot,gradientQP1Foot,ConstraintsMatrixQP1Foot,bVectorConstraintsQp1Foot,...
          HessianMatrixQP2Feet,gradientQP2Feet,ConstraintsMatrixQP2Feet,bVectorConstraintsQp2Feet,...
          errorCoM,qTilde,f0]   =  ...
          controllerFCN (constraints,DOF,ConstraintsMatrix,bVectorConstraints,...
          q,qDes,v, M, h, H, posLeftFoot, posRightFoot,footSize, Jc, JcDv, xcom, J_CoM, desired_x_dx_ddx_CoM,...
          gainsPCOM, gainsICOM, gainsDCOM, gainMomentum, impedances, dampings, intErrorCoM, ki_int_qtilde, USE_HD_SOLVER,pos_feet)
      
%codegen
e1              = [1;0;0];
e2              = [0;1;0];
e3              = [0;0;1];

ROBOT_DOF       = DOF;
PINV_TOL        = 1e-10;

%reg             = 0.01;
 regHessiansQP   = 1e-3;
%regHessiansQP   = 1e-7;

gravAcc          = 9.81;

 m               = M(1,1);
 Mb              = M(1:6,1:6);
 Mbj             = M(1:6,7:end);
%Mj              = M(7:end,7:end);

St              = [  zeros(6,ROBOT_DOF);
                     eye(ROBOT_DOF,ROBOT_DOF)];

grav            = [ zeros(2,1);
                   -m*gravAcc;
                    zeros(3,1)];
               
pos_leftFoot   = posLeftFoot(1:3);
rotMatLeftFoot = Rf(posLeftFoot(4:7));

pos_rightFoot   = posRightFoot(1:3);
rotMatRightFoot = Rf(posRightFoot(4:7));

xDcom           = J_CoM(1:3,:)*v;
qD              = v(7:end);

xDDcomStar      = desired_x_dx_ddx_CoM(:,3) - gainsPCOM*(xcom  - desired_x_dx_ddx_CoM(:,1))...
                  - gainsICOM*intErrorCoM   - gainsDCOM*(xDcom - desired_x_dx_ddx_CoM(:,2));

Pr              = pos_rightFoot - xcom;   % Application point of the contact force on the right foot w.r.t. CoM
Pl              = pos_leftFoot  - xcom;   % Application point of the contact force on the left foot w.r.t. CoM

AL = [ eye(3),  zeros(3);
       Sf(Pl),  eye(3)];
AR = [ eye(3),  zeros(3);
       Sf(Pr),  eye(3) ];

%% corrections for only one foot on the ground
if sum(constraints)==1
    
    A      = AL;
    pinvA  = pinv( A, PINV_TOL);

elseif sum(constraints)==2

    A      = [AL, AR];
    
   %pinvA  = pinv( A, PINV_TOL)*constraints(1)*constraints(2) +...
   %         [inv(AL);zeros(6)]*constraints(1)*(1-constraints(2)) + [zeros(6);inv(AR)]*constraints(2)*(1-constraints(1)); 
   
    pinvA  = pinv( A, PINV_TOL);
    
end

%%
JcMinv          = Jc/M;
JcMinvSt        = JcMinv*St;
JcMinvJct       = JcMinv*transpose(Jc);

 PInv_JcMinvSt = pinv( JcMinvSt, PINV_TOL);
%PInv_JcMinvSt = JcMinvSt'/(JcMinvSt*JcMinvSt' + reg*eye(size(JcMinvSt,1)));

%% HD controller
HDotDes_w = -gainMomentum*H(4:end);

HDotDes   = [  m*xDDcomStar;
               HDotDes_w    ];

if USE_HD_SOLVER==1
    
    S_w = [zeros(3); eye(3)];
   
else
    
    S_w = zeros(6,3);
    
end
                      
NL               = eye(ROBOT_DOF) - PInv_JcMinvSt*JcMinvSt;

f_HDot           = pinvA*(HDotDes - grav);

%% corrections for only one foot on the ground
if sum(constraints)==1
     
fHdotDesC1C2     =  f_HDot;
NA               =  [zeros(6) pinvA*S_w]; 
    
elseif sum(constraints)==2
    
NA               =  [eye(12,12)-pinvA*A pinvA*S_w];
    
fHdotDesC1C2     =  f_HDot*constraints(1)*constraints(2);
   
end

%%
JBar             =  transpose(Jc(:,7:end)) - Mbj'/Mb*transpose(Jc(:,1:6));   % multiplier of f in tau0
qTilde           =  q-qDes;

Sigma            = -(PInv_JcMinvSt*JcMinvJct + NL*JBar);

SigmaNA          =  Sigma*NA;

%% adding a correction term in the costraint equation
% this is necessary to reduce the numerical error in the costraint
% equation. 
 k_corr_pos = 50;
 k_corr_vel = 2*sqrt(k_corr_pos);

 tauModel       = PInv_JcMinvSt*(JcMinv*h - JcDv -k_corr_vel.*Jc*v -k_corr_pos.*pos_feet) + NL*(h(7:end)...
                  -Mbj'/Mb*h(1:6) - diag(impedances)*qTilde  -ki_int_qtilde -diag(dampings)*qD);
%tauModel       = PInv_JcMinvSt*(JcMinv*h - JcDv) + NL*(h(7:end) - Mbj'/Mb*h(1:6) - diag(impedances)*qTilde  -ki_int_qtilde -diag(dampings)*qD);

%% 
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

ConstraintsMatrixQP1Foot  = constraints(1) * (1 - constraints(2)) * CL + ...
                            constraints(2) * (1 - constraints(1)) * CR;
bVectorConstraintsQp1Foot = bVectorConstraints;

%% corrections for only one foot on the ground
if sum(constraints)==1
    
ConstraintsMatrixQP2Feet  = 0;
bVectorConstraintsQp2Feet = 0;

HessianMatrixQP2Feet      = 0;
gradientQP2Feet           = 0;

elseif sum(constraints)==2

ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*NA;
bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*f_HDot;

HessianMatrixQP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*regHessiansQP;

gradientQP2Feet             = SigmaNA'*(tauModel + Sigma*f_HDot);

end

%%
A1Foot                     =  AL*constraints(1)*(1-constraints(2)) + AR*constraints(2)*(1-constraints(1));
HessianMatrixQP1Foot       =  A1Foot'*A1Foot + eye(size(A1Foot,2))*regHessiansQP;
%gradientQP1Foot           = -A1Foot'*(HDotDes + grav);
 gradientQP1Foot           = -A1Foot'*(HDotDes - grav);

%% desired f0 no QP
%corrections for only one foot on the ground
if sum(constraints)==1

f0                        = -pinv(SigmaNA, PINV_TOL)*(tauModel + Sigma*f_HDot);
f0(1:6)                   =  zeros(6,1);

elseif sum(constraints)==2

f0                       = -pinv(SigmaNA, PINV_TOL)*(tauModel+Sigma*f_HDot);

end

%f                         = f_HDot + NA*f0;
errorCoM                   = xcom - desired_x_dx_ddx_CoM(:,1);

