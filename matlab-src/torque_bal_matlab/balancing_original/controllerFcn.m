function [f_c,tau,errorCoM,f0]   = controllerFcn(param, constraintParam, Nu, M, h, H, feetParam, Jc,...
                                                 dJcNu, xCom, J_CoM, desired_x_dx_ddx_CoM, gainParam)
%% controllerFCN
% Generates the desired control torques from a given robot state. 
% It also uses a QP solver for the minimization of torques by means of contact forces' null space  
LEFT_RIGHT_FOOT_IN_CONTACT = param.feet_on_ground;
DOF                        = param.ndof;
USE_QP_SOLVER              = param.QP_solver;

ConstraintsMatrix  = constraintParam.ConstraintsMatrix;
bVectorConstraints = constraintParam.bVectorConstraints ;
footSize           = constraintParam.footSize;

%% Variables and tolerances definition
 PINV_TOL        = 1e-8;
 regHessiansQP   = 1e-3;
%reg             = 0.01;

e1              = [1;0;0];
e2              = [0;1;0];
e3              = [0;0;1];

gravAcc         = 9.81;

m               = M(1,1);
Mb              = M(1:6,1:6);
Mbj             = M(1:6,7:end);

St              = [ zeros(6,DOF);
                    eye(DOF,DOF)];

grav            = [ zeros(2,1);
                   -m*gravAcc;
                    zeros(3,1)];
  
[pos_rightFoot,rotMatRightFoot] = frame2posrot(feetParam.r_sole);
[pos_leftFoot,rotMatLeftFoot]   = frame2posrot(feetParam.l_sole);

dxCom           = J_CoM(1:3,:)*Nu;
dq              = Nu(7:end);

ddxComStar      = desired_x_dx_ddx_CoM(:,3) - gainParam.gainsPCOM*(xCom-desired_x_dx_ddx_CoM(:,1))...
                 -gainParam.gainsDCOM*(dxCom-desired_x_dx_ddx_CoM(:,2));

Pr              = pos_rightFoot - xCom;   % Application point of the contact force on the right foot w.r.t. CoM
Pl              = pos_leftFoot  - xCom;   % Application point of the contact force on the left  foot w.r.t. CoM

AL              = [ eye(3),zeros(3);
                    skew(Pl),eye(3)];
AR              = [ eye(3),zeros(3);
                    skew(Pr),eye(3)];

%% One foot or two feet on ground selector
if      LEFT_RIGHT_FOOT_IN_CONTACT(1) == 1 &&  LEFT_RIGHT_FOOT_IN_CONTACT(2) == 0
    
    A      = AL;
    pinvA  = eye(6)/A;
    
elseif  LEFT_RIGHT_FOOT_IN_CONTACT(1) == 0 &&  LEFT_RIGHT_FOOT_IN_CONTACT(2) == 1
    
    A      = AR;
    pinvA  = eye(6)/A;
    
elseif  LEFT_RIGHT_FOOT_IN_CONTACT(1) == 1 &&  LEFT_RIGHT_FOOT_IN_CONTACT(2) == 1

    A      = [AL,AR];
    pinvA  = pinv(A,PINV_TOL);
    
end

%% Contact constraints equations
 JcMinv          = Jc/M;
 JcMinvSt        = JcMinv*St;
 JcMinvJct       = JcMinv*transpose(Jc);

 PInv_JcMinvSt   = pinv(JcMinvSt,PINV_TOL);
%PInv_JcMinvSt   = JcMinvSt'/(JcMinvSt*JcMinvSt' + reg*eye(size(JcMinvSt,1)));

%% Newton-Euler equations of motion at CoM
HDotDes         = [ m*ddxComStar;
                   -gainParam.gainMomentum*H(4:end)];
                      
NL              = eye(DOF) - PInv_JcMinvSt*JcMinvSt;

f_HDot          = pinvA*(HDotDes - grav);

%% One foot or two feet on ground selector
if     sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 1 && USE_QP_SOLVER == 0

NA              =  zeros(6);
    
elseif sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 2
    
NA              =  eye(6*param.numConstraints)-pinvA*A;

end
   
%% Terms used for torques equation definition
JBar             =  transpose(Jc(:,7:end)) - Mbj'/Mb*transpose(Jc(:,1:6));    % multiplier of f in tau0
qTilde           =  param.qj-param.qjInit;

Sigma            = -(PInv_JcMinvSt*JcMinvJct + NL*JBar);
SigmaNA          =  Sigma*NA;

tauModel         = PInv_JcMinvSt*(JcMinv*h - dJcNu) + NL*(h(7:end) - Mbj'/Mb*h(1:6) - diag(gainParam.impedances)*qTilde - diag(gainParam.dampings)*dq);

%% Quadratic Programming solver
CL               =  ConstraintsMatrix; 
CL(end-4,1:3)    = -e3'*rotMatLeftFoot;                
CL(end-3,:)      =  [ footSize(1,1)*e3'*rotMatLeftFoot', e2'*rotMatLeftFoot'];
CL(end-2,:)      =  [-footSize(1,2)*e3'*rotMatLeftFoot',-e2'*rotMatLeftFoot'];

CL(end-1,:)      =  [ footSize(2,1)*e3'*rotMatLeftFoot',-e1'*rotMatLeftFoot'];
CL(end  ,:)      =  [-footSize(2,2)*e3'*rotMatLeftFoot', e1'*rotMatLeftFoot'];

CR               =  ConstraintsMatrix;
CR(end-4,1:3)    = -e3'*rotMatRightFoot;  
CR(end-3,:)      =  [ footSize(1,1)*e3'*rotMatRightFoot', e2'*rotMatRightFoot'];
CR(end-2,:)      =  [-footSize(1,2)*e3'*rotMatRightFoot',-e2'*rotMatRightFoot'];

CR(end-1,:)      =  [ footSize(2,1)*e3'*rotMatRightFoot',-e1'*rotMatRightFoot'];
CR(end  ,:)      =  [-footSize(2,2)*e3'*rotMatRightFoot', e1'*rotMatRightFoot'];


ConstraintsMatrix2Feet    = blkdiag(CL,CR);
bVectorConstraints2Feet   = [bVectorConstraints;bVectorConstraints];

ConstraintsMatrixQP1Foot  = CL;
bVectorConstraintsQP1Foot = bVectorConstraints;

if  sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 1 && USE_QP_SOLVER == 1
    
HessianMatrixQP1Foot       =  A'*A;
gradientQP1Foot            = -A'*(HDotDes - grav);

[f_HDot, ~, exitFlag, iter, lambda, auxOutput] = qpOASES(HessianMatrixQP1Foot, gradientQP1Foot, ConstraintsMatrixQP1Foot,...
                                                               [], [], [], bVectorConstraintsQP1Foot);           

  if exitFlag ~= 0
      
  disp('QP failed with');
  disp( exitFlag)
  disp( iter)
  disp( auxOutput)
  disp( lambda)
  error('qp_failed')
      
  end
 
elseif sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 2 && USE_QP_SOLVER == 1

ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*NA;
bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*f_HDot;

HessianMatrixQP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*regHessiansQP;
gradientQP2Feet           = SigmaNA'*(tauModel + Sigma*f_HDot);

[f0, ~, exitFlag, iter, lambda, auxOutput] = qpOASES(HessianMatrixQP2Feet, gradientQP2Feet, ConstraintsMatrixQP2Feet,...
                                                     [], [], [], bVectorConstraintsQp2Feet);           
            
 if exitFlag ~= 0

   disp('QP failed')
   disp(exitFlag);
   disp(iter);
   disp(auxOutput);
   disp(lambda);
   error('qp_failed')
    
 end

end

%% Desired f0 without Quadratic Programming
if      sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 1

  f0      =  zeros(6,1);

elseif  sum(LEFT_RIGHT_FOOT_IN_CONTACT) == 2 && USE_QP_SOLVER == 0 

  f0      = -pinv(SigmaNA,PINV_TOL)*(tauModel+Sigma*f_HDot);

end

%% Joint torques and contact forces
% CoM error
errorCoM = xCom - desired_x_dx_ddx_CoM(:,1);

% Desired contact forces at feet and control torques
fc_des   = f_HDot + NA*f0;
tau      = tauModel + Sigma*fc_des;

% Real contact forces computation
f_c      = (JcMinv*transpose(Jc))\(JcMinv*h -JcMinvSt*tau -dJcNu -feetParam.K_corr_vel.*Jc*Nu -feetParam.K_corr_pos.*feetParam.pos_feet_delta);

end
