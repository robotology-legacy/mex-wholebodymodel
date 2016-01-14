function [tau,errorCoM,f0]=stackOfTaskController(param, constraints, feet, gains, Nu, M, h, H, Jc, dJcNu, xCoM, J_CoM, desired_x_dx_ddx_CoM)
%% stackOfTaskController
%  It computes the desired contact forces and moments and the joint torques for iCub balancing control using the "Stack of Task" control approach. 
%  If QP_SOLVER is selected, it uses a quadratic programming to calculate the null space of desired contact forces which minimize 
%  the norm of joint torques.     
%  The output are:
%
%  tau [nDof x 1]   which is the vector of control torques at joints;
%
%  errorCoM [3 x 1] which is the CoM position error with respect of the
%                   desired CoM trajectory;
%
%  f0 [numConstraints x 1] which is the vector in the null space of contact
%                          forces and moments
%% Variables and tolerances definition
 Pinv_tol        = 1e-8;
 regHessiansQP   = 5e-2;
%reg             = 0.01;

feet_on_ground     = param.feet_on_ground;
ndof               = param.ndof;
use_QPsolver       = param.use_QPsolver;

ConstraintsMatrix  = constraints.ConstraintsMatrix;
bVectorConstraints = constraints.bVectorConstraints ;
footSize           = constraints.footSize;

e1              = [1;0;0];
e2              = [0;1;0];
e3              = [0;0;1];

gravAcc         = 9.81;

m               = M(1,1);
Mb              = M(1:6,1:6);
Mbj             = M(1:6,7:end);

S               = [ zeros(6,ndof);
                    eye(ndof,ndof)];

f_grav          = [ zeros(2,1);
                   -m*gravAcc;
                    zeros(3,1)];
  
% Feet position and rotation matrix
[pos_rightFoot,rotMatRightFoot] = frame2posrot(feet.r_sole);
[pos_leftFoot,rotMatLeftFoot]   = frame2posrot(feet.l_sole);

dxCoM            = J_CoM(1:3,:)*Nu;
dqj              = Nu(7:end);

ddxCoMStar      = desired_x_dx_ddx_CoM(:,3) - gains.gainsPCoM*(xCoM-desired_x_dx_ddx_CoM(:,1))...
                  -gains.gainsDCoM*(dxCoM-desired_x_dx_ddx_CoM(:,2));

Pr              = pos_rightFoot - xCoM;   % Application point of the contact force on the right foot w.r.t. CoM
Pl              = pos_leftFoot  - xCoM;   % Application point of the contact force on the left  foot w.r.t. CoM

AL              = [ eye(3),zeros(3);
                    skew(Pl),eye(3)];
AR              = [ eye(3),zeros(3);
                    skew(Pr),eye(3)];

%% One foot or two feet on ground selector
if      sum(feet_on_ground) == 2
    
    A      = [AL,AR];
    pinvA  = pinv(A,Pinv_tol);
    
else  
    
    if      feet_on_ground(1) == 1 
    
    A      = AL;
    
    elseif  feet_on_ground(2) == 1

    A      = AR;
    
    end
    
    pinvA  = eye(6)/A;

end

%% Contact constraints equations
 JcMinv         = Jc/M;
 JcMinvS        = JcMinv*S;
 JcMinvJct      = JcMinv*transpose(Jc);

 Pinv_JcMinvS   = pinv(JcMinvS,Pinv_tol);
%Pinv_JcMinvS   = JcMinvS'/(JcMinvS*JcMinvS' + reg*eye(size(JcMinvS,1)));

%% Newton-Euler equations of motion at CoM
HDotDes         = [ m*ddxCoMStar;
                   -gains.gainMomentum*H(4:end)];
                     
f_HDot          = pinvA*(HDotDes - f_grav);
    
% Forces and torques null spaces
Nullfc          =  eye(6*param.numConstraints)-pinvA*A;
NullLambda      =  eye(ndof) - Pinv_JcMinvS*JcMinvS;

%% Terms used for torques equation definition
JBar             =  transpose(Jc(:,7:end)) - Mbj'/Mb*transpose(Jc(:,1:6));    % multiplier of f in tau0
qTilde           =  param.qj-param.qjInit;

Sigma            = -(Pinv_JcMinvS*JcMinvJct + NullLambda*JBar);
SigmaNA          =  Sigma*Nullfc;

tauModel         = Pinv_JcMinvS*(JcMinv*h - dJcNu) +NullLambda*(h(7:end) -Mbj'/Mb*h(1:6) -diag(gains.impedances)*qTilde -diag(gains.dampings)*dqj);

%% Quadratic Programming solver
f0               =  zeros(6,1);

if use_QPsolver == 1
    
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

if  sum(feet_on_ground) == 1
    
HessianMatrixQP1Foot      =  A'*A;
gradientQP1Foot           = -A'*(HDotDes - f_grav);
  
[f_HDot, ~, exitFlag, iter, lambda, auxOutput] = qpOASES(HessianMatrixQP1Foot, gradientQP1Foot, ConstraintsMatrixQP1Foot,...
                                                         [], [], [], bVectorConstraintsQP1Foot);           

elseif sum(feet_on_ground) == 2 

ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*Nullfc;
bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*f_HDot;

HessianMatrixQP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*regHessiansQP;
gradientQP2Feet           = SigmaNA'*(tauModel + Sigma*f_HDot);

[f0, ~, exitFlag, iter, lambda, auxOutput] = qpOASES(HessianMatrixQP2Feet, gradientQP2Feet, ConstraintsMatrixQP2Feet,...
                                                     [], [], [], bVectorConstraintsQp2Feet);           

end

if exitFlag ~= 0

   disp('QP failed with:')
   disp(exitFlag);
   disp(iter);
   disp(auxOutput);
   disp(lambda);
   error('qp_failed')
    
end

else

% Desired f0 without Quadratic Programming

if  sum(feet_on_ground) == 2

  f0    = -pinv(SigmaNA,Pinv_tol)*(tauModel+Sigma*f_HDot);

end

end

%% Joint torques and contact forces
% CoM error
errorCoM = xCoM - desired_x_dx_ddx_CoM(:,1);

% Desired contact forces at feet and control torques
fc_des   = f_HDot + Nullfc*f0;
tau      = tauModel + Sigma*fc_des;

end
