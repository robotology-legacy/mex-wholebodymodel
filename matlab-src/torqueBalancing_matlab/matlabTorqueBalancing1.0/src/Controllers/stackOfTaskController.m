function [tau,f0,ddqjNonLin] = stackOfTaskController(params, constraints, gains, trajectory, dynamics)
%STACKOFTASKCONTROLLER is a task-based balancing controller for the humanoid
%                     robot iCub.
%   STACKOFTASKCONTROLLER computes the control torques at joints using a 
%   two task-based approach. 
%
%   [tau,f0,ddqjNonLin] = stackOfTaskController(params, constraints, gains,
%    trajectory, dynamics) takes as input the structure params, which contains
%   all the utility parameters, and the structure dynamics, gains, constraints 
%   and JointReferences which are used to compute the desired torques. The
%   outputs are the desired control torques tau [ndof x 1], f0 [6*nconstr
%   x1] is the vector in the nullspace of the desired contact forces while
%   ddqjNonLin [ndof x 1] are the linear joints accelerations.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% general parameters
pinv_tol            = params.pinv_tol;
regHessianQP        = params.reg_HessianQP;
%pinv_damp          = params.pinv_damp;
feet_on_ground      = params.feet_on_ground;
ndof                = params.ndof;
use_QPsolver        = params.use_QPsolver;
e1                  = [1;0;0];
e2                  = [0;1;0];
e3                  = [0;0;1];
gravAcc             = 9.81;
S                   = [zeros(6,ndof);
                      eye(ndof,ndof)];
% constraints
ConstraintsMatrix   = constraints.ConstraintsMatrix;
bVectorConstraints  = constraints.bVectorConstraints ;
footSize            = constraints.footSize;
% gains
impedances          = gains.impedances;
dampings            = gains.dampings;
posturalCorr        = gains.posturalCorr;
VelGainsMom         = gains.VelGainsMom;
PosGainsMom         = gains.PosGainsMom;
% dynamics and forward kinematics
M                   = dynamics.M;
g                   = dynamics.g;
CNu                 = dynamics.CNu;
dJcNu               = dynamics.dJcNu;
H                   = dynamics.H;
qj                  = dynamics.qj;
xCoM                = dynamics.xCoM;
dxCoM               = dynamics.dxCoM;
dqj                 = dynamics.Nu(7:end);
Jc                  = dynamics.Jc;
h                   = g + CNu;
m                   = M(1,1);
Mb                  = M(1:6,1:6);
Mbj                 = M(1:6,7:end);
Mjb                 = M(7:end,1:6);
Mj                  = M(7:end,7:end);
JcBase              = Jc(:,1:6);
JcJoint             = Jc(:,7:end);
x_dx_ddx_CoMDes     = trajectory.desired_x_dx_ddx_CoM;
f_grav              = [ zeros(2,1);
                       -m*gravAcc;
                        zeros(3,1)];
% Feet position and rotation matrix
[posRFoot,RotRFoot] = frame2posrot(dynamics.RFootPose);
[posLFoot,RotLFoot] = frame2posrot(dynamics.LFootPose);
distRF              = posRFoot - xCoM;       % Application point of the contact force on the right foot w.r.t. CoM
distLF              = posLFoot - xCoM;       % Application point of the contact force on the left  foot w.r.t. CoM
AL                  = [eye(3),     zeros(3);
                       skew(distLF),eye(3)];
AR                  = [eye(3),     zeros(3);
                       skew(distRF),eye(3)];
                   
% One foot or two feet on ground selector
if      sum(feet_on_ground) == 2
    
    A      = [AL,AR];
    pinvA  = pinv(A,pinv_tol);    
else  
    
    if      feet_on_ground(1) == 1 
    
    A      = AL;
    
    elseif  feet_on_ground(2) == 1

    A      = AR;
    end
    
    pinvA  = eye(6)/A;
end

%% Terms used in the control torques equation
ddqjRef         = trajectory.JointReferences.ddqjRef;
dqjRef          = trajectory.JointReferences.dqjRef;
qjRef           = trajectory.JointReferences.qjRef;
qjTilde         = qj-qjRef;
dqjTilde        = dqj-dqjRef;
JcMinv          = Jc/M;
Lambda          = JcMinv*S;
JBar            = transpose(Jc(:,7:end)) - Mbj'/Mb*transpose(Jc(:,1:6));   % multiplier of f in tau0
JcMinvJct       = JcMinv*transpose(Jc);
pinvLambda      = pinv(Lambda,pinv_tol);
%pinvLambda     = Lambda'/(Lambda*Lambda' + pinv_damp*eye(size(Lambda,1)));

%% Newton-Euler equations of motion at CoM
% closing the loop on angular momentum integral; desired centroidal
% momentum dynamics
deltaPhi           = params.JhReduced(4:end,:)*(qj-params.qjInit);
accDesired         = [m.*x_dx_ddx_CoMDes(:,3); zeros(3,1)];
velDesired         = -VelGainsMom*[m.*(dxCoM-x_dx_ddx_CoMDes(:,2)); H(4:end)];
posDesired         = -PosGainsMom*[m.*(xCoM-x_dx_ddx_CoMDes(:,1)); deltaPhi];
HDotDes            = accDesired + velDesired + posDesired;
% contact forces that generate the desired dynamics; contact forces and
% torques null spaces
fc_HDot            = pinvA*(HDotDes - f_grav);
Nullfc             = eye(6*params.numConstraints)-pinvA*A;
NullLambda         = eye(ndof)-pinvLambda*Lambda;

%% Separate the terms which contains the contact forces into the control torque equation
Sigma              = -(pinvLambda*JcMinvJct + NullLambda*JBar);
SigmaNA            =  Sigma*Nullfc;
tauModel           =   pinvLambda*(JcMinv*h - dJcNu) +NullLambda*(h(7:end) -Mbj'/Mb*h(1:6)...
                     + Mj*ddqjRef -impedances*posturalCorr*qjTilde -dampings*posturalCorr*dqjTilde);
        
%% Quadratic Programming solver
f0                 =  zeros(6,1);

if use_QPsolver == 1
    
CL                 =  ConstraintsMatrix; 
CL(end-4,1:3)      = -e3'*RotLFoot;                
CL(end-3,:)        =  [ footSize(1,1)*e3'*RotLFoot', e2'*RotLFoot'];
CL(end-2,:)        =  [-footSize(1,2)*e3'*RotLFoot',-e2'*RotLFoot'];
CL(end-1,:)        =  [ footSize(2,1)*e3'*RotLFoot',-e1'*RotLFoot'];
CL(end  ,:)        =  [-footSize(2,2)*e3'*RotLFoot', e1'*RotLFoot'];
CR                 =  ConstraintsMatrix;
CR(end-4,1:3)      = -e3'*RotRFoot;  
CR(end-3,:)        =  [ footSize(1,1)*e3'*RotRFoot', e2'*RotRFoot'];
CR(end-2,:)        =  [-footSize(1,2)*e3'*RotRFoot',-e2'*RotRFoot'];
CR(end-1,:)        =  [ footSize(2,1)*e3'*RotRFoot',-e1'*RotRFoot'];
CR(end  ,:)        =  [-footSize(2,2)*e3'*RotRFoot', e1'*RotRFoot'];

ConstraintsMatrix2Feet    = blkdiag(CL,CR);
bVectorConstraints2Feet   = [bVectorConstraints;bVectorConstraints];
ConstraintsMatrixQP1Foot  = CL;
bVectorConstraintsQP1Foot = bVectorConstraints;

if  sum(feet_on_ground) == 1
    
HessianMatrixQP1Foot      =  A'*A;
gradientQP1Foot           = -A'*(HDotDes-f_grav);
  
[fc_HDot, ~, exitFlag, iter, lambda, auxOutput] = qpOASES(HessianMatrixQP1Foot, gradientQP1Foot, ConstraintsMatrixQP1Foot,...
                                                          [], [], [], bVectorConstraintsQP1Foot);           

elseif sum(feet_on_ground) == 2 

ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*Nullfc;
bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*fc_HDot;
HessianMatrixQP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*regHessianQP;
gradientQP2Feet           = SigmaNA'*(tauModel + Sigma*fc_HDot);

[f0, ~, exitFlag, iter, lambda, auxOutput] = qpOASES(HessianMatrixQP2Feet, gradientQP2Feet, ConstraintsMatrixQP2Feet,...
                                                     [], [], [], bVectorConstraintsQp2Feet);           
end

if exitFlag ~= 0

   disp('QP failed with:')
   disp(exitFlag);
   disp(iter);
   disp(auxOutput);
   disp(lambda);
   error('QP failed')   
end

else

% Desired f0 without Quadratic Programming
if  sum(feet_on_ground) == 2

  f0    = -pinv(SigmaNA,pinv_tol)*(tauModel+Sigma*fc_HDot);
end
end

%% Joint torques and contact forces
fcDes         = fc_HDot + Nullfc*f0;
tau           = tauModel + Sigma*fcDes;

%% Desired nonLinear joints accelerations for the linearized system analysis
Mbar           = Mj - Mjb/Mb*Mbj;
invMbar        = eye(ndof)/Mbar;
%invMbar       = Mbar'/(Mbar*Mbar' + pinv_damp*eye(size(Mbar,1)));
LambdaLin      = (JcJoint - JcBase/Mb*Mbj)*invMbar;
MultFirstTask  = JcBase/Mb*transpose(JcBase)*pinvA;
pinvLambdaLin  = pinv(LambdaLin,pinv_tol);
CbNu           = CNu(1:6);
NullLambda     = eye(ndof) - pinvLambdaLin*LambdaLin;
Postural       = -Mbar*ddqjRef +impedances*posturalCorr*qjTilde +dampings*posturalCorr*dqjTilde;

ddqjNonLin     = -invMbar*(pinvLambdaLin*(MultFirstTask*(HDotDes-CbNu)+dJcNu) + NullLambda*Postural);

end
