function fcDes = QPSolver(controlParam,CONFIG,FORKINEMATICS)
%QPSOLVER  implements a quadratic programming solver. The objective is the
%          minimization of the control torques by means of the nullspace of
%          contact forces, f0. Inequality constraints to ensure fc is inside
%          the friction cones are also taken into account in the optimization.
%
% fcDes = QPSOLVER(controlParam,CONFIG,FORKINEMATICS) takes as
% input the structures: controlParam, containing the controller
% parameters; CONFIG, which contains all the configuration parameters and 
% FORKINEMATICS, which contains the forward kinematics of the robot. The 
% output are the desired contact forces fcDes [6*numconstraints x 1].
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
%% Config parameters
import WBM.utilities.frame2posRotm;

regHessianQP         = CONFIG.reg_HessianQP;
feet_on_ground       = CONFIG.feet_on_ground;

%% Control parameters
fcHDot               = controlParam.fcHDot;
tauModel             = controlParam.tauModel;
Sigma                = controlParam.Sigma;
NullA                = controlParam.NullA;
A                    = controlParam.A;
HDotDes              = controlParam.HDotDes;
f_grav               = controlParam.f_grav;
SigmaNA              = Sigma*NullA;

%% Forward Kinematics
poseRFoot_qt         = FORKINEMATICS.poseRFoot_qt;
poseLFoot_qt         = FORKINEMATICS.poseLFoot_qt;

%% Constraints for QP
e1                   = [1;0;0];
e2                   = [0;1;0];
e3                   = [0;0;1];
[~,w_R_RFoot]        = frame2posRotm(poseRFoot_qt);
[~,w_R_LFoot]        = frame2posRotm(poseLFoot_qt);

% the friction cone is approximated by using linear interpolation of the circle.
% numberOfPoints defines the number of points used to interpolate the circle in
% each cicle's quadrant
numberOfPoints               = 4;
forceFrictionCoefficient     = 1;
torsionalFrictionCoefficient = 2/150;
footSize                     = CONFIG.footSize;
fZmin                        = 10;

% the QP solver will search a solution f0 that satisfies the inequality Aineq_f F(f0) < bineq_f
[ConstraintsMatrix,bVectorConstraints] = frictionCones(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,footSize,fZmin);

%% QP SOLVER
CL                 =  ConstraintsMatrix;
CL(end-4,1:3)      = -e3'*w_R_LFoot;
CL(end-3,:)        =  [ footSize(1,1)*e3'*w_R_LFoot', e2'*w_R_LFoot'];
CL(end-2,:)        =  [-footSize(1,2)*e3'*w_R_LFoot',-e2'*w_R_LFoot'];
CL(end-1,:)        =  [ footSize(2,1)*e3'*w_R_LFoot',-e1'*w_R_LFoot'];
CL(end  ,:)        =  [-footSize(2,2)*e3'*w_R_LFoot', e1'*w_R_LFoot'];
CR                 =  ConstraintsMatrix;
CR(end-4,1:3)      = -e3'*w_R_RFoot;
CR(end-3,:)        =  [ footSize(1,1)*e3'*w_R_RFoot', e2'*w_R_RFoot'];
CR(end-2,:)        =  [-footSize(1,2)*e3'*w_R_RFoot',-e2'*w_R_RFoot'];
CR(end-1,:)        =  [ footSize(2,1)*e3'*w_R_RFoot',-e1'*w_R_RFoot'];
CR(end  ,:)        =  [-footSize(2,2)*e3'*w_R_RFoot', e1'*w_R_RFoot'];

ConstraintsMatrix2Feet    = blkdiag(CL,CR);
bVectorConstraints2Feet   = [bVectorConstraints;bVectorConstraints];
ConstraintsMatrixQP1Foot  = CL;
bVectorConstraintsQP1Foot = bVectorConstraints;

if     sum(feet_on_ground) == 1
    
    HessianMatrixQP1Foot      =  A'*A;
    gradientQP1Foot           = -A'*(HDotDes-f_grav);
    
    [fcDes, ~, exitFlag, iter, lambda, auxOutput] = qpOASES(HessianMatrixQP1Foot, gradientQP1Foot, ConstraintsMatrixQP1Foot,...
                                                            [], [], [], bVectorConstraintsQP1Foot);
    
elseif sum(feet_on_ground) == 2
    
    ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*NullA;
    bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*fcHDot;
    HessianMatrixQP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*regHessianQP;
    gradientQP2Feet           = SigmaNA'*(tauModel + Sigma*fcHDot);
    
    [f0, ~, exitFlag, iter, lambda, auxOutput]    = qpOASES(HessianMatrixQP2Feet, gradientQP2Feet, ConstraintsMatrixQP2Feet,...
                                                            [], [], [], bVectorConstraintsQp2Feet);
    
    fcDes                     = fcHDot   + NullA*f0;
end

if exitFlag ~= 0
    
    disp('QP failed with:')
    disp(exitFlag);
    disp(iter);
    disp(auxOutput);
    disp(lambda);
    error('QP failed')
end

end
