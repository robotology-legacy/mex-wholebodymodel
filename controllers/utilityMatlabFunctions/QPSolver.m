function fcDes = QPSolver(controlParam,CONFIG,FORKINEMATICS)
%QPSOLVER  implements a quadratic programming solver. The objective is the
%          minimization of the control torques by means of the nullspace of
%          contact forces, f0. Inequality constraints to ensure fc is inside 
%          the friction cones are also taken into account in the optimization.
%
%          fcDes = QPSOLVER(controlParam,config,forKinematics) takes as 
%          input the structures: CONTROLPARAM, containing the controller 
%          parameters; CONFIG, which contains all the configuration
%          parameters and FORKINEMATICS, which contains the forward kinematics 
%          of the robot. The output are the desired contact forces
%          FCDES [6*numconstraints x 1].
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
%% Config parameters
regHessianQP          = CONFIG.reg_HessianQP;
feet_on_ground        = CONFIG.feet_on_ground;

%% Control parameters
fcHDot                = controlParam.fcHDot;
tauModel              = controlParam.tauModel;
Sigma                 = controlParam.Sigma;   
Nullfc                = controlParam.Nullfc;  
A                     = controlParam.A;
SigmaNA               = Sigma*Nullfc;

%% Forward Kinematics
RFootPoseQuat        = FORKINEMATICS.RFootPoseQuat;
LFootPoseQuat        = FORKINEMATICS.LFootPoseQuat;

%% Constraints for QP 
e1                   = [1;0;0];
e2                   = [0;1;0];
e3                   = [0;0;1];
[~,RotRFoot]         = frame2posrot(RFootPoseQuat);
[~,RotLFoot]         = frame2posrot(LFootPoseQuat);

% the friction cone is approximated by using linear interpolation of the circle.
% numberOfPoints defines the number of points used to interpolate the circle in 
% each cicle's quadrant 
numberOfPoints               = 4;   
forceFrictionCoefficient     = 1;              
torsionalFrictionCoefficient = 2/150;
footSize                     = [-0.1 0.1;       % xMin, xMax
                                -0.1 0.1];      % yMin, yMax    
fZmin                        = 10;

% the QP solver will search a solution f0 that satisfies the inequality Aineq_f F(f0) < bineq_f 
[ConstraintsMatrix,bVectorConstraints] = frictionCones(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,footSize,fZmin);

%% QP SOLVER    
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

if     sum(feet_on_ground) == 1
    
HessianMatrixQP1Foot      =  A'*A;
gradientQP1Foot           = -A'*(HDotDes-f_grav);

[fcDes, ~, exitFlag, iter, lambda, auxOutput] = qpOASES(HessianMatrixQP1Foot, gradientQP1Foot, ConstraintsMatrixQP1Foot,...
                                                        [], [], [], bVectorConstraintsQP1Foot);           

elseif sum(feet_on_ground) == 2 

ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*Nullfc;
bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*fcHDot;
HessianMatrixQP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*regHessianQP;
gradientQP2Feet           = SigmaNA'*(tauModel + Sigma*fcHDot);

[f0, ~, exitFlag, iter, lambda, auxOutput]    = qpOASES(HessianMatrixQP2Feet, gradientQP2Feet, ConstraintsMatrixQP2Feet,...
                                                        [], [], [], bVectorConstraintsQp2Feet); 
                                                    
fcDes                     = fcHDot   + Nullfc*f0;                                                   
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
