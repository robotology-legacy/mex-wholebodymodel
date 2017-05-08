function fcDes = QPSolver(CONTROLLER,MODEL,FORKINEMATICS)
%QPSOLVER  implements a quadratic programming solver. The objective is the
%          minimization of the control torques by means of the nullspace of
%          contact forces, f0. Inequality constraints to ensure fc is inside
%          the friction cones are also taken into account in the optimization.
%
% Format: fcDes = QPSOLVER(CONTROLLER,MODEL,FORKINEMATICS)
%
% Inputs:  - CONTROLLER is a structure containing the control torques, 
%            and other parameters for controlling the robot.
%          - MODEL is a structure defining the robot model;
%          - FORKINEMATICS stores position, orientation and velocity of
%            points in cartesian space;
%
% Output:  - fcDes desired contact forces;
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
regHessianQP         = MODEL.CONFIG.reg_HessianQP;
feet_on_ground       = MODEL.CONFIG.feet_on_ground;

%% Control parameters
fcHStar              = CONTROLLER.fcHStar;
tauModel             = CONTROLLER.tauModel;
Sigma                = CONTROLLER.Sigma;
NullA                = CONTROLLER.NullA;
A                    = CONTROLLER.A;
HDotStar             = CONTROLLER.HDotStar;
f_grav               = CONTROLLER.f_grav;
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
feetSize                     = MODEL.feetSize;
fZmin                        = 10;
% the QP solver will search a solution f0 that satisfies the inequality Aineq_f F(f0) < bineq_f
[ConstraintsMatrix,bVectorConstraints] = frictionCones(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,feetSize,fZmin);

%% QP SOLVER
CL                 =  ConstraintsMatrix;
CL(end-4,1:3)      = -e3'*w_R_LFoot;
CL(end-3,:)        =  [ feetSize(1,1)*e3'*w_R_LFoot', e2'*w_R_LFoot'];
CL(end-2,:)        =  [-feetSize(1,2)*e3'*w_R_LFoot',-e2'*w_R_LFoot'];
CL(end-1,:)        =  [ feetSize(2,1)*e3'*w_R_LFoot',-e1'*w_R_LFoot'];
CL(end  ,:)        =  [-feetSize(2,2)*e3'*w_R_LFoot', e1'*w_R_LFoot'];
CR                 =  ConstraintsMatrix;
CR(end-4,1:3)      = -e3'*w_R_RFoot;
CR(end-3,:)        =  [ feetSize(1,1)*e3'*w_R_RFoot', e2'*w_R_RFoot'];
CR(end-2,:)        =  [-feetSize(1,2)*e3'*w_R_RFoot',-e2'*w_R_RFoot'];
CR(end-1,:)        =  [ feetSize(2,1)*e3'*w_R_RFoot',-e1'*w_R_RFoot'];
CR(end  ,:)        =  [-feetSize(2,2)*e3'*w_R_RFoot', e1'*w_R_RFoot'];

ConstraintsMatrix2Feet    = blkdiag(CL,CR);
bVectorConstraints2Feet   = [bVectorConstraints;bVectorConstraints];
ConstraintsMatrixQP1Foot  = CL;
bVectorConstraintsQP1Foot = bVectorConstraints;

if     sum(feet_on_ground) == 1
    % QP solver in case of 1 foot balancing. It just applies the
    % constraints to desired contact forces
    HessianMatrixQP1Foot          =  A'*A;
    gradientQP1Foot               = -A'*(HDotStar-f_grav);
    [fcDes, ~, exitFlag, ~, ~, ~] = qpOASES(HessianMatrixQP1Foot, gradientQP1Foot, ConstraintsMatrixQP1Foot,...
                                            [], [], [], bVectorConstraintsQP1Foot);  
elseif sum(feet_on_ground) == 2
    % QP solver for 2 feet balancing
    ConstraintsMatrixQP2Feet   = ConstraintsMatrix2Feet*NullA;
    bVectorConstraintsQp2Feet  = bVectorConstraints2Feet-ConstraintsMatrix2Feet*fcHStar;
    HessianMatrixQP2Feet       = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*regHessianQP;
    gradientQP2Feet            = SigmaNA'*(tauModel + Sigma*fcHStar);
    
    [f0, ~, exitFlag, ~, ~, ~] = qpOASES(HessianMatrixQP2Feet, gradientQP2Feet, ConstraintsMatrixQP2Feet,...
                                         [], [], [], bVectorConstraintsQp2Feet);   
    fcDes                      = fcHStar + NullA*f0;
end
% detect eventual Qp failures
if exitFlag ~= 0   
    warning(['QP failed with: ', num2str(exitFlag)])
end

end
