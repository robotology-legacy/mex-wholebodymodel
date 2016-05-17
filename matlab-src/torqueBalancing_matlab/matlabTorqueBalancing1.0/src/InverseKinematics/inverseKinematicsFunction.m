function [dChi, visualizeIkinParam] = inverseKinematicsFunction(t,Chi,params)
%INVERSEKINEMATICSFUNCTION is the function that generates the derivative of
%   the robot state from inverse kinematics.
%   INVERSEKINEMATICSFUNCTION calculates the state derivative using a three
%   task stack-of-task controller. The first task is the contact constraint
%   dynamics, the second is the reachement of a desired momentum trajectory,
%   the third is to keep the initial posture.
%
%   [dChiIkin,visualizeIkinParam]=INVERSEKINEMATICSFUNCTION(t,ChiIkin,params)
%   takes as input the integration time t, the robot state ChiIkin and the
%   structure params which contains all the utility parameters. The output
%   are the state derivative dChiIkin  [13+2ndof x 1] and the structure 
%   which contains the parameters for visualization, visualizeIkinParam.
%   
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% setup initial parameters
waitbar(t/params.tEnd,params.wait)
pinv_tol          = params.pinv_tol;
ndof              = params.ndof;  
m                 = params.MInit(1,1);
% gains for inverse kinematics
KPosFeet          = 15;
KPosMom           = 15;
KPosPost          = 15;
KVelFeet          = 2*sqrt(KPosFeet);
KVelMom           = 2*sqrt(KPosMom);
KVelPost          = 2*sqrt(KPosPost);

%% CoM trajectory generator
desired_x_dx_ddx_CoM = generTraj(params.CoMInit(1:3),t,params.trajectory);

%% Demux the robot state
BasePose             = Chi(1:7);
qj                   = Chi(8:7+ndof);
[PosBase,RotBase]    = frame2posrot(BasePose);
Nu                   = Chi(8+ndof:end);
dqj                  = Nu(7:end);
NuBaseIkin           = Nu(1:6);                               
dQuatBase            = quaternionDerivative(transpose(RotBase)*NuBaseIkin(4:end), BasePose(4:end));

%% Feet, CoM and angular momentum Jacobians
for ii=1:params.numConstraints
    
Jc(6*(ii-1)+1:6*ii,:)    = wbm_jacobian(RotBase,PosBase,qj,params.constraintLinkNames{ii});
end

JMomBase                 = zeros(6,6);
JMomJoint                = zeros(6,ndof);

for ii = 1:6
    
NuBaseJMom               = zeros(6,1);
NuBaseJMom(ii)           = 1;
JMomBase(:,ii)           = wbm_centroidalMomentum(RotBase,PosBase,qj,zeros(ndof,1),NuBaseJMom);
end

for ii = 1:ndof

dqjJMom                  = zeros(ndof,1);
dqjJMom(ii)              = 1;
JMomJoint(:,ii)          = wbm_centroidalMomentum(RotBase,PosBase,qj,dqjJMom,zeros(6,1));
end

JCoMPose                 = wbm_jacobian(RotBase,PosBase,qj,'com');
JCoM                     = JCoMPose(1:3,:);
JAng                     = [JMomBase(4:6,:) JMomJoint(4:6,:)];
JMom                     = [m*JCoM; JAng];

%% Jacobians time derivatives
for ii=1:params.numConstraints
    
dJcNu(6*(ii-1)+1:6*ii,:) = wbm_djdq(RotBase,PosBase,qj,dqj,NuBaseIkin,params.constraintLinkNames{ii});
end

dJCoMNuPose   =  wbm_djdq(RotBase,PosBase,qj,dqj,NuBaseIkin,'com');
dJCoMNu       =  dJCoMNuPose(1:3);
dJMomNu       = [m*dJCoMNu; zeros(3,1)];

%% Position errors at feet, angular momentum and CoM
% angular momentum integral
deltaIntAng                  = params.JhReduced(4:6,:)*(qj-params.qjInit); 
% CoM position
CoMPose                      = wbm_forwardKinematics(RotBase,PosBase,qj,'com');
xCoMIkin                     = CoMPose(1:3);
deltaPosCoM                  = xCoMIkin - desired_x_dx_ddx_CoM(:,1);
% feet pose
lsole                        = wbm_forwardKinematics(RotBase,PosBase,qj,'l_sole');
rsole                        = wbm_forwardKinematics(RotBase,PosBase,qj,'r_sole');
lsoleIni                     = params.PoseLFootQuatInit ;
rsoleIni                     = params.PoseRFootQuatInit ;

[xLfoot,RotLfoot]            = frame2posrot(lsole);
[xRfoot,RotRfoot]            = frame2posrot(rsole);
[xLfootIni,RotLfootIni]      = frame2posrot(lsoleIni);
[xRfootIni,RotRfootIni]      = frame2posrot(rsoleIni);

[TLfoot,LfootOri]            = parametrization(RotLfoot);
[TRfoot,RfootOri]            = parametrization(RotRfoot);
[~,LfootOriIni]              = parametrization(RotLfootIni);
[~,RfootOriIni]              = parametrization(RotRfootIni);

poseLeftFoot                 = [xLfoot; LfootOri'];
poseRightFoot                = [xRfoot; RfootOri'];
poseLeftFootIni              = [xLfootIni; LfootOriIni'];
poseRightFootIni             = [xRfootIni; RfootOriIni'];
  
if       sum(params.feet_on_ground) == 1 && params.feet_on_ground(1) == 1
     
TLeftFoot        = [eye(3)   zeros(3);
                    zeros(3)  TLfoot];
feetPoseError    = TLeftFoot*(poseLeftFoot-poseLeftFootIni);
 
elseif   sum(params.feet_on_ground) == 1 && params.feet_on_ground(2) == 1
     
TRightFoot       = [eye(3)   zeros(3);
                    zeros(3)  TRfoot];
feetPoseError    = TRightFoot*(poseRightFoot-poseRightFootIni);
 
elseif   sum(params.feet_on_ground) == 2
     
TLeftFoot        = [eye(3)    zeros(3);
                     zeros(3)  TLfoot];
TRightFoot       = [eye(3)    zeros(3);
                     zeros(3)  TRfoot];
Tfeet            = [TLeftFoot     zeros(6);
                    zeros(6)   TRightFoot];
feetPoseError    = Tfeet*[poseLeftFoot-poseLeftFootIni; poseRightFoot-poseRightFootIni];
end 

%% Velocity errors
deltaVelFeet       = Jc*Nu;
deltaVelCoM        = JCoM*Nu - desired_x_dx_ddx_CoM(:,2);
deltaAngMom        = JAng*Nu;

%% STACK OF TASK INVERSE KINEMATICS
% first task: respect the constraints at feet 
feetErrorDynamics  = -dJcNu -KPosFeet*feetPoseError -KVelFeet*deltaVelFeet;
NullFeet           =  eye(ndof+6) -pinv(Jc,pinv_tol)*Jc;

% second task: generate a desired Momentum trajectory
MomErrorDynamics   = -dJMomNu +[m*desired_x_dx_ddx_CoM(:,3);zeros(3,1)] -KPosMom*[m*deltaPosCoM; deltaIntAng] ...
                     -KVelMom*[m*deltaVelCoM; deltaAngMom] -JMom*pinv(Jc,pinv_tol)*feetErrorDynamics;
JMomTaks           =  JMom*NullFeet;
NullMom            =  eye(ndof+6) - pinv(JMomTaks,pinv_tol)*JMomTaks;

% third task: keep the initial posture
JPost              = [zeros(ndof,6) eye(ndof)];

postErrorDynamics  = -KVelPost*Nu(7:end) -KPosPost*(qj-params.qjInit)...
                     -JPost*pinv(Jc,pinv_tol)*feetErrorDynamics -JPost*NullFeet*pinv(JMomTaks,pinv_tol)*MomErrorDynamics;
JPostTask          =  JPost*NullFeet*NullMom;

% accelerations from inverse kinematics
dNuPost            = pinv(JPostTask,pinv_tol)*postErrorDynamics;
dNuMom             = pinv(JMomTaks,pinv_tol)*MomErrorDynamics + NullMom*dNuPost;
dNuIkin            = pinv(Jc,pinv_tol)*feetErrorDynamics + NullFeet*dNuMom;

%% State derivative
dChi               = [NuBaseIkin(1:3); dQuatBase; dqj; dNuIkin];

%% Parameters for visualization
ObtainedCoMTraj                       = [xCoMIkin; JCoM*Nu; (JCoM*dNuIkin+dJCoMNu)];
DesiredCoMTraj                        = [desired_x_dx_ddx_CoM(:,1); desired_x_dx_ddx_CoM(:,2); desired_x_dx_ddx_CoM(:,3)];

visualizeIkinParam.CoMTrajectoryError = [DesiredCoMTraj; ObtainedCoMTraj];
visualizeIkinParam.feetError          = feetPoseError;
visualizeIkinParam.momentumError      = [m*deltaVelCoM; JAng*Nu];

end
