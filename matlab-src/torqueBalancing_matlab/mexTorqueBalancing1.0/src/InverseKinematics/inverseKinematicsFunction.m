%% inverseKinematicsFunction
% calculates the Joints and base accelerations using a stack of task
% inverse kinematics approach. 
%
% The outputs are:
%
% dChiIkin [13+2*ndof x 1]   which is the state vector that will be integrated
%
% visulizeIkinParam          which contains the parameters for visualization
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function [dChiIkin, visualizeIkinParam] = inverseKinematicsFunction(t, ChiIkin, params)

waitbar(t/params.tEnd,params.wait)
 
%% Desired oscillation at CoM
desired_x_dx_ddx_CoM   = generTraj(params.CoMInit(1:3), t,params.trajectory); 

%% Setup parameters
ndof                   = params.ndof; 
toll                   = params.pinv_tol;
% gains
Kcorr_feet             = 10;
Kcorr_CoM              = 10;
Kcorr_Ang              = 10;
Kcorr_qj               = 10;  
Kcorr_feetVel          = 2*sqrt(Kcorr_feet);
Kcorr_CoMVel           = 2*sqrt(Kcorr_CoM);
Kcorr_AngVel           = 2*sqrt(Kcorr_Ang);
Kcorr_dqj              = 2*sqrt(Kcorr_qj);

%% Demux the state vector 
BasePose             = ChiIkin(1:7);
qj                   = ChiIkin(8:7+ndof);
Nu                   = ChiIkin(8+ndof:end);
% base position and orientation
[PosBase,RotBase]    = frame2posrot(BasePose);

%% Joints limits check
limits = params.limits;
l_min  = limits(:,1);
l_max  = limits(:,2);
tol    = 0.01;

res = qj < l_min + tol | qj > l_max - tol;
res = sum(res);

if res==0

else
 
% disp('Joint limits reached at time:')    
% disp(t)
% error('Joint limits reached in the inverse kinematics solver '); 

end

%% Jacobian at feet, CoM and centroidal momentum
for ii=1:params.numConstraints
    
    Jc(6*(ii-1)+1:6*ii,:)    = wbm_jacobian(RotBase,PosBase,qj,params.constraintLinkNames{ii});
end

JCoM  =  wbm_jacobian(RotBase,PosBase,qj,'com');
JCoM  =  JCoM(1:3,:);

% centroidal momentum jacobian
JhBase              = zeros(6,6);
JhJoint             = zeros(6,params.ndof);

for ii = 1:6
    
NuBase              = zeros(6,1);
NuBase(ii)          = 1;
JhBase(:,ii)        = wbm_centroidalMomentum(RotBase, PosBase, qj, zeros(params.ndof,1), NuBase);
end

for ii = 1:params.ndof

dqj_Jh              = zeros(params.ndof,1);
dqj_Jh(ii)          = 1;
JhJoint(:,ii)       = wbm_centroidalMomentum(RotBase, PosBase, qj, dqj_Jh, zeros(6,1));
end

Jh                  = [JhBase JhJoint]; 
Jh                  = Jh(4:6,:); 

%% CoM position error, feet position and orientation error, angular momentum integral error
xCoM          = wbm_forwardKinematics(RotBase,PosBase,qj,'com');
deltaPosCoM   = xCoM(1:3) - desired_x_dx_ddx_CoM(:,1);
 
lsole    = wbm_forwardKinematics(RotBase,PosBase,qj,'l_sole');
rsole    = wbm_forwardKinematics(RotBase,PosBase,qj,'r_sole');

% feet orientation
[PosLfoot,RotLfoot]    = frame2posrot(lsole);
[PosRfoot,RotRfoot]    = frame2posrot(rsole);

% parametrize the rotation matrices with Euler angles
[TRfoot,OriRfoot]      = parametrization(RotRfoot);
[TLfoot,OriLfoot]      = parametrization(RotLfoot);

PoseLeftFoot           = [PosLfoot; OriLfoot'];
PoseRightFoot          = [PosRfoot; OriRfoot'];

% initial feet position and orientation
lfoot_ini                    = params.PoseLFootQuatInit ;
rfoot_ini                    = params.PoseRFootQuatInit ;

[PosRfootIni,RotRfootIni]    = frame2posrot(rfoot_ini);
[PosLfootIni,RotLfootIni]    = frame2posrot(lfoot_ini);

[~,OriLfootIni]              = parametrization(RotLfootIni);
[~,OriRfootIni]              = parametrization(RotRfootIni);

PoseLeftFootIni              = [PosLfootIni; OriLfootIni'];
PoseRightFootIni             = [PosRfootIni; OriRfootIni'];
  
 if       sum(params.feet_on_ground) == 1 && params.feet_on_ground(1) == 1
     
 TtildeLFoot       = [eye(3)   zeros(3);
                      zeros(3)  TLfoot];
 deltaPosFeet      = TtildeLFoot*(PoseLeftFoot-PoseLeftFootIni);
 
 elseif   sum(params.feet_on_ground) == 1 && params.feet_on_ground(2) == 1
     
 TtildeRFoot       = [eye(3)   zeros(3);
                      zeros(3)  TRfoot];
 deltaPosFeet      = TtildeRFoot*(PoseRightFoot-PoseRightFootIni);
 
 elseif   sum(params.feet_on_ground) == 2
     
 TtildeLFoot       = [eye(3) zeros(3);
                      zeros(3)  TLfoot];
 TtildeRFoot       = [eye(3) zeros(3);
                      zeros(3)  TRfoot];
 Ttilde            = [TtildeLFoot   zeros(6);
                      zeros(6)   TtildeRFoot];
 deltaPosFeet      = Ttilde*[PoseLeftFoot-PoseLeftFootIni;
                             PoseRightFoot-PoseRightFootIni];
 end 

deltaIntAngMom     = params.JhReduced(4:6,:)*(qj-params.qjInit); 
 
%% CoM velocities error, feet velocities error, angular momentum error
deltaVelFeet       = Jc*Nu;
deltaVelCoM        = JCoM*Nu - desired_x_dx_ddx_CoM(:,2);
deltaAngMom        = Jh*Nu;

%% Joints velocities; base velocities; base quaternion derivative
dqj                = Nu(7:end);
NuBaseIkin         = Nu(1:6);                               
dquatBase          = quaternionDerivative(transpose(RotBase)*NuBaseIkin(4:end), BasePose(4:end));
 
%% Time derivative of jacobians (this will assume that the time derivative of Jh multiplied with Nu is ~ 0)
for ii=1:params.numConstraints
    
    dJcNu(6*(ii-1)+1:6*ii,:) = wbm_djdq(RotBase,PosBase,qj,dqj,NuBaseIkin,params.constraintLinkNames{ii});
end

dJCoMNu       =  wbm_djdq(RotBase,PosBase,qj,dqj,NuBaseIkin,'com');
dJCoMNu       =  dJCoMNu(1:3);

%% Joint+base accelerations from inverse kinematics
% first task: constraints at feet 
feetDynamics     = -dJcNu -Kcorr_feet*deltaPosFeet -Kcorr_feetVel*deltaVelFeet;
pinvJc           = pinv(Jc,toll);
NullFeet         = eye(ndof+6) - pinvJc*Jc;

% second task: Centroidal dynamics
JCentr           = [JCoM; Jh];
dJCentrNu        = [dJCoMNu; zeros(3,1)];
CentrDynamics    = -dJCentrNu + [desired_x_dx_ddx_CoM(:,3); zeros(3,1)] - [Kcorr_CoM*deltaPosCoM; Kcorr_Ang*deltaIntAngMom]...
                  - [Kcorr_CoMVel*deltaVelCoM; Kcorr_AngVel*deltaAngMom]  -JCentr*pinvJc*feetDynamics;
pinvJCentr       = pinv(JCentr*NullFeet,toll);
NullCentr        = eye(ndof+6) - pinvJCentr*JCentr*NullFeet;

% third task: desired posture
JPosture         = [zeros(ndof,6) eye(ndof)];

postureDynamics  = -Kcorr_dqj*Nu(7:end) -Kcorr_qj*(qj-params.qjInit)...
                   -JPosture*(pinv(Jc,toll)*feetDynamics +NullFeet*pinvJCentr*CentrDynamics);

dNuThirdTask     = pinv(JPosture*NullFeet*NullCentr,toll)*postureDynamics;
dNuSecondTask    = pinvJCentr*CentrDynamics + NullCentr*dNuThirdTask;
dNuFirstTask     = pinvJc*feetDynamics + NullFeet*dNuSecondTask;

%% State to be integrated
dChiIkin         = [NuBaseIkin(1:3); dquatBase; dqj; dNuFirstTask];

%% Parameters for visualization
ObtainedCoMTrajectory             = [xCoM(1:3); JCoM*Nu; (JCoM*dNuFirstTask+dJCoMNu)];
DesiredCoMTrajectory              = [desired_x_dx_ddx_CoM(:,1); desired_x_dx_ddx_CoM(:,2); desired_x_dx_ddx_CoM(:,3)];
visualizeIkinParam.CoMTrajectory  = [DesiredCoMTrajectory; ObtainedCoMTrajectory];
% momentum and feet position error 
deltaMomentum                     = [params.MInit(1,1)*(JCoM*Nu-desired_x_dx_ddx_CoM(:,2)); Jh*Nu];
visualizeIkinParam.H_Feet_Error   = [deltaMomentum; deltaPosFeet];

end
