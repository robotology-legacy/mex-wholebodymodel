%% forwardDynamics
% this is the function that will be integrated in the forward dynamics integrator.
% It calculates the forward dynamics of the model loaded in the wholeBodyInterface
% from the URDF description. The dynamic model is described as an explicit ordinary 
% differential equation of the form:
%
%              dchi = forwardDynamics(t,chi)
%
% where chi is the variable to be integrated. For a floating base
% articulated chain, the variable chi contains the following
% subvariables:
%
% PosBase:         the cartesian position of the base (R^3)
% quatBase:        the quaternion describing the orientation of the base (global parametrization of SO(3))
% qj:              the joint positions (R^ndof)
% VelBase:         the cartesian velocity of the base (R^3)
% omegaBaseWorld:  the velocity describing the orientation of the base (SO(3))
% dqj:             the joint velocities (R^ndof)
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function [dchi,visualParam] = forwardDynamics(t,chi,params)
% general parameters; sets the base orientation in stateDemux.m 
% as positions + quaternions (1) or transformation matrix (0)
waitbar(t/params.tEnd,params.wait)
ndof                             = params.ndof;
params.demux.baseOrientationType = 1;                           
[basePose,qj,baseVelocity,dqj]   = stateDemux(chi,params);

% base position and orientation (in quaternions) and base velocity; 
% conversion of the base orientation into a rotation matrix; state velocity
PosBase                          = basePose(1:3,:);
quatBase                         = basePose(4:7,:);
VelBase                          = baseVelocity(1:3,:);
omegaBaseWorld                   = baseVelocity(4:6,:);
[~,RotBase]                      = frame2posrot(basePose);
Nu                               = [VelBase;omegaBaseWorld;dqj];
% normalize quaternions to avoid numerical errors
%quatBase                        = quatBase/norm(quatBase);
 
% Robot dynamics: mass matrix, generalized bias forces, Coriolis terms,
% centroidal momentum, contacts and CoM jacobians, jacobians derivative
M                                = wbm_massMatrix(RotBase,PosBase,qj); 
h                                = wbm_generalisedBiasForces(RotBase,PosBase,qj,dqj,[VelBase;omegaBaseWorld]);
g                                = wbm_generalisedBiasForces(RotBase,PosBase,qj,zeros(ndof,1),zeros(6,1));
CNu                              = h-g;
H                                = wbm_centroidalMomentum(RotBase,PosBase,qj,dqj,[VelBase;omegaBaseWorld]);
JCoM                             = wbm_jacobian(RotBase,PosBase,qj,'com');
Jc                               = zeros(6*params.numConstraints,6+ndof);
dJcNu                            = zeros(6*params.numConstraints,1);

for i=1:params.numConstraints
    
    Jc(6*(i-1)+1:6*i,:)          = wbm_jacobian(RotBase,PosBase,qj,params.constraintLinkNames{i});
    dJcNu(6*(i-1)+1:6*i,:)       = wbm_djdq(RotBase,PosBase,qj,dqj,[VelBase;omegaBaseWorld],params.constraintLinkNames{i});
    
end

% forward kinematics: feet pose, CoM position and velocity
PoseLFootQuat                    = wbm_forwardKinematics(RotBase,PosBase,qj,'l_sole');
PoseRFootQuat                    = wbm_forwardKinematics(RotBase,PosBase,qj,'r_sole');
CoM                              = wbm_forwardKinematics(RotBase,PosBase,qj,'com');
xCoM                             = CoM(1:3);
dCoM                             = JCoM*Nu;
dxCoM                            = dCoM(1:3);

%% Joints limits check
limits     = params.limits;
l_min      = limits(:,1);
l_max      = limits(:,2);
tol        = 0.01;
res        = qj < l_min + tol | qj > l_max - tol;
res        = sum(res);

if res == 0
else
% disp('Joint limits reached at time:')    
% disp(t)
% error('Joint limits reached '); 
end

%% Feet corrections to avoid numerical integration errors
% feet correction gains
K_corr_pos                       = 2.5;
K_corr_vel                       = 2*sqrt(K_corr_pos);

% feet current position and orientation (rotation matrix)
[posLfoot,RotBaseLfoot]          = frame2posrot(PoseLFootQuat);
[posRfoot,RotBaseRfoot]          = frame2posrot(PoseRFootQuat);

% orientation is parametrized with euler angles
[~,oriLfoot]                     = parametrization(RotBaseLfoot);
[~,oriRfoot]                     = parametrization(RotBaseRfoot);
PoseLFoot                        = [posLfoot; oriLfoot'];
PoseRFoot                        = [posRfoot; oriRfoot'];

% feet initial position and orientation
PoseLFootQuatInit                = params.PoseLFootQuatInit;
PoseRFootQuatInit                = params.PoseRFootQuatInit;
[posInitlfoot,RotBaseInitLfoot]  = frame2posrot(PoseLFootQuatInit);
[posInitrfoot,RotBaseInitRfoot]  = frame2posrot(PoseRFootQuatInit);
[~,oriInitLfoot]                 = parametrization(RotBaseInitLfoot);
[~,oriInitRfoot]                 = parametrization(RotBaseInitRfoot);
PoseInitLfoot                    = [posInitlfoot; oriInitLfoot'];
PoseInitRfoot                    = [posInitrfoot; oriInitRfoot'];
  
% error between initial and current feet position and orientation
if     params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 0
     
       DeltaPoseFeet = PoseLFoot-PoseInitLfoot;
 
elseif params.feet_on_ground(1) == 0 && params.feet_on_ground(2) == 1
     
       DeltaPoseFeet = PoseRFoot-PoseInitRfoot;       

elseif params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1
    
       DeltaPoseFeet = [(PoseLFoot-PoseInitLfoot);...
                        (PoseRFoot-PoseInitRfoot)];    
end

%% Centroidal transformation (for joint space controller only)
if params.JSController == 1
% transformation matrix for centroidal
[T,dT]                     = centroidalTransformationT_TDot(xCoM,PosBase,dxCoM,VelBase,M);
% conversion to the centroidal frame of reference
[M, CNu, g, Jc, dJcNu, Nu] = fromFloatingToCentroidalDynamics(M, h, g, Jc, dJcNu, Nu, T, dT);
end

%% Gains, constraints and trajectory variables
gains                              = params.gains;
constraints                        = params.constraints;
trajectory                         = params.trajectory;

% parameters for the balancing controller
dynamics.LFootPose                 = PoseLFootQuat;
dynamics.RFootPose                 = PoseRFootQuat;
dynamics.M                         = M;
dynamics.g                         = g;
dynamics.CNu                       = CNu;
dynamics.dJcNu                     = dJcNu;
dynamics.H                         = H;
dynamics.qj                        = qj;
dynamics.xCoM                      = xCoM;
dynamics.dxCoM                     = dxCoM;
dynamics.JCoM                      = JCoM;
dynamics.Nu                        = Nu;
dynamics.Jc                        = Jc;

% CoM trajectory generator
trajectory.desired_x_dx_ddx_CoM    = generTraj(params.CoMInit(1:3),t,trajectory);
errorCoM                           = xCoM-trajectory.desired_x_dx_ddx_CoM(:,1);

if params.jointRef_with_ikin == 1 || params.JSController == 1
% ikin trajectory interpolation
ikin                               = params.ikin;
trajectory.JointReferences         = interpolateIkin(t,ikin);
else
trajectory.JointReferences.ddqjRef = zeros(ndof,1);
trajectory.JointReferences.dqjRef  = zeros(ndof,1);
trajectory.JointReferences.qjRef   = params.qjInit;
end

%% Balancing controller
if params.SoTController    == 1

[tau,f0,ddqjNonLin] = stackOfTaskController(params, constraints, gains, trajectory, dynamics);    

elseif params.JSController == 1        

[tau,f0,ddqjNonLin] = JointSpaceController(params, dynamics, gains, trajectory.JointReferences);
end

%% Real contact forces computation
S               = [ zeros(6,ndof);
                    eye(ndof,ndof)];                
JcMinv          = Jc/M;
JcMinvS         = JcMinv*S;
fc              = (JcMinv*transpose(Jc))\(JcMinv*h -JcMinvS*tau -dJcNu -K_corr_vel.*Jc*Nu -K_corr_pos.*DeltaPoseFeet);

%% State derivative computation
% Need to calculate the quaternions derivative
omegaWorldBase = transpose(RotBase)*omegaBaseWorld;                               
dquatBase      = quaternionDerivative(omegaWorldBase,quatBase);       

NuQuat         = [VelBase;dquatBase;dqj];
dNu            = M\(Jc'*fc + [zeros(6,1); tau]-h);

% state to be integrated
dchi           = [NuQuat;dNu];  

%% Visualization parameters
% these parameters are used by the visualization function to plot the
% graphics of the state integration
 visualParam.ddqjNonLin = ddqjNonLin;
 visualParam.dqj        = dqj;
 visualParam.JointRef   = trajectory.JointReferences;
 visualParam.Href       = [M(1,1)*trajectory.desired_x_dx_ddx_CoM(:,2);zeros(3,1)];
 visualParam.H          = H;
 visualParam.pos_feet   = [PoseLFootQuat;PoseRFootQuat];
 visualParam.fc         = fc;
 visualParam.tau        = tau;
 visualParam.qj         = qj;
 visualParam.error_com  = errorCoM;
 visualParam.f0         = f0;
 visualParam.xCoM       = xCoM;

end
