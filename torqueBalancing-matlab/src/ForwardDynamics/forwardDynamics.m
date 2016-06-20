function [dchi,visualization] = forwardDynamics(t,chi,CONFIG)
%FORWARDDYNAMICS is the function that will be integrated in the forward 
%                dynamics integrator.
%
%             [dchi,visualization] = FORWARDDYNAMICS(t,chi,config) takes
%             as input the current time step, T; the robot state, CHI
%             [13+2ndof x 1]; the structure CONFIG which contains the
%             user-defined parameters.
%             The output are the vector to be integrated, DCHI [13+2ndof x1]
%             and the structure VISUALIZATION which contains all the parameters 
%             used to generate the plots in the visualizer.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% waitbar
waitbar(t/CONFIG.tEnd,CONFIG.wait)

%% Robot Configuration
ndof                  = CONFIG.ndof;
gains                 = CONFIG.gainsInit;
qjInit                = CONFIG.qjInit;
xCoMRef               = CONFIG.xCoMRef;

%% Robot State
STATE                 = robotState(chi,CONFIG);
RotBase               = STATE.RotBase;
omegaBaseWorld        = STATE.omegaBaseWorld;
quatBase              = STATE.quatBase;
VelBase               = STATE.VelBase;
dqj                   = STATE.dqj;
qj                    = STATE.qj;
PosBase               = STATE.PosBase;

%% Set the robot state (for wbm functions)
wbm_setWorldFrame(RotBase,PosBase,[0 0 -9.81]')
wbm_updateState(qj,dqj,[VelBase;omegaBaseWorld]);

%% Robot Dynamics
DYNAMICS              = robotDynamics(STATE,CONFIG);
Jc                    = DYNAMICS.Jc;
M                     = DYNAMICS.M;
h                     = DYNAMICS.h;
H                     = DYNAMICS.H;
m                     = M(1,1);

%% Robot Forward kinematics
FORKINEMATICS         = robotForKinematics(STATE,DYNAMICS);
RFootPoseEul          = FORKINEMATICS.RFootPoseEul;
LFootPoseEul          = FORKINEMATICS.LFootPoseEul;
xCoM                  = FORKINEMATICS.xCoM;

%% Joint limits check
% jointLimitsCheck(qj,t);

%% Interpolation for joint reference trajectory with ikin
if CONFIG.jointRef_with_ikin == 1
trajectory.JointReferences         = interpInverseKinematics(t,CONFIG.ikin);
else
trajectory.JointReferences.ddqjRef = zeros(ndof,1);
trajectory.JointReferences.dqjRef  = zeros(ndof,1);
trajectory.JointReferences.qjRef   = qjInit;
end

%% CoM trajectory generator
trajectory.desired_x_dx_ddx_CoM    = trajectoryGenerator(xCoMRef,t,CONFIG);

%% Linearization and gains tuning procedure 
if CONFIG.gains_tuning == 1 
    
cont           = 0;
gains          = CONFIG.gainsVec;
vectorOfPoints = CONFIG.vectorOfPoints;
 
for k = 2:length(vectorOfPoints)
  
if t>=CONFIG.ikin.t(vectorOfPoints(k-1)) && t<=CONFIG.ikin.t(vectorOfPoints(k))
    
cont  = k;
end
end

% define the scalar value for interpolation
if (CONFIG.ikin.t(vectorOfPoints(cont))-CONFIG.ikin.t(vectorOfPoints(cont-1))) == 0

delta = 0;

else
delta = (t-CONFIG.ikin.t(vectorOfPoints(cont-1)))/(CONFIG.ikin.t(vectorOfPoints(cont))-CONFIG.ikin.t(vectorOfPoints(cont-1)));
end

% gains matrix interpolation in the Lie group of symmetric positve definite
% matrices
Kpn = expm(delta*logm(reshape(gains.impedances(:,cont),[ndof,ndof]))+(1-delta)*logm(reshape(gains.impedances(:,cont-1),[ndof,ndof])));
Kdn = expm(delta*logm(reshape(gains.dampings(:,cont),[ndof,ndof]))+(1-delta)*logm(reshape(gains.dampings(:,cont-1),[ndof,ndof])));
Kpx = expm(delta*logm(reshape(gains.intMomentumGains(:,cont),[6,6]))+(1-delta)*logm(reshape(gains.intMomentumGains(:,cont-1),[6,6])));
Kdx = expm(delta*logm(reshape(gains.MomentumGains(:,cont),[6,6]))+(1-delta)*logm(reshape(gains.MomentumGains(:,cont-1),[6,6])));
  
lin = jointSpaceLinearization(CONFIG,trajectory.JointReferences.qjRef ,'normal');
% reset the world frame
wbm_setWorldFrame(RotBase,PosBase,[0 0 -9.81]')

gains.impedances        = Kpn;
gains.dampings          = Kdn;
gains.intMomentumGains  = Kpx;
gains.MomentumGains     = Kdx;  

gains.KSn = lin.ACartesian*Kpx*lin.BCartesian + lin.ANull*Kpn*lin.BNull;
gains.KDn = lin.ACartesian*Kdx*lin.BCartesian + lin.ANull*Kdn*lin.BNull;
visualization.gainTun   = gains;
end

%%%%%%%%%%%%%%% LINEARIZATION DEBUG AND STABILITY ANALYSIS %%%%%%%%%%%%%%%%
if CONFIG.linearizationDebug  == 1
 
linearization            = jointSpaceLinearization(CONFIG,qj,'normal');
% reset the world frame
wbm_setWorldFrame(RotBase,PosBase,[0 0 -9.81]')

% linearized joint accelerations
visualization.ddqjLin    = trajectory.JointReferences.ddqjRef-linearization.KS*(qj-trajectory.JointReferences.qjRef)...
                          -linearization.KD*(dqj-trajectory.JointReferences.dqjRef);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
%% Balancing controller
controlParam    =  initController(gains,trajectory,DYNAMICS,FORKINEMATICS,CONFIG,STATE);
tau             =  controlParam.tau;
fc              =  controlParam.fc;

%% State derivative computation
omegaWorldBase  = transpose(RotBase)*omegaBaseWorld;                               
dquatBase       = quaternionDerivative(omegaWorldBase,quatBase);      
NuQuat          = [VelBase;dquatBase;dqj];
dNu             = M\(Jc'*fc + [zeros(6,1); tau]-h);
% state derivative 
dchi            = [NuQuat;dNu];

%% Parameters for visualization
visualization.ddqjNonLin  = dNu(7:end);
visualization.dqj         = dqj;
visualization.qj          = qj;
visualization.JointRef    = trajectory.JointReferences;
visualization.xCoM        = xCoM;
visualization.poseFeet    = [LFootPoseEul;RFootPoseEul];
visualization.H           = H;
visualization.HRef        = [m*trajectory.desired_x_dx_ddx_CoM(:,2);zeros(3,1)];
visualization.fc          = fc;
visualization.f0          = controlParam.f0;
visualization.tau         = tau;

end
