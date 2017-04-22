function dchi = inverseKinematics(t,chi,MODEL)
%INVERSEKINEMATICS evaluates the desired momentum trajectory for the
%                  robot and generates proper joint references. The joints 
%                  accelerations are obtained using a task-based structure:
%                  the first task is to guarantee the contact constraints,
%                  the second task is to follow a desired momentum trajectory 
%                  while the third task is to keep the initial posture. Joint
%                  reference position and velocity are obtained by means of 
%                  a double fixed step integrator.
%
% Format:  dchi = INVERSEKINEMATICS(t,chi,MODEL)
%
% Inputs:  - t: current time [s]
%          - chi: robot state vector [13+2*ndof x 1];
%          - MODEL: it is a structure defining the robot model;        
%
% Output:  - dchi: state derivative [13+2*ndof x 1];
%         
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% report current time in the waitbar's message field
waitbar(t/MODEL.CONFIG.tEnd,MODEL.wait,['Current time: ',sprintf('%0.3f',t),' [s]'])

% setup configuration parameters 
pinv_tol          = MODEL.CONFIG.pinv_tol;
feet_on_ground    = MODEL.CONFIG.feet_on_ground;
ndof              = MODEL.ndof;
% references parameters for integration
qjRef             = MODEL.REFERENCES.qjRef;
xCoMRef           = MODEL.REFERENCES.xCoMRef;
feetRef           = MODEL.REFERENCES.feetRef;
% demux the current robot state 
STATE             = robotState(chi,MODEL);
basePose          = STATE.basePose;
qj                = STATE.qj;
w_R_b             = STATE.w_R_b;
nu                = STATE.nu;
dqj               = STATE.dqj;
nu_b              = nu(1:6);
% compute all dynamics parameters
DYNAMICS          = robotDynamics(STATE,MODEL);
m                 = DYNAMICS.M(1,1);
Jc                = DYNAMICS.Jc;
JH                = DYNAMICS.JH;
JCoMPose          = DYNAMICS.JCoM;
JCoM              = JCoMPose(1:3,:);
dJCoM_nuPose      = DYNAMICS.dJCoM_nu;
dJCoM_nu          = dJCoM_nuPose(1:3);
dJH_nu            = DYNAMICS.dJH_nu;
dJc_nu            = DYNAMICS.dJc_nu;
JG                = JH(:,7:end)-JH(:,1:6)*(eye(6)/Jc(1:6,1:6))*Jc(1:6,7:end);
% compute robot forward kinematics
FORKINEMATICS     = robotForKinematics(STATE,DYNAMICS);
xCoM              = FORKINEMATICS.xCoM;
TL                = FORKINEMATICS.TL;
TR                = FORKINEMATICS.TR;
poseRFoot_ang     = FORKINEMATICS.poseRFoot_ang;
poseLFoot_ang     = FORKINEMATICS.poseLFoot_ang;
% gains for inverse kinematics
GAINS             = MODEL.GAINS;
KpFeet            = GAINS.KpFeet ;
intMomentumGains  = GAINS.intMomentumGains;
impedances        = GAINS.impedances;
KdFeet            = 2*sqrt(KpFeet);
momentumGains     = GAINS.momentumGains;
dampings          = GAINS.dampings;

%% CoM trajectory generator
x_dx_ddx_CoMDes   = trajectoryGenerator(xCoMRef,t,MODEL.CONFIG);

%% Joint limits check
if MODEL.CONFIG.use_jointLimits
    jointLimitsCheck(qj,t);
end

%% %%%%%%%%%%%%%%%%% TASK BASED INVERSE KINEMATICS %%%%%%%%%%%%%%%%%%%%% %%
%% TASK #1: enforce the contact constraints at feet.
% feet pose error
errorPoseRFoot        = TR*(poseRFoot_ang-feetRef(7:end));
errorPoseLFoot        = TL*(poseLFoot_ang-feetRef(1:6));
% one or two feet balancing selector
if feet_on_ground(1) == 1 && feet_on_ground(2) == 0
    % left foot balancing
    errorPoseFeet     = errorPoseLFoot;
    
elseif feet_on_ground(1) == 0 && feet_on_ground(2) == 1
    % right foot balancing 
    errorPoseFeet     = errorPoseRFoot;
    
elseif sum(feet_on_ground) == 2
    % balancing on both feet
    errorPoseFeet     = [errorPoseLFoot;errorPoseRFoot];
end
% feet velocity error (reference velocity is zero)
errorVelFeet          = Jc*nu;

%% Desired feet dynamics
feetDynamics          = -dJc_nu -KpFeet*errorPoseFeet -KdFeet*errorVelFeet;
% null space projector
NullJc                =  eye(ndof+6) -pinv(Jc,pinv_tol)*Jc;

%% TASK #2: obtain a desired momentum dynamics
% error on the integral of momentum
errorIntMomentum      = [m*(xCoM - x_dx_ddx_CoMDes(:,1)); ...
                         JG(4:6,:)*(qj-qjRef)];
% error on the momentum
errorMomentum         = [m*(JCoM*nu - x_dx_ddx_CoMDes(:,2)); ...
                         JH(4:6,:)*nu];
% desired momentum derivative
desMomentumDerivative = [m*x_dx_ddx_CoMDes(:,3); ...
                         zeros(3,1)];
                     
%% Desired momentum dynamics
momentumDynamics      = -dJH_nu +desMomentumDerivative -intMomentumGains*errorIntMomentum -momentumGains*errorMomentum ...
                        -JH*pinv(Jc,pinv_tol)*feetDynamics;
% null space projector
NullH                 =  eye(ndof+6) -pinv(JH*NullJc,pinv_tol)*JH*NullJc;

%% TASK #3: keep a desired robot posture
% selector of joint dynamics (transpose of S)
St                    = [zeros(ndof,6) eye(ndof,ndof)];
% joint desired dynamics
jointDynamics         = -dampings*nu(7:end) -impedances*(qj-qjRef)...
                        -St*pinv(Jc,pinv_tol)*feetDynamics -St*NullJc*pinv(JH*NullJc,pinv_tol)*momentumDynamics;
                    
%% ROBOT + FLOATING BASE ACCELERATIONS
dnu_thirdTask         = pinv(St*NullJc*NullH,pinv_tol)*jointDynamics;
dnu_secondTask        = pinv(JH*NullJc,pinv_tol)*momentumDynamics + NullH*dnu_thirdTask;
dnu                   = pinv(Jc,pinv_tol)*feetDynamics  + NullJc*dnu_secondTask;
% state derivative: compute the quaternion derivative
dqt_b                 = dquat(basePose(4:end),transpose(w_R_b)*nu_b(4:end));
dchi                  = [nu_b(1:3); dqt_b; dqj; dnu];

%% %%%%%%%%%%%%%%%%%%%%%%%  VISUALIZATION  %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% store parameters for visualizing integration results
if MODEL.CONFIG.enable_visualTool
    % adjust feet position error in case of one foot balancing
    if sum(feet_on_ground) == 1
        errorPoseFeet = [errorPoseFeet; zeros(6,1)];
    end
    % update time index with the current time 
    t_total       =  transpose(MODEL.CONFIG.tStart:MODEL.CONFIG.sim_step:MODEL.CONFIG.tEnd);
    currTimeIndex = sum(t_total <= t);
    % load values from .mat file
    GRAPHICS = matfile('./media/storedValuesIkin.mat','Writable',true);
    GRAPHICS.feetError(:,currTimeIndex)  = errorPoseFeet;
    GRAPHICS.CoMError(:,currTimeIndex)   = [(xCoM              - x_dx_ddx_CoMDes(:,1));...
                                            (JCoM*nu           - x_dx_ddx_CoMDes(:,2)); ...
                                            (JCoM*dnu+dJCoM_nu - x_dx_ddx_CoMDes(:,3))];
    GRAPHICS.HError(:,currTimeIndex)     = errorMomentum;
    GRAPHICS.CoMRef(:,currTimeIndex)     = x_dx_ddx_CoMDes(:,1);
end

end
