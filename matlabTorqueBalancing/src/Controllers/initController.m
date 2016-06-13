function controlParam  = initController(gains,trajectory,DYNAMICS,FORKINEMATICS,CONFIG,STATE)
%INITCONTROLLER is the initialization function for iCub balancing controllers
%               in Matlab.
%
%               controlParam  = INITCONTROLLER(gains,trajectory,dynamics,
%               forKinematics,config,state) takes as input the control gains,
%               the joint reference trajectory, all the configuration parameters
%               and the robot dynamics, forward kinematics and state. The output
%               is the structure CONTROLPARAM which contains the control torques
%               TAU, the contact forces FC and other parameters for visualization.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
%% Feet correction gains
KCorrPos              = 2.5;
KCorrVel              = 2*sqrt(KCorrPos);

%% Config parameters
ndof                  = CONFIG.ndof;
feet_on_ground        = CONFIG.feet_on_ground;
use_QPsolver          = CONFIG.use_QPsolver;
initForKinematics     = CONFIG.initForKinematics;
S                     = [zeros(6,ndof);
                         eye(ndof,ndof)]; 

%% Dynamics 
dJcNu                 = DYNAMICS.dJcNu;
M                     = DYNAMICS.M;
Jc                    = DYNAMICS.Jc;
h                     = DYNAMICS.h;
JcMinv                = Jc/M;
JcMinvS               = JcMinv*S;

%% Forward Kinematics
VelFeet               = FORKINEMATICS.VelFeet;
TLfoot                = FORKINEMATICS.TLfoot;
TRfoot                = FORKINEMATICS.TRfoot;
RFootPoseEul          = FORKINEMATICS.RFootPoseEul;
LFootPoseEul          = FORKINEMATICS.LFootPoseEul;
DeltaPoseRFoot        = TRfoot*(RFootPoseEul-initForKinematics.RFootPoseEul);
DeltaPoseLFoot        = TLfoot*(LFootPoseEul-initForKinematics.LFootPoseEul);

%% BALANCING CONTROLLERS
if     strcmp(CONFIG.controller,'StackOfTask') == 1

controlParam          = stackOfTaskController(CONFIG,gains,trajectory,DYNAMICS,FORKINEMATICS,STATE); 

if     use_QPsolver == 1 

% Quadratic programming solver for the nullspace of contact forces  
controlParam.fcDes    = QPSolver(controlParam,CONFIG,FORKINEMATICS);
end

controlParam.tau      = controlParam.tauModel + controlParam.Sigma*controlParam.fcDes;

elseif strcmp(CONFIG.controller,'JointSpace') == 1 
    
% Centroidal coordinates transformation   
centroidalDynamics    = centroidalConversion(DYNAMICS,FORKINEMATICS,STATE);
controlParam          = jointSpaceController(CONFIG,gains,trajectory,centroidalDynamics,STATE);
end

%% Feet pose correction
% this will avoid numerical errors during the forward dynamics integration
if     feet_on_ground(1) == 1 && feet_on_ground(2) == 0

DeltaPoseFeet  = DeltaPoseLFoot;
      
elseif feet_on_ground(1) == 0 && feet_on_ground(2) == 1
     
DeltaPoseFeet  = DeltaPoseRFoot;          

elseif feet_on_ground(1) == 1 && feet_on_ground(2) == 1
    
DeltaPoseFeet  = [DeltaPoseLFoot;DeltaPoseRFoot];    
end

%% REAL CONTACT FORCES COMPUTATION
% for the linearization analysis, the desired contact forces are used
% instead of the real ones (stack of task only)
if CONFIG.visualize_stability_analysis_results == 1
controlParam.fc  = controlParam.fcDes;
else
controlParam.fc  = (JcMinv*transpose(Jc))\(JcMinv*h -JcMinvS*controlParam.tau -dJcNu -KCorrVel.*VelFeet-KCorrPos.*DeltaPoseFeet);
end

end