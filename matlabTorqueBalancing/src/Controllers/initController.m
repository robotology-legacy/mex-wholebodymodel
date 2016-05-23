function controlParam  = initController(gains,trajectory,dynamics,forKinematics,config,state)
%INITCONTROLLER is the function which contains all the available
%               balancing controllers in MATLAB.
%               controlParam  = INITCONTROLLER(gains,trajectory,
%               dynamics,forKinematics,config) takes as input all the 
%               structures previously defined (dynamics, gains, ecc...). 
%               The output is the structure CONTROLPARAM which contains the
%               control torques TAU, the contact forces FC and other
%               parameters for visualization.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% feet correction gains
KCorrPos              = 2.5;
KCorrVel              = 2*sqrt(KCorrPos);

% Config parameters
ndof                  = config.ndof;
SoTController         = config.SoTController;
JSController          = config.JSController;
feet_on_ground        = config.feet_on_ground;
use_QPsolver          = config.use_QPsolver;
InitForKinematics     = config.InitForKinematics;

% Dynamics parameters
dJcNu                 = dynamics.dJcNu;
M                     = dynamics.M;
Jc                    = dynamics.Jc;

% Forward Kinematics parameters
VelFeet               = forKinematics.VelFeet;
TLfoot                = forKinematics.TLfoot;
TRfoot                = forKinematics.TRfoot;
RFootPoseEul          = forKinematics.RFootPoseEul;
LFootPoseEul          = forKinematics.LFootPoseEul;

%% CONTROLLERS
if     SoTController    == 1

controlParam          = stackOfTaskController(config,gains,trajectory,dynamics,forKinematics,state); 

if     use_QPsolver == 1 
% Quadratic programming solver for the contact forces Nullspace   
controlParam.fcDes    = QPSolver(controlParam,config,forKinematics);
end
controlParam.tau      = controlParam.tauModel + controlParam.Sigma*controlParam.fcDes;

elseif JSController == 1        
% Centroidal coordinates transformation   
dynamics.centroidal   = centroidalConversion(dynamics,forKinematics);
controlParam          = jointSpaceController(config,gains,trajectory,dynamics.centroidal,state);
end

%% Feet pose error
DeltaPoseRFoot = TRfoot*(RFootPoseEul-InitForKinematics.RFootPoseEul); 
DeltaPoseLFoot = TLfoot*(LFootPoseEul-InitForKinematics.LFootPoseEul);

if     feet_on_ground(1) == 1 && feet_on_ground(2) == 0

DeltaPoseFeet  = DeltaPoseLFoot;
      
elseif feet_on_ground(1) == 0 && feet_on_ground(2) == 1
     
DeltaPoseFeet  = DeltaPoseRFoot;          

elseif feet_on_ground(1) == 1 && feet_on_ground(2) == 1
    
DeltaPoseFeet  = [DeltaPoseLFoot;DeltaPoseRFoot];    
end

%% Real contact forces computation
S                = [zeros(6,ndof);
                    eye(ndof,ndof)]; 
JcMinv           =  Jc/M;
JcMinvS          =  JcMinv*S;

if config.visualize_stability_analysis_results == 1
controlParam.fc  = controlParam.fcDes;
else
controlParam.fc  = (JcMinv*transpose(Jc))\(JcMinv*h -JcMinvS*tau -dJcNu -KCorrVel.*VelFeet-KCorrPos.*DeltaPoseFeet);
end

end
