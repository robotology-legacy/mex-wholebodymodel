function controlParam  = runController(gain,trajectory,DYNAMICS,FORKINEMATICS,CONFIG,STATE)
%RUNCONTROLLER  is the initialization function for iCub balancing controllers
%               in MATLAB.
%
% controlParam  = RUNCONTROLLER(gains,trajectory,DYNAMICS,FORKINEMATICS,
% CONFIG,STATE) takes as input the control gains, the joint reference 
% trajectory, all the configuration parameters and the robot dynamics, 
% forward kinematics and state. The output is the structure controlparam 
% which contains the control torques tau, the contact forces fc and other 
% parameters for visualization.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
%% Feet correction gains
KpFeet                = gain.corrPosFeet;
KdFeet                = 2*sqrt(KpFeet);

%% Configuration parameters
ndof                  = CONFIG.ndof;
feet_on_ground        = CONFIG.feet_on_ground;
initForKinematics     = CONFIG.initForKinematics;
S                     = [zeros(6,ndof);
                         eye(ndof,ndof)];

%% Robot dynamics
dJc_nu                = DYNAMICS.dJc_nu;
M                     = DYNAMICS.M;
Jc                    = DYNAMICS.Jc;
h                     = DYNAMICS.h;
JcMinv                = Jc/M;
JcMinvS               = JcMinv*S;

%% Robot forward Kinematics
v_feet                = FORKINEMATICS.v_feet;
TL                    = FORKINEMATICS.TL;
TR                    = FORKINEMATICS.TR;
poseRFoot_ang         = FORKINEMATICS.poseRFoot_ang;
poseLFoot_ang         = FORKINEMATICS.poseLFoot_ang;
deltaPoseRFoot        = TR*(poseRFoot_ang-initForKinematics.poseRFoot_ang);
deltaPoseLFoot        = TL*(poseLFoot_ang-initForKinematics.poseLFoot_ang);

%% BALANCING CONTROLLER
% Centroidal coordinates transformation
centroidalDynamics    = centroidalConversion(DYNAMICS,FORKINEMATICS,STATE);
controlParam          = jointSpaceController(CONFIG,gain,trajectory,centroidalDynamics,STATE);

%% Feet pose correction
% this will avoid numerical errors during the forward dynamics integration
if     feet_on_ground(1) == 1 && feet_on_ground(2) == 0
    
    deltaPoseFeet    = deltaPoseLFoot;
    
elseif feet_on_ground(1) == 0 && feet_on_ground(2) == 1
    
    deltaPoseFeet    = deltaPoseRFoot;
    
elseif feet_on_ground(1) == 1 && feet_on_ground(2) == 1
    
    deltaPoseFeet    = [deltaPoseLFoot;deltaPoseRFoot];
end

%% Contact forces computation
controlParam.fc      = (JcMinv*transpose(Jc))\(JcMinv*h -JcMinvS*controlParam.tau -dJc_nu -KdFeet.*v_feet-KpFeet.*deltaPoseFeet);

end
