function controlParam  = runController(gain,trajectory,DYNAMICS,FORKINEMATICS,INIT_CONDITIONS,STATE,dxi,xi,ELASTICITY)
%RUNCONTROLLER  initializes balancing controllers. Default controller uses
%               "stack of tasks" approach.
%
% controlParam  = RUNCONTROLLER(gains,trajectory,DYNAMICS,FORKINEMATICS,
% INIT_CONDITIONS,STATE) takes as input control gains, joint reference trajectory, 
% and the robot dynamics, forward kinematics, state and systeminitial conditions.
% The output is structure controlparam which contains the control torques tau, 
% the contact forces fc, parameters for visualizing the results, etc.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
%% Feet correction gains
KpFeet                 = gain.corrPosFeet;
KdFeet                 = 2*sqrt(KpFeet);

%% Configuration parameters
ndof                   = INIT_CONDITIONS.CONFIG.ndof;
use_QPsolver           = INIT_CONDITIONS.CONFIG.use_QPsolver;
feet_on_ground         = INIT_CONDITIONS.CONFIG.feet_on_ground;
initForKinematics      = INIT_CONDITIONS.initForKinematics;
S                      = [zeros(6,ndof);
                         eye(ndof,ndof)];

%% Robot dynamics
dJc_nu                 = DYNAMICS.dJc_nu;
M                      = DYNAMICS.M;
Jc                     = DYNAMICS.Jc;
h                      = DYNAMICS.h;
JcMinv                 = Jc/M;
JcMinvS                = JcMinv*S;

%% Robot forward Kinematics
v_feet                 = FORKINEMATICS.v_feet;
TL                     = FORKINEMATICS.TL;
TR                     = FORKINEMATICS.TR;
poseRFoot_ang          = FORKINEMATICS.poseRFoot_ang;
poseLFoot_ang          = FORKINEMATICS.poseLFoot_ang;
deltaPoseRFoot         = TR*(poseRFoot_ang-initForKinematics.poseRFoot_ang);
deltaPoseLFoot         = TL*(poseLFoot_ang-initForKinematics.poseLFoot_ang);

%% BALANCING CONTROLLER: the current version uses a stack of task approach
controlParam           = stackOfTaskController(INIT_CONDITIONS.CONFIG,gain,trajectory,DYNAMICS,FORKINEMATICS,STATE,xi,ELASTICITY);

if  use_QPsolver == 1                 
    % quadratic programming solver for the nullspace of contact forces
    controlParam.fcDes = QPSolver(controlParam,INIT_CONDITIONS.CONFIG,FORKINEMATICS);
end

% backstepping on the motor dynamics: desired MOTOR torques
[controlParam.tau_xi,controlParam.dxi_ref] = motorController(dxi,xi,ELASTICITY,STATE,controlParam);

if INIT_CONDITIONS.CONFIG.assume_rigid_joints == 1
    
    controlParam.tau_xi  = controlParam.tauModel + controlParam.Sigma*controlParam.fcDes;
end

% torque saturation
controlParam.tau_xi      = min(INIT_CONDITIONS.CONFIG.satTorque, max(-INIT_CONDITIONS.CONFIG.satTorque, controlParam.tau_xi));

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
controlParam.fc      = (JcMinv*transpose(Jc))\(JcMinv*h -JcMinvS*(ELASTICITY.KS*(xi-STATE.qj)+ELASTICITY.KD*(dxi-STATE.dqj)) -dJc_nu -KdFeet.*v_feet-KpFeet.*deltaPoseFeet);

end
