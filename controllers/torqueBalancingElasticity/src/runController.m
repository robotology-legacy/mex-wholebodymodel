function CONTROLLER = runController(GAINS,TRAJECTORY,DYNAMICS,FORKINEMATICS,INIT_CONDITIONS,STATE,MODEL)
%RUNCONTROLLER initializes balancing controllers. Currently, default 
%              controller uses "stack of tasks" approach (see documentation).
%
% Format: CONTROLLER = RUNCONTROLLER(GAINS,TRAJECTORY,DYNAMICS,FORKINEMATICS,INIT_CONDITIONS,STATE,MODEL)
%
% Inputs:  - GAINS is a structure containing all control gains;
%          - TRAJECTORY stores the CoM and joints reference trajectories;
%          - DYNAMICS contains current robot dynamics;
%          - FORKINEMATICS stores position, orientation and velocity of
%            points in cartesian space;
%          - STATE contains the current system state;
%          - INIT_CONDITIONS is a structure containing initial conditions
%            for integration.
%          - MODEL is a structure defining the robot model;
%
% Output:  - CONTROLLER is a structure containing the control torques, contact 
%            forces (and desired contact forces) and other parameters for
%            controlling the robot.
%
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% feet correction gains (to avoid numerical instability during constraint
% integration)
KpFeet                 = GAINS.KpFeet;
KdFeet                 = 2*sqrt(KpFeet);

%% Configuration parameters
ndof                   = MODEL.ndof;
use_QPsolver           = MODEL.CONFIG.use_QPsolver;
feet_on_ground         = MODEL.CONFIG.feet_on_ground;
INITFORKINEMATICS      = INIT_CONDITIONS.INITFORKINEMATICS;
% joints selector matrix
S                      = [zeros(6,ndof);
                          eye(ndof,ndof)];

%% Robot and motors configurations
xi                     = STATE.xi;
dxi                    = STATE.dxi;
qj                     = STATE.qj;
dqj                    = STATE.dqj;

%% Robot and elastic joints dynamics
dJc_nu                 = DYNAMICS.dJc_nu;
M                      = DYNAMICS.M;
Jc                     = DYNAMICS.Jc;
h                      = DYNAMICS.h;
JcMinv                 = Jc/M;
JcMinvS                = JcMinv*S;
KS                     = DYNAMICS.KS;
KD                     = DYNAMICS.KD;

%% Robot forward kinematics
v_feet                 = FORKINEMATICS.v_feet;
TL                     = FORKINEMATICS.TL;
TR                     = FORKINEMATICS.TR;
poseRFoot_ang          = FORKINEMATICS.poseRFoot_ang;
poseLFoot_ang          = FORKINEMATICS.poseLFoot_ang;
% correction terms for avoiding numerical instability
deltaPoseRFoot         = TR*(poseRFoot_ang-INITFORKINEMATICS.poseRFoot_ang);
deltaPoseLFoot         = TL*(poseLFoot_ang-INITFORKINEMATICS.poseLFoot_ang);

%% %%%%%%%%%%%%%%%%%%% STACK OF TASK CONTROLLER %%%%%%%%%%%%%%%%%%%%%%%% %%
CONTROLLER             = stackOfTaskController(MODEL,GAINS,TRAJECTORY,DYNAMICS,FORKINEMATICS,STATE);

% quadratic programming solver for the nullspace of contact forces. It will
% also apply the unilateral constraints at feet and it ensures the contact
% forces to be inside the friction cones.
if  use_QPsolver == 1                 
    CONTROLLER.fcDes = QPSolver(CONTROLLER,MODEL,FORKINEMATICS);
end

%% Application of backstepping algorithm for computing motor torques
[CONTROLLER.tau_xi,CONTROLLER.dxi_ref] = motorController(DYNAMICS,STATE,GAINS,CONTROLLER);

% if this option is enabled, the controller uses a stiff control model
% intead of the elastic control model. Be careful: the closed loop system
% may be unstable!
if MODEL.CONFIG.assume_rigid_joints == 1
    CONTROLLER.tau_xi  = CONTROLLER.tauModel + CONTROLLER.Sigma*CONTROLLER.fcDes;
end

%% Add torque saturation
CONTROLLER.tau_xi      = min(MODEL.CONFIG.satTorque, max(-MODEL.CONFIG.satTorque, CONTROLLER.tau_xi));

%% Feet pose correction
% this will avoid numerical errors during the forward dynamics integration
if feet_on_ground(1) == 1 && feet_on_ground(2) == 0    
    % left foot balancing
    deltaPoseFeet    = deltaPoseLFoot;    
elseif feet_on_ground(1) == 0 && feet_on_ground(2) == 1    
    % right foot balancing
    deltaPoseFeet    = deltaPoseRFoot;    
elseif feet_on_ground(1) == 1 && feet_on_ground(2) == 1
    % two feet balancing
    deltaPoseFeet    = [deltaPoseLFoot;deltaPoseRFoot];
end

%% Contact forces computation
CONTROLLER.fc        = (JcMinv*transpose(Jc))\(JcMinv*h -JcMinvS*(KS*(xi-qj) +KD*(dxi-dqj)) -dJc_nu -KdFeet.*v_feet -KpFeet.*deltaPoseFeet);

end
