function CONTROLLER = runController(GAINS,TRAJECTORY,DYNAMICS,FORKINEMATICS,INIT_CONDITIONS,STATE,MODEL)
%RUNCONTROLLER initializes balancing controllers, QP solver and centroidal
%              transformation.
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

%% Robot configuration
qj                     = STATE.qj;
dqj                    = STATE.dqj;

%% Robot dynamics
dJc_nu                 = DYNAMICS.dJc_nu;
M                      = DYNAMICS.M;
Jc                     = DYNAMICS.Jc;
h                      = DYNAMICS.h;
JcMinv                 = Jc/M;
JcMinvS                = JcMinv*S;

%% Robot forward kinematics
v_feet                 = FORKINEMATICS.v_feet;
TL                     = FORKINEMATICS.TL;
TR                     = FORKINEMATICS.TR;
poseRFoot_ang          = FORKINEMATICS.poseRFoot_ang;
poseLFoot_ang          = FORKINEMATICS.poseLFoot_ang;
% correction terms for avoiding numerical instability
deltaPoseRFoot         = TR*(poseRFoot_ang-INITFORKINEMATICS.poseRFoot_ang);
deltaPoseLFoot         = TL*(poseLFoot_ang-INITFORKINEMATICS.poseLFoot_ang);

%% %%%%%%%%%%%%%%%%%%% BALANCING CONTROLLERS %%%%%%%%%%%%%%%%%%%%%%%% %%
% apply centroidal transformation to the control model. 
if MODEL.CONFIG.use_centroidalTransf  
    [STATE,DYNAMICS]   = centroidalConversion(DYNAMICS,FORKINEMATICS,STATE);
end
% balancing controllers
if strcmp(MODEL.CONFIG.control_type,'stackOfTask') 
    CONTROLLER         = stackOfTaskController(MODEL,GAINS,TRAJECTORY,DYNAMICS,FORKINEMATICS,STATE);
elseif strcmp(MODEL.CONFIG.control_type,'jointControl') 
    CONTROLLER         = jointSpaceController(MODEL,GAINS,TRAJECTORY,DYNAMICS,STATE);
end
% quadratic programming solver for the nullspace of contact forces. It will
% also apply the unilateral constraints at feet and it ensures the contact
% forces to be inside the friction cones.
if  use_QPsolver                
    CONTROLLER.fcDes   = QPSolver(CONTROLLER,MODEL,FORKINEMATICS);
end

%% Application of backstepping algorithm for computing motor torques
if MODEL.CONFIG.use_SEA
    [CONTROLLER.tau_xi,CONTROLLER.dxi_ref] = motorController(DYNAMICS,STATE,GAINS,CONTROLLER);
else
    % torque control assuming rigid joints
    CONTROLLER.tau_xi  = CONTROLLER.tauModel + CONTROLLER.Sigma*CONTROLLER.fcDes;
end

%% Add torque saturation
CONTROLLER.tau_xi      = min(MODEL.CONFIG.satTorque, max(-MODEL.CONFIG.satTorque, CONTROLLER.tau_xi));

%% Feet pose correction
% this will avoid numerical errors during the forward dynamics integration
if feet_on_ground(1) == 1 && feet_on_ground(2) == 0    
    % left foot balancing
    deltaPoseFeet      = deltaPoseLFoot;    
elseif feet_on_ground(1) == 0 && feet_on_ground(2) == 1    
    % right foot balancing
    deltaPoseFeet      = deltaPoseRFoot;    
elseif feet_on_ground(1) == 1 && feet_on_ground(2) == 1
    % two feet balancing
    deltaPoseFeet      = [deltaPoseLFoot;deltaPoseRFoot];
end

%% Contact forces computation
if MODEL.CONFIG.use_SEA
    % contact forces in case motors dynamics is considered
    xi                 = STATE.xi;
    dxi                = STATE.dxi;
    KS                 = DYNAMICS.KS;
    KD                 = DYNAMICS.KD;
    CONTROLLER.fc      = (JcMinv*transpose(Jc))\(JcMinv*h -JcMinvS*(KS*(xi-qj) +KD*(dxi-dqj)) -dJc_nu -KdFeet.*v_feet -KpFeet.*deltaPoseFeet);
else
    CONTROLLER.fc      = (JcMinv*transpose(Jc))\(JcMinv*h -JcMinvS*controlParam.tau -dJc_nu -KdFeet.*v_feet -KpFeet.*deltaPoseFeet);
end
end
