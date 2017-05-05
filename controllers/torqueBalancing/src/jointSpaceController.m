function  CONTROLLER = jointSpaceController(MODEL,GAINS,TRAJECTORY,DYNAMICS,STATE)
%JOINTSPACECONTROLLER implements a simple controller in the joint space of
%                     a floating base robot.
%
% Format: CONTROLLER = JOINTSPACECONTROLLER(MODEL,GAINS,TRAJECTORY,DYNAMICS,STATE)
%
% Inputs:  - MODEL is a structure defining the robot model;
%          - GAINS is a structure containing all control gains;
%          - TRAJECTORY stores the CoM and joints reference trajectories;
%          - DYNAMICS contains current robot dynamics;
%          - STATE contains the current system state;
%
% Output:  - CONTROLLER is a structure containing the control torques, 
%            and other parameters for controlling the robot.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% config parameters
pinv_damp        = MODEL.CONFIG.pinv_damp;
ndof             = MODEL.CONFIG.ndof;
feet_on_ground   = MODEL.CONFIG.feet_on_ground;

%% Dynamics
M                = DYNAMICS.M;
Jc               = DYNAMICS.Jc;
dqj              = DYNAMICS.nu(7:end);
g                = DYNAMICS.g;
C_nu             = DYNAMICS.C_nu;
dJc_nu           = DYNAMICS.dJc_nu;
Mj               = M(7:end,7:end);

%% Gains and references
impedances       = GAINS.impedances;
dampings         = GAINS.dampings;
ddqjRef          = TRAJECTORY.jointReferences.ddqjRef;
dqjRef           = TRAJECTORY.jointReferences.dqjRef;
qjRef            = TRAJECTORY.jointReferences.qjRef;

%% Robot state
qj               = STATE.qj;
qjTilde          = qj-qjRef;
dqjTilde         = dqj-dqjRef;

%% Project the equations in the joint space
Jct              = transpose(Jc);
JcMinv           = Jc/M;
JcMinvJct        = JcMinv*Jct;
Lambda           = (JcMinvJct\JcMinv);

% new dynamics parameters in the equations of motion after substituting the
% contact forces from the constraints equations.
gTilde            = -Jct*Lambda*g;
CNuTilde          = -Jct*Lambda*C_nu;
dJcNuTilde        =  Jct*(JcMinvJct\dJc_nu);
TauMatrix         = (eye(ndof+6) - Jct*Lambda);

% the dynamics is projected into the joint space
gTilde            = gTilde(7:end);
CNuTilde          = CNuTilde(7:end);
dJcNuTilde        = dJcNuTilde(7:end);
TauMatrix         = TauMatrix(7:end,7:end);

%% Joint space controller
if      sum(feet_on_ground) == 1   
    tau           = TauMatrix\(CNuTilde + gTilde + dJcNuTilde + Mj*ddqjRef -impedances*qjTilde -dampings*dqjTilde);    
elseif  sum(feet_on_ground) == 2   
    pinvTauMatrix = TauMatrix'/(TauMatrix*TauMatrix' + pinv_damp*eye(size(TauMatrix,1)));
    tau           = pinvTauMatrix*(CNuTilde + gTilde + dJcNuTilde + Mj*ddqjRef -impedances*qjTilde -dampings*dqjTilde);
end

CONTROLLER.tau    = tau;

end
