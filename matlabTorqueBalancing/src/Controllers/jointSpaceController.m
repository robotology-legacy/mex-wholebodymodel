function  controlParam = jointSpaceController(CONFIG,gains,trajectory,DYNAMICS,STATE)
%JOINTSPACECONTROLLER implemements a joint space balancing controller for
%                     the robot iCub.
%
%   JOINTSPACECONTROLLER computes the control torques at joints reducing the 
%   system to the joint space. It compensates for the gravity terms and 
%   defines the desired control torques at joints oin order to follow a joint
%   reference trajectory.
%
%   controlParam = JOINTSPACECONTROLLER(config, gains, trajectory, dynamics,
%   state) takes as input the structure CONFIG, which contains all the 
%   configuration parameters, the desired control gains, the reference
%   trajectory, the robot dynamics and state.
%   The output is the structure CONTROLPARAM which contains the desired
%   control torques and other parameters for visualization.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
%% Config parameters
pinv_damp        = CONFIG.pinv_damp;
ndof             = CONFIG.ndof;
feet_on_ground   = CONFIG.feet_on_ground;
%pinv_tol        = CONFIG.pinv_tol;

%% Dynamics
M                = DYNAMICS.M;
Jc               = DYNAMICS.Jc;
dqj              = DYNAMICS.Nu(7:end);
g                = DYNAMICS.g;
CNu              = DYNAMICS.CNu;
dJcNu            = DYNAMICS.dJcNu;
Mj               = M(7:end,7:end);

%% Gains and references
impedances       = gains.impedances;
dampings         = gains.dampings;
ddqjRef          = trajectory.JointReferences.ddqjRef;
dqjRef           = trajectory.JointReferences.dqjRef;
qjRef            = trajectory.JointReferences.qjRef;

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
gTilde           = -Jct*Lambda*g;
CNuTilde         = -Jct*Lambda*CNu;
dJcNuTilde       =  Jct*(JcMinvJct\dJcNu);
TauMatrix        = (eye(ndof+6) - Jct*Lambda);

% the dynamics is projected into the joint space
gTilde           = gTilde(7:end);
CNuTilde         = CNuTilde(7:end);
dJcNuTilde       = dJcNuTilde(7:end);
TauMatrix        = TauMatrix(7:end,7:end);

%% Joint space controller
if      sum(feet_on_ground) == 1

 tau             = TauMatrix\(CNuTilde + gTilde + dJcNuTilde + Mj*ddqjRef -impedances*qjTilde -dampings*dqjTilde);

elseif  sum(feet_on_ground) == 2 

 pinvTauMatrix   = TauMatrix'/(TauMatrix*TauMatrix' + pinv_damp*eye(size(TauMatrix,1))); 
%pinvTauMatrix   = pinv(TauMatrix,pinv_tol);
 tau             = pinvTauMatrix*(CNuTilde + gTilde + dJcNuTilde + Mj*ddqjRef -impedances*qjTilde -dampings*dqjTilde);
end

controlParam.f0  = zeros(6*CONFIG.numConstraints,1);
controlParam.tau = tau;

end
