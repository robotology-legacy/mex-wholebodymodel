function  controlParam = jointSpaceController(config, gains, trajectory, dynamics, state)
%JOINTSPACECONTROLLER is a simple balancing controller for the humanoid
%                     robot iCub.
%   JOINTSPACECONTROLLER computes the control torques at joints reducing the 
%   system to the joint space, compensating the gravity terms and trying to
%   impose a desired dynamics for the joint accelerations. 
%
%   controlParam = JOINTSPACECONTROLLER(config, gains, trajectory, dynamics,
%   state) takes as input the structure CONFIG, which contains all the 
%   utility parameters, and the structure GAINS, TRAJECTORY, DYNAMICS and 
%   STATE which are used to compute the desired torques.
%   The output is the structure CONTROLPARAM which contains the desired
%   control torques TAU and other parameters for visualization.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% Config parameters
pinv_damp        = config.pinv_damp; 
ndof             = config.ndof;
feet_on_ground   = config.feet_on_ground;
%pinv_tol        = config.pinv_tol;

% Dynamics
M                = dynamics.M;
Jc               = dynamics.Jc;
dqj              = dynamics.Nu(7:end);
g                = dynamics.g;
CNu              = dynamics.CNu;
dJcNu            = dynamics.dJcNu;
Mj               = M(7:end,7:end);

% State
qj               = state.qj;
S                = [zeros(6,ndof);
                    eye(ndof,ndof)];

% Gains and references
impedances       = gains.impedances;
dampings         = gains.dampings;
ddqjRef          = trajectory.JointReferences.ddqjRef;
dqjRef           = trajectory.JointReferences.dqjRef;
qjRef            = trajectory.JointReferences.qjRef;
qjTilde          = qj-qjRef;
dqjTilde         = dqj-dqjRef;

%% Reduce the state to the joint space
Jct              = transpose(Jc);
JcMinv           = Jc/M;
JcMinvJct        = JcMinv*transpose(Jc);
Lambda           = (JcMinvJct\JcMinv);
gTilde           = -Jct*Lambda*g;
CNuTilde         = -Jct*Lambda*CNu;
dJcNuTilde       =  Jct*(JcMinvJct\dJcNu);
TauMatrix        = (eye(ndof+6) - Jct*Lambda)*S;

gTilde           = gTilde(7:end);
CNuTilde         = CNuTilde(7:end);
dJcNuTilde       = dJcNuTilde(7:end);
TauMatrix        = TauMatrix(7:end,:);

%% Joint space controller
if      sum(feet_on_ground) == 1

 tau             = TauMatrix\(CNuTilde + gTilde + dJcNuTilde + Mj*ddqjRef -impedances*qjTilde -dampings*dqjTilde);

elseif  sum(feet_on_ground) == 2 

 pinvTauMatrixJS = TauMatrix'/(TauMatrix*TauMatrix' + pinv_damp*eye(size(TauMatrix,1)));  
%pinvTauMatrixJS = pinv(TauMatrixJS,pinv_tol);

 tau             = pinvTauMatrixJS*(CNuTilde + gTilde + dJcNuTilde + Mj*ddqjRef -impedances*qjTilde -dampings*dqjTilde);
end

controlParam.f0  = zeros(6*config.numConstraints,1);
controlParam.tau = tau;

end
