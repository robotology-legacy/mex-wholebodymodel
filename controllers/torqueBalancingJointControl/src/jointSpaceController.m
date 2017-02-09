function  controlParam = jointSpaceController(CONFIG,gains,trajectory,DYNAMICS,STATE)
%JOINTSPACECONTROLLER implemements a joint space balancing controller for
%                     floating base robots.
%
% JOINTSPACECONTROLLER computes joint control torques directly from joint
% space equations of motion, after considering the contact constraints in
% the model.
%
% controlParam = JOINTSPACECONTROLLER(CONFIG, gains, trajectory,DYNAMICS,
% STATE) the robot configuration parameters, control gains, the joint 
% reference trajectory, and robot dynamics and state.
% The output controlParam contains the desired control torques 
% and other parameters for visualization.
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
dqj              = DYNAMICS.nu(7:end);
g                = DYNAMICS.g;
C_nu             = DYNAMICS.C_nu;
dJc_nu           = DYNAMICS.dJc_nu;
Mj               = M(7:end,7:end);

%% Gains and references
impedances       = gains.impedances;
dampings         = gains.dampings;
ddqjRef          = trajectory.jointReferences.ddqjRef;
dqjRef           = trajectory.jointReferences.dqjRef;
qjRef            = trajectory.jointReferences.qjRef;

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
CNuTilde         = -Jct*Lambda*C_nu;
dJcNuTilde       =  Jct*(JcMinvJct\dJc_nu);
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
    %pinvTauMatrix  = pinv(TauMatrix,pinv_tol);
    tau             = pinvTauMatrix*(CNuTilde + gTilde + dJcNuTilde + Mj*ddqjRef -impedances*qjTilde -dampings*dqjTilde);
end

controlParam.tau = tau;

end
