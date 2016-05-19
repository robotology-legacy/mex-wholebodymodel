function  [tau,f0,ddqjNonLin] = JointSpaceController(params, dynamics, gains, JointReferences)
%JOINTSPACECONTROLLER is a simple balancing controller for the humanoid
%                     robot iCub.
%   JOINTSPACECONTROLLER computes the control torques at joints reducing the 
%   system to the joint space, compensating the gravity terms and trying to
%   impose a desired dynamics for the joint accelerations. 
%
%   [tau,f0,ddqjNonLin] = JOINTSPACECONTROLLER(params, dynamics, gains,
%   JointReferences) takes as input the structure params, which contains
%   all the utility parameters, and the structure dynamics, gains and
%   JointReferences which are used to compute the desired torques. The
%   outputs are the desired control torques tau [ndof x 1], while both f0
%   and ddqjNonLin are zero in this controller.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% general parameters
%pinv_tol        = params.pinv_tol;
pinv_damp        = params.pinv_damp; 
ndof             = params.ndof;
feet_on_ground   = params.feet_on_ground;
S                = [ zeros(6,ndof);
                    eye(ndof,ndof)];
% dynamics
qj               = dynamics.qj;
M                = dynamics.M;
Jc               = dynamics.Jc;
dqj              = dynamics.Nu(7:end);
g                = dynamics.g;
CNu              = dynamics.CNu;
dJcNu            = dynamics.dJcNu;
Mj               = M(7:end,7:end);
% gains and references
impedances       = gains.impedances;
dampings         = gains.dampings;
ddqjRef          = JointReferences.ddqjRef;
dqjRef           = JointReferences.dqjRef;
qjRef            = JointReferences.qjRef;

%% Composed terms definition
qjTilde          = qj-qjRef;
dqjTilde         = dqj-dqjRef;
Jct              = transpose(Jc);
JcMinv           = Jc/M;
JcMinvJct        = JcMinv*transpose(Jc);
Lambda           = (JcMinvJct\JcMinv);
gTilde           = -Jct*Lambda*g;
CNuTilde         = -Jct*Lambda*CNu;
dJcNuTilde       =  Jct*(JcMinvJct\dJcNu);
TauMatrix        = (eye(ndof+6) - Jct*Lambda)*S;

% Consider only the joint space part of the equation of motion
gJS              = gTilde(7:end);
CNuJS            = CNuTilde(7:end);
dJcNuJS          = dJcNuTilde(7:end);
TauMatrixJS      = TauMatrix(7:end,:);

%% Joint space controller
if      sum(feet_on_ground) == 1

 tau             = TauMatrixJS\(CNuJS + gJS + dJcNuJS + Mj*ddqjRef -impedances*qjTilde -dampings*dqjTilde);

elseif  sum(feet_on_ground) == 2 

 pinvTauMatrixJS = TauMatrixJS'/(TauMatrixJS*TauMatrixJS' + pinv_damp*eye(size(TauMatrixJS,1)));  
%pinvTauMatrixJS = pinv(TauMatrixJS,pinv_tol);
 tau             = pinvTauMatrixJS*(CNuJS + gJS + dJcNuJS + Mj*ddqjRef -impedances*qjTilde -dampings*dqjTilde);
end

f0         = zeros(6*params.numConstraints,1);
ddqjNonLin = zeros(ndof,1);

end
