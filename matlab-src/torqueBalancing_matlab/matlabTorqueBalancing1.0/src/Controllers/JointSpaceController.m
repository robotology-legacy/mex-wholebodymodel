function  [tau,f0,ddqjNonLin] = JointSpaceController(params, dynamics, gains, JointReferences)
%JOINTSPACECONTROLLER
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% general parameters
%reg             = params.pinv_tol;
damp             = params.pinv_damp; 
ndof             = params.ndof;
feet_on_ground   = params.feet_on_ground;
qj               = dynamics.qj;
impedances       = gains.impedances;
dampings         = gains.dampings;
ddqjRef          = JointReferences.ddqjRef;
dqjRef           = JointReferences.dqjRef;
qjRef            = JointReferences.qjRef;
M                = dynamics.M;
Jc               = dynamics.Jc;
%Mb              = M(1:6,1:6);
Mj               = M(7:end,7:end);
S                = [zeros(6,ndof);
                    eye(ndof,ndof)];
dqj              = dynamics.Nu(7:end);
g                = dynamics.g;
CNu              = dynamics.CNu;
dJcNu            = dynamics.dJcNu;

%% Composed terms definition
qjTilde          = qj-qjRef;
dqjTilde         = dqj-dqjRef;
Jct              = transpose(Jc);
JcMinv           = Jc/M;
JcMinvJct        = JcMinv*transpose(Jc);
Lambda           = (JcMinvJct\JcMinv);

% Rewrite the equations of motion substituting the contact forces (calculated 
% with respect of joint torques from contact constraints equation)  
g_tilde          = -Jct*Lambda*g;
CNu_tilde        = -Jct*Lambda*CNu;
Nu_tilde         =  Jct*(JcMinvJct\dJcNu);
TauMatrix        = (eye(ndof+6) - Jct*Lambda)*S;

% Consider only the joint space part of the equation of motion
g_js             = g_tilde(7:end);
CNu_js           = CNu_tilde(7:end);
Nu_js            = Nu_tilde(7:end);
TauMatrix_js     = TauMatrix(7:end,:);

%% Joint space controller
if      sum(feet_on_ground) == 1

 tau               = TauMatrix_js\(CNu_js + g_js + Nu_js + Mj*ddqjRef -impedances*qjTilde -dampings*dqjTilde);

elseif  sum(feet_on_ground) == 2 

 pinvTauMatrix_js  = TauMatrix_js'/(TauMatrix_js*TauMatrix_js' + damp*eye(size(TauMatrix_js,1)));  
%pinvTauMatrix_js  = pinv(TauMatrix_js,PINV_TOL);

 tau               = pinvTauMatrix_js*(CNu_js + g_js + Nu_js + Mj*ddqjRef -impedances*qjTilde -dampings*dqjTilde);
end

f0          = zeros(6*params.numConstraints,1);
ddqjNonLin  = zeros(ndof,1);

end
