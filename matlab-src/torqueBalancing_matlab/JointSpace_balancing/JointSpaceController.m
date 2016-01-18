function [tau_js] = JointSpaceController (param, Nu, M, g, CNu, Jc, dJcNu, gains, jointReferences)
%% JointspaceController
%  Computes the desired control torques at joints using a simple joint space controller.
%  The output is the vector of desired torques at joints 
%  tau_js [ndofx1]

%% Parameters definition
%PINV_TOL        = 1e-8;
 reg             = 1e-7;
 
 ndof            = param.ndof;
 feet_on_ground  = param.feet_on_ground;
 qj              = param.qj;
 impedances      = gains.impedances;
 dampings        = gains.dampings;
 
 ddqjDes         = jointReferences.ddqjDes;
 dqjDes          = jointReferences.dqjDes;
 qjDes           = jointReferences.qjDes;
 
%Mb              = M(1:6,1:6);
 Mj              = M(7:end,7:end);

 S               = [ zeros(6,ndof);
                    eye(ndof,ndof)];

 dqj             = Nu(7:end);

%% Composed terms definition
qjTilde          = qj-qjDes;
dqjTilde         = dqj-dqjDes;

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

 tau_js     = TauMatrix_js\(CNu_js + g_js + Nu_js + Mj*ddqjDes -diag(impedances)*qjTilde -diag(dampings)*dqjTilde);

elseif  sum(feet_on_ground) == 2 

 pinvTauMatrix_js  = TauMatrix_js'/(TauMatrix_js*TauMatrix_js' + reg*eye(size(TauMatrix_js,1)));  
%pinvTauMatrix_js  = pinv(TauMatrix_js,PINV_TOL);

 tau_js      = pinvTauMatrix_js*(CNu_js + g_js + Nu_js + Mj*ddqjDes -diag(impedances)*qjTilde -diag(dampings)*dqjTilde);

end

end
