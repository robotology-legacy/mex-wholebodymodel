
function [ dchi, visual_param ] = forwardDynamics( t,chi,param )

%FORWARD DYNAMICS Forward dynamics of the wholeBodyModel
%
%   This is the forward dynamics of the model loaded in the 
%   wholeBodyInterface from the URDF description. The dynamic model is
%   described as an explicit ordinary differential equation of the form:
%
%                dchi = forwardDynamics( t,chi)
%
%   where chi is the variable to be integrated. For a floating base
%   articulated chain, the variable chi contains the following
%   subvariables:
%
%   x_b:      the cartesian position of the base (R^3)
%   qt_b:     the quaternion describing the orientation of the base (global parametrization of SO(3))
%   qj:       the joint positions (R^ndof)
%   dx_b:     the cartesian velocity of the base (R^3)
%   omega_b:  the velocity describing the orientation of the base (so(3))
%   dqj:      the joint velocities (R^ndof)

% disp(t);
 
%% extraction of state
ndof = param.ndof;

x_b  = chi(1:3,:); 
qt_b = chi(4:7,:);
qj   = chi(8:ndof+7,:);

%x = [x_b;qt_b;qj];

dx_b    = chi(ndof+8:ndof+10,:);
omega_W = chi(ndof+11:ndof+13,:);
dqj     = chi(ndof+14:2*ndof+13,:);

v = [dx_b;omega_W;dqj];

%% fixing the world reference frame
qt_b_mod_s = qt_b(1);
qt_b_mod_r = qt_b(2:end);
R_b        = eye(3) - 2*qt_b_mod_s*skew(qt_b_mod_r) + 2 * skew(qt_b_mod_r)^2;

wbm_setWorldFrame(R_b,x_b,[0 0 -9.81]');

%% MexWholeBodyModel calls
wbm_updateState(qj,dqj,[dx_b;omega_W]);

%reconstructing rotation of root to world from the quaternion
%[~,T_b,~,~] = wholeBodyModel('get-state');

%righting quaternion ordering to [real;imaginary]^T since get-state
%returns the opposite ordering
%qt_b_mod   = [T_b(7);T_b(4:6)];
%qt_b_mod_s = T_b(7);
%qt_b_mod_r = T_b(4:6);

%wbm_updateState(qj,dqj,[dx_b;omega_W]);

M = wbm_massMatrix(); 
h = wbm_generalisedBiasForces();
H = wbm_centroidalMomentum();

%M     = wbm_massMatrix(qj);
%hDash = wbm_generalisedBiasForces(qj,dqj,[dx_b;omega_W]);
%g     = wbm_generalisedBiasForces(qj,zeros(size(qj)),zeros(6,1));
%g     = zeros(size(h));
%h     = zeros(size(h));


%% Building up contraints jacobian and djdq
%numConstraints = length(param.constraintLinkNames);

Jc = zeros(6*param.numConstraints,6+ndof);
dJcDq = zeros(6*param.numConstraints,1);

for i=1:param.numConstraints
    Jc(6*(i-1)+1:6*i,:)    = wbm_jacobian(param.constraintLinkNames{i});
    dJcDq(6*(i-1)+1:6*i,:) = wbm_djdq(param.constraintLinkNames{i});
end

%% saturation check
%limits = wbm_jointLimits();
 limits = param.limits;
 l_min  = limits(:,1);
 l_max  = limits(:,2);
 tol    = 0.01;

res = qj < l_min + tol | qj > l_max - tol;
res = sum(res);

if res==0

else
 
 disp('joint limits reached at time')    
 disp(t)
%error('joint limits reached '); 

end

%% parameters for controller
controlParam.qj      = qj;
controlParam.M       = M;
controlParam.h       = h;
controlParam.H       = H;
controlParam.v       = v;
controlParam.Jc      = Jc;
controlParam.dJcDq   = dJcDq;

% if the simulation takes too much time, reduce the dimension of chi and set
% the variable controlParam.pos_feet to zero
 controlParam.pos_feet = chi(64:75);
%controlParam.pos_feet = zeros(12,1);

controlParam.x_lsole = wbm_forwardKinematics('l_sole');
controlParam.x_rsole = wbm_forwardKinematics('r_sole');
controlParam.com     = wbm_forwardKinematics('com');
controlParam.Jcom    = wbm_jacobian('com');

%% control torque
% tau = param.tau(t);
 [tau,fc, cVisualParam] = balController(t,param,controlParam);
  
%% Contact forces computation
% JcMinv    = Jc/M;
% JcMinvJct = JcMinv * Jc';  
%   
% % temp    = JcMinv*h;
% % temp2   = JcMinvJct\(JcMinv*h);
% 
%  fc        = (JcMinvJct)\(JcMinv*(h-[zeros(6,1);tau])-dJcDq);    

% need to apply root-to-world rotation to the spatial angular velocity omega_W to
% obtain angular velocity in body frame omega_b. This is then used in the
% quaternion derivative computation.

omega_b = R_b*omega_W;                               % R_b*omega_W;
dqt_b   = quaternionDerivative(omega_b, qt_b);       %,param.QuaternionDerivativeParam);

dx   = [dx_b;dqt_b;dqj];
dv   = M\(Jc'*fc + [zeros(6,1); tau]-h);

% the vector of variables to integrate is modified to add the velocity of
% the feet to correct numerical errors
%dchi = [dx;dv];  
 dchi = [dx;dv; Jc*v];

%kinEnergy = 0.5*v'*M*v;
%dchi = zeros(size(dchi));

%% visualization 
% these are the variables that can be plotted by the "visualizer" script
  visual_param.pos_feet  = controlParam.pos_feet;
  visual_param.fc        = fc;
  visual_param.tau       = tau;
  visual_param.error_com = cVisualParam.e_com;
  visual_param.f0        = cVisualParam.f0;

end

