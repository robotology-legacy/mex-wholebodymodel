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

%   optionally, it contains also the feet position to correct numerical
%   errors ([12x1] or [6x1] depending if the robot is on two feet or one
%   foot)

%   disp(t);

waitbar(t/param.tEnd,param.wait)

%% extraction of state
ndof = param.ndof;

x_b  = chi(1:3,:); 
qt_b = chi(4:7,:);
qj   = chi(8:ndof+7,:);

%x   = [x_b;qt_b;qj];

dx_b    = chi(ndof+8:ndof+10,:);
omega_W = chi(ndof+11:ndof+13,:);
dqj     = chi(ndof+14:2*ndof+13,:);

v       = [dx_b; omega_W; dqj];

%% fixing the world reference frame
qT         = [x_b; qt_b];
[~,R_b]    = frame2posrot(qT);
 
wbm_setWorldFrame(R_b,x_b,[0 0 -9.81]');

wbm_updateState(qj,dqj,[dx_b;omega_W]);

%% MexWholeBodyModel calls
% warning: there's an issue with the optimized mode: for now, it won't be
% used
% M = wbm_massMatrix(); 
% h = wbm_generalisedBiasForces();
% H = wbm_centroidalMomentum();

R_binv = eye(3)/R_b;

M      = wbm_massMatrix(R_binv,x_b,qj); 
h      = wbm_generalisedBiasForces(R_binv,x_b,qj,dqj,[dx_b;omega_W]);
H      = wbm_centroidalMomentum(R_binv,x_b,qj,dqj,[dx_b;omega_W]);

gen      = wbm_generalisedBiasForces(R_binv,x_b,qj,zeros(25,1),zeros(6,1));

%% Building up contraints jacobian and djdq
Jc    = zeros(6*param.numConstraints,6+ndof);
dJcDq = zeros(6*param.numConstraints,1);

for i=1:param.numConstraints
    
    Jc(6*(i-1)+1:6*i,:)    = wbm_jacobian(R_binv,x_b,qj,param.constraintLinkNames{i});
    dJcDq(6*(i-1)+1:6*i,:) = wbm_djdq(R_binv,x_b,qj,dqj,[dx_b;omega_W],param.constraintLinkNames{i});
    
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
controlParam.gen     = gen;

if param.feet_on_ground == 1
    
controlParam.pos_feet = chi(64:69,:);
controlParam.qDes2    = chi(70:end,:);

elseif param.feet_on_ground == 2

controlParam.pos_feet = chi(64:75,:);
controlParam.qDes2    = chi(76:end,:);

end

controlParam.lsole   = wbm_forwardKinematics(R_binv,x_b,qj,'l_sole');
controlParam.rsole   = wbm_forwardKinematics(R_binv,x_b,qj,'r_sole');
controlParam.com     = wbm_forwardKinematics(R_binv,x_b,qj,'com');
controlParam.Jcom    = wbm_jacobian(R_binv,x_b,qj,'com');

%% control torque and contact forces calculated using the balancing controller
[tau, fc, cVisualParam] = balController(t,param,controlParam);
  
%% Contact forces computation
% need to apply root-to-world rotation to the spatial angular velocity omega_W to
% obtain angular velocity in body frame omega_b. This is then used in the
% quaternion derivative computation.

omega_b = R_binv*omega_W;                               
dqt_b   = quaternionDerivative(omega_b, qt_b);       

dx      = [dx_b;dqt_b;dqj];

dv      = M\(Jc'*fc + [zeros(6,1); tau]-h);

% the vector of variables to be integrated is redefined to add the feet
% position and orientation
% dchi   = [dx;dv];  
  dchi   = [dx;dv;Jc*v; cVisualParam.dqj_des];

%kinEnergy = 0.5*v'*M*v;
%dchi      = zeros(size(dchi));

%% visualization 
% these are the variables that can be plotted by the "visualizer graphics"
% function
  visual_param.pos_feet  = [controlParam.lsole ; controlParam.rsole];
  visual_param.fc        = fc;
  visual_param.tau       = tau;
  visual_param.qj        = qj;
  visual_param.error_com = cVisualParam.e_com;
  visual_param.f0        = cVisualParam.f0;

end

