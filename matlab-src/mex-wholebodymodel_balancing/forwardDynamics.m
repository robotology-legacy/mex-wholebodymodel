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

%% mexWholeBodyModel calls
% warning: there's an issue with the wbm optimized mode: for now, it won't be
% used
M      = wbm_massMatrix(R_b,x_b,qj); 
h      = wbm_generalisedBiasForces(R_b,x_b,qj,dqj,[dx_b;omega_W]);
H      = wbm_centroidalMomentum(R_b,x_b,qj,dqj,[dx_b;omega_W]);

%% building up contraints jacobian and djdq
Jc    = zeros(6*param.numConstraints,6+ndof);
dJcDq = zeros(6*param.numConstraints,1);

for i=1:param.numConstraints
    
    Jc(6*(i-1)+1:6*i,:)    = wbm_jacobian(R_b,x_b,qj,param.constraintLinkNames{i});
    dJcDq(6*(i-1)+1:6*i,:) = wbm_djdq(R_b,x_b,qj,dqj,[dx_b;omega_W],param.constraintLinkNames{i});
    
end

%% saturation check
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
 error('joint limits reached '); 

end

%% parameters for controller
controlParam.qj      = qj;
controlParam.M       = M;
controlParam.h       = h;
controlParam.H       = H;
controlParam.v       = v;
controlParam.Jc      = Jc;
controlParam.dJcDq   = dJcDq;

controlParam.lsole   = wbm_forwardKinematics(R_b,x_b,qj,'l_sole');
controlParam.rsole   = wbm_forwardKinematics(R_b,x_b,qj,'r_sole');
controlParam.com     = wbm_forwardKinematics(R_b,x_b,qj,'com');
controlParam.Jcom    = wbm_jacobian(R_b,x_b,qj,'com');

% adding a correction term in the costraints equation.
% this is necessary to reduce the numerical integration errors 
% current feet position and orientation
[x_lf,R_b_lf]    = frame2posrot(controlParam.lsole);
[T_lf,phi_lf]    = parametrization(R_b_lf);
[x_rf,R_b_rf]    = frame2posrot(controlParam.rsole);
[T_rf,phi_rf]    = parametrization(R_b_rf);

pos_leftFoot     = [x_lf; phi_lf.'];
pos_rightFoot    = [x_rf; phi_rf.'];

% initial feet position and orientation
[xi_lf,R_bi_lf]   = frame2posrot(param.lfoot_ini);
[~,phii_lf]       = parametrization(R_bi_lf);
[xi_rf,R_bi_rf]   = frame2posrot(param.rfoot_ini);
[~,phii_rf]       = parametrization(R_bi_rf);

lfoot_ini_t       = [xi_lf; phii_lf.'];
rfoot_ini_t       = [xi_rf; phii_rf.'];

% feet position and orientation error 
 if      param.feet_on_ground == 1
 
  T_tildelf      = [eye(3)  zeros(3);
                    zeros(3)  T_lf];
               
  pos_feet_delta = T_tildelf*(pos_leftFoot-lfoot_ini_t);
 
 elseif  param.feet_on_ground == 2
     
  T_tildelf      = [eye(3)  zeros(3);
                    zeros(3)  T_lf];
               
  T_tilderf      = [eye(3)  zeros(3);
                    zeros(3)  T_rf];   
                
  T_tilde_tot    = [T_tildelf   zeros(6);
                    zeros(6)   T_tilderf];
               
  pos_feet_delta = T_tilde_tot*[(pos_leftFoot-lfoot_ini_t);...
                                (pos_rightFoot-rfoot_ini_t)];
                
 end

%% desired control torques calculated using the balancing controller
[tau, cVisualParam] = balController(t,param,controlParam);
  
%% contact forces computation
M_inv = eye(ndof+6)/M;
Jct   = Jc.';
St    =  [zeros(6,ndof);
          eye(ndof,ndof)];

JcMinvJct = Jc*M_inv*Jct;
JcMinv    = Jc*M_inv;
JcMinvSt  = Jc*M_inv*St;

K_corr_pos = 5;
K_corr_vel = 2*sqrt(K_corr_pos);

fc    = (eye(6*param.numConstraints)/JcMinvJct)*(JcMinv*h -JcMinvSt*tau -dJcDq  -K_corr_vel.*Jc*v -K_corr_pos.*pos_feet_delta);

%% state definition
% need to apply root-to-world rotation to the spatial angular velocity omega_W to
% obtain angular velocity in body frame omega_b. This is then used in the
% quaternion derivative computation.
omega_b = (R_b')*omega_W;                               
dqt_b   = quaternionDerivative(omega_b, qt_b);       

dx      = [dx_b;dqt_b;dqj];

dv      = M\(Jc'*fc + [zeros(6,1); tau]-h);

dchi    = [dx;dv];  

%% visualization 
% these are the variables that can be plotted by the "visualizer graphics"
% function
  visual_param.pos_feet  = [controlParam.lsole ; controlParam.rsole];
  visual_param.fc        = fc;
  visual_param.tau       = tau;
  visual_param.error_com = cVisualParam.e_com;
  visual_param.f0        = cVisualParam.f0;

end

