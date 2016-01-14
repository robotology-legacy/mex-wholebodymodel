function [dchi, visual_param] = forwardDynamics(t,chi,param )
% forwardDynamics 
% Forward dynamics of the wholeBodyModel
%
% This is the forward dynamics of the model loaded in the 
% wholeBodyInterface from the URDF description. The dynamic model is
% described as an explicit ordinary differential equation of the form:
%
%                dchi = forwardDynamics( t,chi)
%
% where chi is the variable to be integrated. For a floating base
% articulated chain, the variable chi contains the following
% subvariables:
%
% x_b:      the cartesian position of the base (R^3)
% qt_b:     the quaternion describing the orientation of the base (global parametrization of SO(3))
% qj:       the joint positions (R^ndof)
% dx_b:     the cartesian velocity of the base (R^3)
% omega_b:  the velocity describing the orientation of the base (so(3))
% dqj:      the joint velocities (R^ndof)

% disp(t);
waitbar(t/param.tEnd,param.wait)
 
%% extraction of state in the base frame of reference
ndof  = param.ndof;

x_b   = chi(1:3,:); 
qt_b  = chi(4:7,:);
qj    = chi(8:ndof+7,:);

dx_b    = chi(ndof+8:ndof+10,:);
omega_W = chi(ndof+11:ndof+13,:);
dqj     = chi(ndof+14:2*ndof+13,:);

v       = [dx_b; omega_W; dqj];

%% fixing the world reference frame
qT         = [x_b; qt_b];
[~,R_b]    = frame2posrot(qT);

%% parameters definition using wbm_functions
R_b_tr  = R_b.';

M       = wbm_massMatrix(R_b_tr,x_b,qj); 
h       = wbm_generalisedBiasForces(R_b_tr,x_b,qj,dqj,[dx_b;omega_W]);
g       = wbm_generalisedBiasForces(R_b_tr,x_b,qj,zeros(25,1),zeros(6,1));

%% building up constraints jacobian and djdq
Jc    = zeros(6*param.numConstraints,6+ndof);
dJcdv = zeros(6*param.numConstraints,1);

for i=1:param.numConstraints
    
    Jc(6*(i-1)+1:6*i,:)    = wbm_jacobian(R_b_tr,x_b,qj,param.constraintLinkNames{i});
    dJcdv(6*(i-1)+1:6*i,:) = wbm_djdq(R_b_tr,x_b,qj,dqj,[dx_b;omega_W],param.constraintLinkNames{i});
    
end

% jacobian at CoM
Jcom   = wbm_jacobian(R_b_tr,x_b,qj,'com');

%% centroidal coordinates transformation
% CoM linear position and velocity
com     = wbm_forwardKinematics(R_b_tr,x_b,qj,'com');
xcom    = com(1:3);
dcom    = Jcom*v;
dxcom   = dcom(1:3);

% transformation matrix for centroidal
[T, dT] = centroidalTransformationT_TDot(xcom,x_b,dxcom,dx_b,M);

% conversion of parameters to the new frame of reference
[Mc,CNu_c, gc, Jc_c, dJcdv_c, vc] = fromFloatingToCentroidalDynamics(M, h, g, Jc, dJcdv, v, T, dT);

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
controlParam.M       = Mc;
controlParam.g       = gc;
controlParam.CNu     = CNu_c;
controlParam.v       = vc;
controlParam.Jc      = Jc_c;
controlParam.dJcdv   = dJcdv_c;

lsole                = wbm_forwardKinematics(R_b_tr,x_b,qj,'l_sole');
rsole                = wbm_forwardKinematics(R_b_tr,x_b,qj,'r_sole');
controlParam.com     = com;

% adding a correction term in the costraints equation.
% this is necessary to reduce the numerical errors in the costraints
% equation.
[x_lf,R_b_lf]    = frame2posrot(lsole);
[T_lf,phi_lf]    = parametrization(R_b_lf);
[x_rf,R_b_rf]    = frame2posrot(rsole);
[T_rf,phi_rf]    = parametrization(R_b_rf);

pos_leftFoot     = [x_lf; phi_lf.'];
pos_rightFoot    = [x_rf; phi_rf.'];

lfoot_ini        = param.lfoot_ini;
rfoot_ini        = param.rfoot_ini;

[xi_lf,R_bi_lf]    = frame2posrot(lfoot_ini);
[~,phii_lf]        = parametrization(R_bi_lf);
[xi_rf,R_bi_rf]    = frame2posrot(rfoot_ini);
[~,phii_rf]        = parametrization(R_bi_rf);

lfoot_ini_t     = [xi_lf; phii_lf.'];
rfoot_ini_t     = [xi_rf; phii_rf.'];

k_corr_pos = 5;
k_corr_vel = 2*sqrt(k_corr_pos);
  
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

%% control torque calculated using the balancing controller
[tau, cVisualParam] = balController(t,param,controlParam);

%% calculation of the real contact forces
Jct            = Jc.';
St             = [zeros(6,ndof); eye(ndof)];
M_inv          = eye(ndof+6)/M;
JcMinvSt       = Jc*M_inv*St;
JcMinv         = Jc*M_inv;
inv_JcMinvJct  = eye(6*param.feet_on_ground)/(Jc*M_inv*Jct);

f_c            = inv_JcMinvJct*(JcMinv*h -JcMinvSt*tau -dJcdv -k_corr_vel.*Jc*v -k_corr_pos.*pos_feet_delta);

% CoP at feet
CoP(1) = -f_c(5)/f_c(3);
CoP(2) =  f_c(4)/f_c(3);

if  param.feet_on_ground == 2
CoP(3) = -f_c(11)/f_c(9);
CoP(4) =  f_c(10)/f_c(9);

end

CoP=CoP.';

%% dchi computation
% need to apply root-to-world rotation to the spatial angular velocity omega_W to
% obtain angular velocity in body frame omega_b. This is then used in the
% quaternion derivative computation.
omega_b = R_b_tr*omega_W;                               
dqt_b   = quaternionDerivative(omega_b, qt_b);       

dx      = [dx_b;dqt_b;dqj];

dv      = M\(Jc'*f_c + [zeros(6,1); tau]-h);

dchi    = [dx;dv];  

%% visualization 
% these are the variables that can be plotted by the "visualizer graphics"
% function
  visual_param.pos_feet  = [pos_leftFoot; pos_rightFoot];
  visual_param.fc        = f_c;
  
  visual_param.qj        = qj;
  visual_param.dqj       = dqj;
  visual_param.ddqj      = dv(7:end);
  
  visual_param.e_com     = cVisualParam.e_com;
  visual_param.CoP       = CoP;
  visual_param.norm_t    = norm(tau);
  
  visual_param.q         = cVisualParam.q_inv;
  visual_param.dq        = cVisualParam.dq_inv;
  visual_param.ddq       = cVisualParam.ddq_inv;

end

