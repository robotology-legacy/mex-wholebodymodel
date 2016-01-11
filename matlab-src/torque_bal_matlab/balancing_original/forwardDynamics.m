function [dchi,visual_param] = forwardDynamics(t,chi,param)
%% forwardDynamics
%  This is the forward dynamics of the model loaded in the 
%  wholeBodyInterface from the URDF description. The dynamic model is
%  described as an explicit ordinary differential equation of the form:
%
%              dchi = forwardDynamics(t,chi)
%
%  where chi is the variable to be integrated. For a floating base
%  articulated chain, the variable chi contains the following
%  subvariables:
%
%  x_b:      the cartesian position of the base (R^3)
%  qt_b:     the quaternion describing the orientation of the base (global parametrization of SO(3))
%  qj:       the joint positions (R^ndof)
%  dx_b:     the cartesian velocity of the base (R^3)
%  omega_b:  the velocity describing the orientation of the base (so(3))
%  dqj:      the joint velocities (R^ndof)

  waitbar(t/param.tEnd,param.wait)
% disp(t)

%% Extraction of state
ndof = param.ndof;

% position and orientation
x_b  = chi(1:3,:); 
qt_b = chi(4:7,:);
qj   = chi(8:ndof+7,:);

% linear and angular velocity
dx_b    = chi(ndof+8:ndof+10,:);
omega_w = chi(ndof+11:ndof+13,:);
dqj     = chi(ndof+14:2*ndof+13,:);

Nu      = [dx_b; omega_w; dqj];

%% Getting the rotation matrix from root link to world frame
qT         = [x_b;qt_b];
[~,R_b]    = frame2posrot(qT);
 
%% MexWholeBodyModel functions
% dynamics
M      = wbm_massMatrix(R_b,x_b,qj); 
h      = wbm_generalisedBiasForces(R_b,x_b,qj,dqj,[dx_b;omega_w]);
H      = wbm_centroidalMomentum(R_b,x_b,qj,dqj,[dx_b;omega_w]);

% forward kinematics
l_sole   = wbm_forwardKinematics(R_b,x_b,qj,'l_sole');
r_sole   = wbm_forwardKinematics(R_b,x_b,qj,'r_sole');
CoM      = wbm_forwardKinematics(R_b,x_b,qj,'com');

%% Building up jacobians and dJNu
% contact jacobians
Jc    = zeros(6*param.numConstraints,6+ndof);
dJcNu = zeros(6*param.numConstraints,1);

for i=1:param.numConstraints
    
    Jc(6*(i-1)+1:6*i,:)    = wbm_jacobian(R_b,x_b,qj,param.constraintLinkNames{i});
    dJcNu(6*(i-1)+1:6*i,:) = wbm_djdq(R_b,x_b,qj,dqj,[dx_b;omega_w],param.constraintLinkNames{i});
    
end

% CoM jacobian
J_CoM   = wbm_jacobian(R_b,x_b,qj,'com');

%% Joint limits check
limits = param.limits;
l_min  = limits(:,1);
l_max  = limits(:,2);
tol    = 0.01;

res = qj < l_min + tol | qj > l_max - tol;
res = sum(res);

if res==0

else
 
 disp('Joint limits reached at time:')    
 disp(t)
 error('Joint limits reached '); 

end

%% Feet correction to avoid numerical integration errors
% feet correction gain
K_corr_pos  = 5;
K_corr_vel  = 2*sqrt(K_corr_pos);

% feet current position and orientation
[x_lfoot,R_b_lfoot]    = frame2posrot(l_sole);
[x_rfoot,R_b_rfoot]    = frame2posrot(r_sole);

% orientation is parametrized with euler angles
[~,phi_lfoot]       = parametrization(R_b_lfoot);
[~,phi_rfoot]       = parametrization(R_b_rfoot);

pos_leftFoot        = [x_lfoot; phi_lfoot'];
pos_rightFoot       = [x_rfoot; phi_rfoot'];

% feet original position and orientation
lsole_ini           = param.lfoot_ini;
rsole_ini           = param.rfoot_ini;

[xi_lfoot,R_bi_lfoot]  = frame2posrot(lsole_ini);
[xi_rfoot,R_bi_rfoot]  = frame2posrot(rsole_ini);

[~,phi_rfoot_ini]    = parametrization(R_bi_rfoot);
[~,phi_lfoot_ini]    = parametrization(R_bi_lfoot);

lfoot_ini_tot     = [xi_lfoot; phi_lfoot_ini'];
rfoot_ini_tot     = [xi_rfoot; phi_rfoot_ini'];
  
% error between original and current feet position and orientation
if     params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 0
     
       pos_feet_delta = pos_leftFoot-lfoot_ini_tot;
 
elseif params.feet_on_ground(1) == 0 && params.feet_on_ground(2) == 1
     
       pos_feet_delta = pos_rightFoot-rfoot_ini_tot;       

elseif params.feet_on_ground(1) == 1 && params.feet_on_ground(2) == 1
    
       pos_feet_delta = [(pos_leftFoot-lfoot_ini_tot);...
                         (pos_rightFoot-rfoot_ini_tot)];    
end

%% Control torques calculation
xCom          = CoM(1:3);
xComDes       = param.com_ini(1:3);
param.qj      = qj;

% Gains and friction cones definition
[gainParam, constraintParam, trajParam] = gains_and_constraints(param);

% CoM trajectory generator
desired_x_dx_ddx_CoM = generTraj(xComDes,t,trajParam);

% Balancing controller
[tauModel,Sigma,NA,fHdotDesC1C2,errorCoM,f0]   = controllerFcn(param.numConstraints,param.ndof,param.QP_solver,constraintParam,qj,param.qjInit,Nu, M, h, H, l_sole, r_sole,footSize, Jc,...
                                                               dJcNu, xCom, J_CoM, desired_x_dx_ddx_CoM,gainParam);      
  
% Desired contact forces at feet and control torques
fc_des  = fHdotDesC1C2 + NA*f0;
tau     = tauModel + Sigma*fc_des;

%% Real contact forces computation
Jct            = Jc';
St             = [zeros(6,ndof); eye(ndof)];
M_inv          = eye(ndof+6)/M;
JcMinvSt       = Jc*M_inv*St;
JcMinv         = Jc*M_inv;
inv_JcMinvJct  = eye(6*param.feet_on_ground)/(Jc*M_inv*Jct);

f_c            = inv_JcMinvJct*(JcMinv*h -JcMinvSt*tau -dJcNu -K_corr_vel.*Jc*Nu -K_corr_pos.*pos_feet_delta);

%% State derivative computation
% Need to apply root-to-world rotation to the spatial angular velocity omega_w to
% obtain angular velocity in body frame omega_b. 
% This is then used in the quaternion derivative computation.
omega_b = transpose(R_b)*omega_w;                               
dqt_b   = quaternionDerivative(omega_b,qt_b);       

dx      = [dx_b;dqt_b;dqj];
dv      = M\(Jc'*f_c + [zeros(6,1); tau]-h);

dchi   = [dx;dv];  

%% Visualization 
% These are the variables that can be plotted by the "visualizer graphics"
% function
  visual_param.pos_feet  =  [l_sole;r_sole];
  visual_param.fc        =  f_c;
  visual_param.tau       =  tau;
  visual_param.qj        =  qj;
  visual_param.error_com =  errorCoM;
  visual_param.f0        =  f0;

end

