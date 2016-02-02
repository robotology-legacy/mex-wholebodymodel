function [dchi,visual_param]=forwardDynamics_SoT(t,chi,param)
%% forwardDynamics_SoT
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
%  omega_b:  the velocity describing the orientation of the base (SO(3))
%  dqj:      the joint velocities (R^ndof)

  waitbar(t/param.tEnd,param.wait)
  ndof = param.ndof;
% disp(t)

%% Extraction of state
% position and orientation
x_b  = chi(1:3,:); 
qt_b = chi(4:7,:);
qj   = chi(8:ndof+7,:);

% linear and angular velocity
dx_b    = chi(ndof+8:ndof+10,:);
omega_w = chi(ndof+11:ndof+13,:);
dqj     = chi(ndof+14:2*ndof+13,:);

Nu      = [dx_b;omega_w;dqj];

% Obtaining the rotation matrix from root link to world frame
qT         = [x_b;qt_b];
[~,R_b]    = frame2posrot(qT);
 
%% MexWholeBodyModel functions
% dynamics
M        = wbm_massMatrix(R_b,x_b,qj); 
h        = wbm_generalisedBiasForces(R_b,x_b,qj,dqj,[dx_b;omega_w]);
H        = wbm_centroidalMomentum(R_b,x_b,qj,dqj,[dx_b;omega_w]);

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
J_CoM = wbm_jacobian(R_b,x_b,qj,'com');

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
[~,phi_lfoot]          = parametrization(R_b_lfoot);
[~,phi_rfoot]          = parametrization(R_b_rfoot);

pos_leftFoot           = [x_lfoot; phi_lfoot'];
pos_rightFoot          = [x_rfoot; phi_rfoot'];

% feet original position and orientation
lsole_ini              = param.lfoot_ini;
rsole_ini              = param.rfoot_ini;

[xi_lfoot,R_bi_lfoot]  = frame2posrot(lsole_ini);
[xi_rfoot,R_bi_rfoot]  = frame2posrot(rsole_ini);

[~,phi_rfoot_ini]      = parametrization(R_bi_rfoot);
[~,phi_lfoot_ini]      = parametrization(R_bi_lfoot);

lfoot_ini_tot          = [xi_lfoot; phi_lfoot_ini'];
rfoot_ini_tot          = [xi_rfoot; phi_rfoot_ini'];
  
% error between original and current feet position and orientation
if     param.feet_on_ground(1) == 1 && param.feet_on_ground(2) == 0
     
       pos_feet_delta = pos_leftFoot-lfoot_ini_tot;
 
elseif param.feet_on_ground(1) == 0 && param.feet_on_ground(2) == 1
     
       pos_feet_delta = pos_rightFoot-rfoot_ini_tot;       

elseif param.feet_on_ground(1) == 1 && param.feet_on_ground(2) == 1
    
       pos_feet_delta = [(pos_leftFoot-lfoot_ini_tot);...
                         (pos_rightFoot-rfoot_ini_tot)];    
end

% parameters for controller
feet.l_sole         = l_sole;
feet.r_sole         = r_sole;

%% Control torques calculation
xCoM          = CoM(1:3);
xCoMDes       = param.CoM_ini(1:3);
param.qj      = qj;

% gains and friction cones definition
[gains,constraints,trajectory] = gainsAndConstraints_SoT(param);

% CoM trajectory generator
desired_x_dx_ddx_CoM = generTraj_SoT(xCoMDes,t,trajectory);

% balancing controller
[tau,errorCoM,f0] = stackOfTaskController(param, constraints, feet, gains, Nu, M, h, H, Jc, dJcNu, xCoM, J_CoM, desired_x_dx_ddx_CoM);    
        
% Real contact forces computation
S               = [ zeros(6,ndof);
                    eye(ndof,ndof)];
JcMinv          = Jc/M;
JcMinvS         = JcMinv*S;

fc              = (JcMinv*transpose(Jc))\(JcMinv*h -JcMinvS*tau -dJcNu -K_corr_vel.*Jc*Nu -K_corr_pos.*pos_feet_delta);

%% State derivative computation
% Need to calculate the quaternions derivative
omega_b = transpose(R_b)*omega_w;                               
dqt_b   = quaternionDerivative(omega_b,qt_b);       

dx      = [dx_b;dqt_b;dqj];
dNu     = M\(Jc'*fc + [zeros(6,1); tau]-h);

dchi    = [dx;dNu];  

%% Visualization 
% These are the variables that can be plotted by the visualizer.m
% function
 visual_param.pos_feet  =  [l_sole;r_sole];
 visual_param.fc        =  fc;
 visual_param.tau       =  tau;
 visual_param.qj        =  qj;
 visual_param.error_com =  errorCoM;
 visual_param.f0        =  f0;

end

