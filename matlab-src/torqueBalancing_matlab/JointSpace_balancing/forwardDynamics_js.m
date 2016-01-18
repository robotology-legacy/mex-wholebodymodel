function [dchi,visual_param] = forwardDynamics_js(t,chi,param )
%% forwardDynamics_js 
%  Forward dynamics of the wholeBodyModel
%
%  This is the forward dynamics of the model loaded in the 
%  wholeBodyInterface from the URDF description. The dynamic model is
%  described as an explicit ordinary differential equation of the form:
%
%                 dchi = forwardDynamics( t,chi)
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

%disp(t);
waitbar(t/param.tEnd,param.wait)
ndof  = param.ndof; 

%% Extraction of state in the base frame of reference
% position
x_b   = chi(1:3,:); 
qt_b  = chi(4:7,:);
qj    = chi(8:ndof+7,:);

% velocity
dx_b     = chi(ndof+8:ndof+10,:);
omega_w  = chi(ndof+11:ndof+13,:);
dqj      = chi(ndof+14:2*ndof+13,:);

Nu       = [dx_b; omega_w; dqj];
param.qj = qj;

%% World to base rotation matrix
qT         = [x_b; qt_b];
[~,R_b]    = frame2posrot(qT);

%% Parameters definition
% dynamics
M       = wbm_massMatrix(R_b,x_b,qj); 
h       = wbm_generalisedBiasForces(R_b,x_b,qj,dqj,[dx_b;omega_w]);
g       = wbm_generalisedBiasForces(R_b,x_b,qj,zeros(25,1),zeros(6,1));

% forward kinematics
CoM     = wbm_forwardKinematics(R_b,x_b,qj,'com');
lsole   = wbm_forwardKinematics(R_b,x_b,qj,'l_sole');
rsole   = wbm_forwardKinematics(R_b,x_b,qj,'r_sole');

%% Building up constraints jacobian and dJNu
Jc    = zeros(6*param.numConstraints,6+ndof);
dJcNu = zeros(6*param.numConstraints,1);

for i=1:param.numConstraints
    
    Jc(6*(i-1)+1:6*i,:)    = wbm_jacobian(R_b,x_b,qj,param.constraintLinkNames{i});
    dJcNu(6*(i-1)+1:6*i,:) = wbm_djdq(R_b,x_b,qj,dqj,[dx_b;omega_w],param.constraintLinkNames{i});
    
end

% CoM jacobian
JCoM   = wbm_jacobian(R_b,x_b,qj,'com');

%% Centroidal coordinates transformation
% CoM linear position and velocity
dCoM    = JCoM*Nu;
xCoM    = CoM(1:3);
dxCoM   = dCoM(1:3);

% transformation matrix for centroidal
[T,dT] = centroidalTransformationT_TDot(xCoM,x_b,dxCoM,dx_b,M);

% conversion to the centroidal frame of reference
[M_c,CNu_c, g_c, Jc_c, dJcNu_c, Nu_c] = fromFloatingToCentroidalDynamics(M, h, g, Jc, dJcNu, Nu, T, dT);

%% Joint limit check
limits  = param.limits;
l_min   = limits(:,1);
l_max   = limits(:,2);
tol     = 0.01;

res = qj < l_min + tol | qj > l_max - tol;
res = sum(res);

if res==0

else
 
 disp('Joint limits reached at time:')    
 disp(t)
 error('joint limits reached '); 

end

%% Feet correction to avoid numerical integration errors
Kcorr_pos = 5;
Kcorr_vel = 2*sqrt(Kcorr_pos);

% feet position and orientation at time t
[x_rfoot,R_b_rfoot]    = frame2posrot(rsole);
[x_lfoot,R_b_lfoot]    = frame2posrot(lsole);

[T_lfoot,phi_lfoot]    = parametrization(R_b_lfoot);
[T_rfoot,phi_rfoot]    = parametrization(R_b_rfoot);

pos_leftFoot           = [x_lfoot; phi_lfoot'];
pos_rightFoot          = [x_rfoot; phi_rfoot'];

% feet initial position and orientation
lfoot_ini                      = param.lfoot_ini;
rfoot_ini                      = param.rfoot_ini;

[xInit_rfoot,R_bInit_rfoot]    = frame2posrot(rfoot_ini);
[xInit_lfoot,R_bInit_lfoot]    = frame2posrot(lfoot_ini);

[~,phiInit_lfoot]              = parametrization(R_bInit_lfoot);
[~,phiInit_rfoot]              = parametrization(R_bInit_rfoot);

lfoot_iniPosRot                = [xInit_lfoot; phiInit_lfoot'];
rfoot_iniPosRot                = [xInit_rfoot; phiInit_rfoot'];

% feet error (corrected with angles time derivative)  
 if      param.feet_on_ground(1) == 1 && param.feet_on_ground(2) == 0
 
  T_tildeLeftfoot      = [eye(3)  zeros(3);
                          zeros(3)  T_lfoot];
               
  pos_feet_delta       = T_tildeLeftfoot*(pos_leftFoot-lfoot_iniPosRot);
 
 elseif param.feet_on_ground(1) == 0 && param.feet_on_ground(2) == 1
 
  T_tildeRightfoot      = [eye(3)  zeros(3);
                          zeros(3)  T_rfoot];
               
  pos_feet_delta       = T_tildeRightfoot*(pos_rightFoot-rfoot_iniPosRot);
  
 elseif param.feet_on_ground(1) == 1 && param.feet_on_ground(2) == 1
     
  T_tildeLeftfoot      = [eye(3)  zeros(3);
                          zeros(3)  T_lfoot];
                 
  T_tildeRightfoot     = [eye(3)  zeros(3);
                          zeros(3)  T_rfoot];   
                
  T_tilde_tot          = [T_tildeLeftfoot   zeros(6);
                          zeros(6)   T_tildeRightfoot];
               
  pos_feet_delta       = T_tilde_tot*[(pos_leftFoot  - lfoot_iniPosRot);...
                                      (pos_rightFoot - rfoot_iniPosRot)];
                
 end

%% Control torques computation
% gains definition
 gains = gainsAndConstraints_js(param);

% ikin trajectory interpolation
 [jointReferences, errorCoM] = interpolation(t,param,xCoM);

% Joint space balancing controller
 tau   = JointSpaceController(param, Nu_c, M_c, g_c, CNu_c, Jc_c, dJcNu_c, gains, jointReferences);

%% Real contact forces
Jct            = transpose(Jc);
S              = [zeros(6,ndof); eye(ndof)];
Minv           = eye(ndof+6)/M;
JcMinv         = Jc*Minv;
JcMinvS        = JcMinv*S;
invJcMinvJct   = eye(6*param.numConstraints)/(JcMinv*Jct);

fc             = invJcMinvJct*(JcMinv*h -JcMinvS*tau -dJcNu -Kcorr_vel.*Jc*Nu -Kcorr_pos.*pos_feet_delta);

%% State derivative
% need to calculate the quaternions time derivative
omega_b = transpose(R_b)*omega_w;                               
dqt_b   = quaternionDerivative(omega_b, qt_b);       

dx      = [dx_b; dqt_b; dqj];
dNu     = M\(Jc'*fc + [zeros(6,1); tau]-h);

dchi    = [dx;dNu];  

%% Visualization parameters
% these are the variables that can be plotted by visualizer_js.m
  visual_param.pos_feet  = [pos_leftFoot; pos_rightFoot];
  visual_param.fc        = fc;
  visual_param.qj        = qj;
  visual_param.qjDes     = jointReferences.qjDes;
  visual_param.tau       = tau;
  visual_param.errorCoM  = errorCoM; 

end

