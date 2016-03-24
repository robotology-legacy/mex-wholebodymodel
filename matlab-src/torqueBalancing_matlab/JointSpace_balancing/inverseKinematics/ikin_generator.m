function [ddqjDes, dikin_total, traj_obt, delta] = ikin_generator(desired_x_dx_ddx_CoM, param)
%% ikin_generator 
%  Computes the inverse kinematics of the robot from a desired CoM position 
%  to the desired joints position. 
%  The outputs are:
%
%  ddqjDes      [ndofx1]              which is the vector of desired joints accelerations
%
%  d_ikin_total [ndof+6x1]            which is the vector of desired robot state time
%                                     derivative
%
%  traj_obt     [9x1]                 which contains the desired CoM linear position,
%                                     velocity and acceleration
%
%  delta        [3+nfeet_on_groundx1] which contains the CoM and feet position errors

%% Setup parameters
PINV_TOL    = 1e-8;

Kcorr_feet  = 10;
Kcorr_CoM   = 10;
Kcorr_qj    = 10;
ndof        = param.ndof;  

%% Terms coming from state integration
integr_terms = param.kin_total;

qt_b             = integr_terms(1:7);
qjDes            = integr_terms(8:7+ndof);

[x_b_Des,Rb_Des] = frame2posrot(qt_b);

%% Jacobian at feet and CoM 
for ii=1:param.numConstraints
    
    Jc_ikin(6*(ii-1)+1:6*ii,:)    = wbm_jacobian(Rb_Des,x_b_Des,qjDes,param.constraintLinkNames{ii});

end

JCoM_ikin_total  =  wbm_jacobian(Rb_Des,x_b_Des,qjDes,'com');
JCoM_ikin        =  JCoM_ikin_total(1:3,:);

%% CoM position
CoM_PosAndOrient     = wbm_forwardKinematics(Rb_Des,x_b_Des,qjDes,'com');
xCoM_real            = CoM_PosAndOrient(1:3);

%% Feet position and orientation
lsole   = wbm_forwardKinematics(Rb_Des,x_b_Des,qjDes,'l_sole');
rsole   = wbm_forwardKinematics(Rb_Des,x_b_Des,qjDes,'r_sole');

[x_lfoot,Rb_lfoot]    = frame2posrot(lsole);
[x_rfoot,Rb_rfoot]    = frame2posrot(rsole);

[T_rfoot,phi_rfoot]   = parametrization(Rb_rfoot);
[T_lfoot,phi_lfoot]   = parametrization(Rb_lfoot);

pos_leftFoot          = [x_lfoot; phi_lfoot.'];
pos_rightFoot         = [x_rfoot; phi_rfoot.'];

% initial feet position and orientation
lfoot_ini             = param.lfoot_ini;
rfoot_ini             = param.rfoot_ini;

[xInit_rfoot,RbInit_rfoot]    = frame2posrot(rfoot_ini);
[xInit_lfoot,RbInit_lfoot]    = frame2posrot(lfoot_ini);

[~,phiInit_lfoot]             = parametrization(RbInit_lfoot);
[~,phiInit_rfoot]             = parametrization(RbInit_rfoot);

lfoot_initPosAndOrient        = [xInit_lfoot; phiInit_lfoot.'];
rfoot_initPosAndOrient        = [xInit_rfoot; phiInit_rfoot.'];
  
 if       sum(param.feet_on_ground) == 1 && param.feet_on_ground(1) == 1
     
 T_tildeLeftFoot       = [eye(3) zeros(3);
                          zeros(3)  T_lfoot];
              
 feetPosAndOrient_real = T_tildeLeftFoot*(pos_leftFoot-lfoot_initPosAndOrient);
 
 elseif   sum(param.feet_on_ground) == 1 && param.feet_on_ground(2) == 1
     
 T_tildeRightFoot       = [eye(3) zeros(3);
                          zeros(3)  T_rfoot];
              
 feetPosAndOrient_real = T_tildeRightFoot*(pos_rightFoot-rfoot_initPosAndOrient);
 
 elseif   sum(param.feet_on_ground) == 2
     
 T_tildeLeftFoot       = [eye(3) zeros(3);
                          zeros(3)  T_lfoot];
              
 T_tildeRightFoot      = [eye(3) zeros(3);
                          zeros(3)  T_rfoot];
              
 T_tildeTotal          = [T_tildeLeftFoot   zeros(6);
                          zeros(6)   T_tildeRightFoot];
              
 feetPosAndOrient_real = T_tildeTotal*[pos_leftFoot-lfoot_initPosAndOrient;
                                       pos_rightFoot-rfoot_initPosAndOrient];
              
 end 

%% CoM and feet correction terms
delta_pos_CoM   = xCoM_real - desired_x_dx_ddx_CoM(:,1);
delta_pos_feet  = feetPosAndOrient_real;

%% Desired joints velocity
Nu_ikin         = integr_terms(8+ndof:end);
dqjDes          = Nu_ikin(7:end);

Nu_ikin_base    = Nu_ikin(1:6);                               
dqt_b           = quaternionDerivative(transpose(Rb_Des)*Nu_ikin_base(4:end), qt_b(4:end));
 
%% Jacobian time derivative
for ii=1:param.numConstraints
    
    dJcNu(6*(ii-1)+1:6*ii,:) = wbm_djdq(Rb_Des,x_b_Des,qjDes,dqjDes,Nu_ikin_base,param.constraintLinkNames{ii});

end

dJCoMNu_total =  wbm_djdq(Rb_Des,x_b_Des,qjDes,dqjDes,Nu_ikin_base,'com');
dJCoMNu       =  dJCoMNu_total(1:3);

%% Feet and CoM corrections of velocities
Kcorr_feet_Nu = 2*sqrt(Kcorr_feet);
Kcorr_CoM_Nu  = 2*sqrt(Kcorr_CoM);
Kcorr_dq      = 2*sqrt(Kcorr_qj);

delta_vel_feet = Jc_ikin*Nu_ikin;
delta_vel_CoM  = JCoM_ikin*Nu_ikin - desired_x_dx_ddx_CoM(:,2);

%% Desired joints accelerations
% first task: constraints at feet 
feetPosAndOrient  = -dJcNu -Kcorr_feet*delta_pos_feet -Kcorr_feet_Nu*delta_vel_feet;
NullFeet          = eye(ndof+6) - pinv(Jc_ikin,PINV_TOL)*Jc_ikin;

% second task: CoM trajectory
CoMPos            = -dJCoMNu + desired_x_dx_ddx_CoM(:,3) - Kcorr_CoM*delta_pos_CoM -Kcorr_CoM_Nu*delta_vel_CoM...
                    -JCoM_ikin*pinv(Jc_ikin,PINV_TOL)*feetPosAndOrient;

JCoM_0            = JCoM_ikin*NullFeet;
NullCoM           = eye(ndof+6) - pinv(JCoM_0,PINV_TOL)*JCoM_0;

% third task: posture
Jddqj_ikin        = [zeros(ndof,6) eye(ndof)];

posturePos        = -Kcorr_dq*(Nu_ikin(7:end)) -Kcorr_qj*(qjDes-param.qjInit)...
                    -Jddqj_ikin*pinv(Jc_ikin,PINV_TOL)*feetPosAndOrient -Jddqj_ikin*NullFeet*pinv(JCoM_0,PINV_TOL)*CoMPos;

Jddqj_0           = Jddqj_ikin*NullFeet*NullCoM;

% state acceleration
dNu_00        = pinv(Jddqj_0,PINV_TOL)*posturePos;
dNu_0         = pinv(JCoM_0, PINV_TOL)*CoMPos + NullCoM*dNu_00;
dNu_ikin      = pinv(Jc_ikin,  PINV_TOL)*feetPosAndOrient + NullFeet*dNu_0;

% joints acceleration
ddqjDes       = dNu_ikin(7:end);

%% Vector for integration
dikin_total   = [Nu_ikin_base(1:3); dqt_b; dqjDes; dNu_ikin];

%% Obtained trajectory at CoM
traj_obt      = [xCoM_real; JCoM_ikin*Nu_ikin; (JCoM_ikin*dNu_ikin+dJCoMNu)];

%% CoM and feet position error 
delta         = [delta_pos_CoM; delta_pos_feet];

end


