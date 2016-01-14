function [ddq_inv, d_kin_total, traj_obt, delta] = ikin_generator(desired_x_dx_ddx_CoM, param)
% ikin_generator 
% computes the inverse kinematics of the robot from desired
% CoM position to desired joints position
PINV_TOL    = 1e-8;
Kcorr_feet  = 10;
Kcorr_com   = 10;
Kcorr_q     = 10;
ndof        = param.ndof;  

%% Terms from integration of dv_i
integr_terms = param.kin_total;

qt_b         = integr_terms(1:7);
q_inv        = integr_terms(8:7+ndof);

[x_b_des,Rb_des] = frame2posrot(qt_b);
R_btr_des        = Rb_des';

%% Jacobian at feet and CoM 
for ii=1:param.numConstraints
    
    Jc_i(6*(ii-1)+1:6*ii,:)    = wbm_jacobian(R_btr_des,x_b_des,q_inv,param.constraintLinkNames{ii});

end

Jcom_i_tot  =  wbm_jacobian(R_btr_des,x_b_des,q_inv,'com');
Jcom_i      =  Jcom_i_tot(1:3,:);

%% CoM position
real_des_com_tot     = wbm_forwardKinematics(R_btr_des,x_b_des,q_inv,'com');
real_des_xcom        = real_des_com_tot(1:3);

%% Feet position and orientation
lsole   = wbm_forwardKinematics(R_btr_des,x_b_des,q_inv,'l_sole');
rsole   = wbm_forwardKinematics(R_btr_des,x_b_des,q_inv,'r_sole');

[x_lf,R_b_lf]    = frame2posrot(lsole);
[T_lf,phi_lf]    = parametrization(R_b_lf);
[x_rf,R_b_rf]    = frame2posrot(rsole);
[T_rf,phi_rf]    = parametrization(R_b_rf);

pos_leftFoot     = [x_lf; phi_lf.'];
pos_rightFoot    = [x_rf; phi_rf.'];

lfoot_ini       = param.lfoot_ini;
rfoot_ini       = param.rfoot_ini;

[xi_lf,R_bi_lf]    = frame2posrot(lfoot_ini);
[~,phii_lf]        = parametrization(R_bi_lf);
[xi_rf,R_bi_rf]    = frame2posrot(rfoot_ini);
[~,phii_rf]        = parametrization(R_bi_rf);

lfoot_ini_t     = [xi_lf; phii_lf.'];
rfoot_ini_t     = [xi_rf; phii_rf.'];
  
 if      param.feet_on_ground == 1
     
 T_tildelf     = [eye(3) zeros(3);
                  zeros(3)  T_lf];
              
 real_des_feet = T_tildelf*(pos_leftFoot-lfoot_ini_t);
 
 elseif  param.feet_on_ground == 2
     
 T_tildelf     = [eye(3) zeros(3);
                  zeros(3)  T_lf];
              
 T_tilderf     = [eye(3) zeros(3);
                  zeros(3)  T_rf];
              
 T_tildetot    = [T_tildelf   zeros(6);
                  zeros(6)   T_tilderf];
              
 real_des_feet = T_tildetot*[pos_leftFoot-lfoot_ini_t;
                             pos_rightFoot-rfoot_ini_t];
              
 end 

%% Correction terms at CoM and feet
delta_pos_com  = real_des_xcom - desired_x_dx_ddx_CoM(:,1);
delta_pos_feet = real_des_feet;

%% Desired joints velocity
v_i         = integr_terms(8+ndof:end);
dq_inv      = v_i(7:end);

v_ib        = v_i(1:6);                               
dqt_b       = quaternionDerivative(R_btr_des*v_ib(4:end), qt_b(4:end));
 
%% Jacobian time derivative
for ii=1:param.numConstraints
    
    dJcdv(6*(ii-1)+1:6*ii,:) = wbm_djdq(R_btr_des,x_b_des,q_inv,dq_inv,v_ib,param.constraintLinkNames{ii});

end

dJcomdv_tot =  wbm_djdq(R_btr_des,x_b_des,q_inv,dq_inv,v_ib,'com');
dJcomdv     =  dJcomdv_tot(1:3);

%% Feet and CoM corrections of velocities
Kcorr_feet_v = 2*sqrt(Kcorr_feet);
Kcorr_com_v  = 2*sqrt(Kcorr_com);
Kcorr_dq     = 2*sqrt(Kcorr_q);

delta_vel_feet = Jc_i*v_i;
delta_vel_com  = Jcom_i*v_i - desired_x_dx_ddx_CoM(:,2);

%% Desired joints accelerations
% first task: constraints at feet 
vett_feet  = -dJcdv -Kcorr_feet*delta_pos_feet -Kcorr_feet_v*delta_vel_feet;

Jtot  = Jc_i;
NJ    = eye(ndof+6) - pinv(Jtot,PINV_TOL)*Jtot;

% second task: CoM trajectory
vett_com0 = -dJcomdv + desired_x_dx_ddx_CoM(:,3) - Kcorr_com*delta_pos_com -Kcorr_com_v*delta_vel_com -Jcom_i*pinv(Jtot,PINV_TOL)*vett_feet;

Jtot0     = Jcom_i*NJ;
NJ0       = eye(ndof+6) - pinv(Jtot0,PINV_TOL)*Jtot0;

% third task: posture
Jddq_i      = [zeros(ndof,6) eye(ndof)];

vett_qqd00  = -Kcorr_dq*(v_i(7:end)) -Kcorr_q*(q_inv-param.qjInit) -Jddq_i*pinv(Jtot,PINV_TOL)*vett_feet -Jddq_i*NJ*pinv(Jtot0,PINV_TOL)*vett_com0;
Jtot00      = Jddq_i*NJ*NJ0;

% state acceleration
dvi_00    = pinv(Jtot00,PINV_TOL)*vett_qqd00;
dvi_0     = pinv(Jtot0, PINV_TOL)*vett_com0 + NJ0*dvi_00;
dv_i      = pinv(Jtot,  PINV_TOL)*vett_feet + NJ*dvi_0;

% joints acceleration
ddq_inv     = dv_i(7:end);

%% Vector for integration
d_kin_total = [v_ib(1:3); dqt_b; dq_inv; dv_i];

%% Obtained trajectory at CoM
traj_obt    = [real_des_xcom; Jcom_i*v_i; (Jcom_i*dv_i+dJcomdv)];

%% CoM and feet position error 
delta       = [delta_pos_com; delta_pos_feet];

end


