function [dq_inv, ddq_inv, d_kin_total, traj_obt, Delta] = inverse_kin(desired_x_dx_ddx_CoM,param)

%This function computes the inverse kinematics of the robot from desired
%CoM position to desired joints position
PINV_TOL    = 1e-10;
Kcorr_feet  = 10;
Kcorr_com   = 10;

%% Terms from integration
integr_terms = param.d_kin_total;

qt_b         = integr_terms(1:7);
q_inv        = integr_terms(8:end);

[x_b_des,Rb_des] = frame2posrot(qt_b);
R_binv_des       = Rb_des.';

%% Jacobian at feet and CoM
for ii=1:param.numConstraints
    
    Jc_i(6*(ii-1)+1:6*ii,:)    = wbm_jacobian(R_binv_des,x_b_des,q_inv,param.constraintLinkNames{ii});

end

Jcom_i_tot  =  wbm_jacobian(R_binv_des,x_b_des,q_inv,'com');
Jcom_i      =  Jcom_i_tot(1:3,:);

%% CoM position
real_des_com_tot     = wbm_forwardKinematics(R_binv_des,x_b_des,q_inv,'com');
real_des_xcom        = real_des_com_tot(1:3);

%% Feet position and orientation
lsole   = wbm_forwardKinematics(R_binv_des,x_b_des,q_inv,'l_sole');
rsole   = wbm_forwardKinematics(R_binv_des,x_b_des,q_inv,'r_sole');

[x_lf,R_b_lf]    = frame2posrot(lsole);
[~,phi_lf]       = parametrization(R_b_lf);
[x_rf,R_b_rf]    = frame2posrot(rsole);
[~,phi_rf]       = parametrization(R_b_rf);

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
     
 real_des_feet = pos_leftFoot-lfoot_ini_t;
 
 elseif  param.feet_on_ground == 2
     
 real_des_feet = [pos_leftFoot-lfoot_ini_t;
                  pos_rightFoot-rfoot_ini_t];
              
 end 

%% Corrective terms
delta_pos_com  = real_des_xcom - desired_x_dx_ddx_CoM(:,1);
delta_pos_feet = real_des_feet;

%% Desired joints velocity
Jtot        = [Jc_i; Jcom_i];
vett        = [-Kcorr_feet*delta_pos_feet; (desired_x_dx_ddx_CoM(:,2) - Kcorr_com*delta_pos_com)];

v_i         = pinv(Jtot,PINV_TOL)*vett;
dq_inv      = v_i(7:end);

v_ib        = v_i(1:6);                               
dqt_b       = quaternionDerivative(R_binv_des*v_ib(4:end), qt_b(4:end));

d_kin_total = [v_ib(1:3); dqt_b; dq_inv];

%% Jacobian derivative
for ii=1:param.numConstraints
    
    dJcdv(6*(ii-1)+1:6*ii,:) = wbm_djdq(R_binv_des,x_b_des,q_inv,dq_inv,v_ib,param.constraintLinkNames{ii});

end

dJcomdv_tot =  wbm_djdq(R_binv_des,x_b_des,q_inv,dq_inv,v_ib,'com');
dJcomdv     =  dJcomdv_tot(1:3);

%% Feet and CoM corrections of velocities
Kcorr_feet_v = 0*2*sqrt(Kcorr_feet);
Kcorr_com_v  = 0*2*sqrt(Kcorr_com);

delta_vel_feet = Jc_i*v_i;
delta_vel_com  = Jcom_i*v_i - desired_x_dx_ddx_CoM(:,2);

%% Desired joints accelerations
vett_acc = [(-dJcdv -0*Kcorr_feet*delta_pos_feet -Kcorr_feet_v*delta_vel_feet);
            (-dJcomdv +desired_x_dx_ddx_CoM(:,3) - 0*Kcorr_com*delta_pos_com -Kcorr_com_v*delta_vel_com)];

dv_i     = pinv(Jtot,PINV_TOL)*vett_acc;

ddq_inv  = dv_i(7:end);

%% Obtained trajectory at CoM
traj_obt = [real_des_xcom; Jcom_i*v_i; (Jcom_i*dv_i+dJcomdv)];

%% Delta
Delta = [delta_pos_com; delta_pos_feet];

end


