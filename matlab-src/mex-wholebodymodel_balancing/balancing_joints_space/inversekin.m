function [ddqDes, dqDes] = inversekin (inverse_param,desired_x_dx_ddx_CoM)

PINV_TOL = 1e-10;
Jc    = inverse_param.Jc;
J_CoM = inverse_param.Jcom;

Kcorr     =  5;
Kcorr2 = 2*sqrt(Kcorr);

p_feet = inverse_param.pos_feet;
corr_term_feet= -Kcorr*p_feet;


Jc_tot   = -(eye(6)/Jc(1:6,1:6))*Jc(1:6,7:end);

Jc_rf    = (Jc(7:end,1:6)*Jc_tot + Jc(7:end,7:end));
J_CoM_rf = (J_CoM(1:3,1:6)*Jc_tot + J_CoM(1:3,7:end));


Jt_bar   = [Jc_rf ; J_CoM_rf];


xTilde    =  inverse_param.xdes_real - desired_x_dx_ddx_CoM(:,1);
corr_term = -Kcorr*xTilde;


vett   = [corr_term_feet(7:end); (desired_x_dx_ddx_CoM(:,2)+corr_term)];

dqDes = pinv(Jt_bar, PINV_TOL)*vett;

term1 = Kcorr2*Jc_rf*dqDes;
term2 = Kcorr2*J_CoM_rf*dqDes;

vel = [(-inverse_param.dJcDq(7:end)-term1);(-inverse_param.dJcom(1:3)-term2)];
ddqDes = pinv(Jt_bar,PINV_TOL)*(vett+vel);

end