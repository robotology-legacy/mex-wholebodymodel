function tau = motorController(dtheta,theta,ELASTICITY,STATE,controlParam)

qj  = STATE.qj;
dqj = STATE.dqj;

u   = controlParam.ddtheta_ref - ELASTICITY.KD_gain*(dtheta-controlParam.dtheta_ref);
tau = u + ELASTICITY.KD*(dtheta-dqj) - ELASTICITY.KS*(theta-qj); 

end

