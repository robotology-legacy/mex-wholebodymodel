function tau = motorController(dtheta,thetaRef,theta,ELASTICITY,STATE)

qj  = STATE.qj;
dqj = STATE.dqj;
tau = u -ELASTICITY.KD*dqj + ELASTICITY.KS*(theta-qj) -ELASTICITY.KS_gain*(theta-thetaRef)  -ELASTICITY.KD_gain*dtheta;