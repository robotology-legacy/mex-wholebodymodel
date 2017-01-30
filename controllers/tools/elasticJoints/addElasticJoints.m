function ELASTICITY            = addElasticJoints(CONFIG)

ndof = CONFIG.ndof;
ELASTICITY.B  = eye(ndof);
ELASTICITY.KS = eye(ndof);
ELASTICITY.KD = eye(ndof);
ELASTICITY.KS_gain = eye(ndof);
ELASTICITY.KD_gain = eye(ndof);
