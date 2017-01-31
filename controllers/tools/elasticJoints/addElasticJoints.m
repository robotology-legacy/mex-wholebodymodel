function ELASTICITY = addElasticJoints(CONFIG)

ndof                 = CONFIG.ndof;
ELASTICITY.B         = 0.1*eye(ndof);
ELASTICITY.KS        = 0.1*eye(ndof);
ELASTICITY.KD        = 0.1*eye(ndof);
ELASTICITY.KD_gain   = 0.1*eye(ndof);

end
