function ELASTICITY = addElasticJoints(CONFIG)
%ADDELASTICJOINTS configure the motor dynamics in case joints elasticity
%                 is considered in the model.
%
% ELASTICITY = ADDELASTICJOINTS(CONFIG) takes as input the robot configuration.
% The output is a structure containing motors dynamics, stiffness, damping
% and motor control gains.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
%% Configure parameters
ndof                 = CONFIG.ndof;
% reduction ratio
r                    = 100;

%% Motor dynamics
% motor inertia
ELASTICITY.B         = 5.480e-5*(r^2)*eye(ndof);

% stiffness
ELASTICITY.KS        = 1000*eye(ndof);

% damping
ELASTICITY.KD        = 1000*eye(ndof);

% contorl gains
ELASTICITY.KD_gain   = 1000*eye(ndof);

end
