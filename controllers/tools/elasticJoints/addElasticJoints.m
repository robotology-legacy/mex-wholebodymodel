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

%% Motor dynamics
% motor inertia
ELASTICITY.B         = 1e-5*eye(ndof);

% stiffness
ELASTICITY.KS        = 1e4*eye(ndof);

% damping
ELASTICITY.KD        = 10*eye(ndof);

% contorl gains
ELASTICITY.KD_gain   = 10000*eye(ndof);

end
