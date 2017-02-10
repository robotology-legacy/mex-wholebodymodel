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
robot_name           = CONFIG.robot_name;

if strcmp(robot_name,'bigman') == 1
    
    % reduction ratio
    p                    = CONFIG.p;

    %% Motor dynamics
    % motor inertia
    ELASTICITY.B_xi      = 5.480e-5*(p^2)*eye(ndof);

    % stiffness
    ELASTICITY.KS        = 50*eye(ndof);

    % damping
    ELASTICITY.KD        = 5*eye(ndof);

    % control gains
    ELASTICITY.KD_gain   = 100*eye(ndof);

elseif strcmp(robot_name,'icubGazeboSim') == 1
    
    % reduction ratio
    p                    = CONFIG.p;

    %% Motor dynamics
    % motor inertia (expressed w.r.t. the link side)
    ELASTICITY.B_xi      = 1e-5*(p^2)*eye(ndof);

    % stiffness
    ELASTICITY.KS        = 100*eye(ndof);

    % damping
    ELASTICITY.KD        = 10*eye(ndof);

    % control gains
    ELASTICITY.KD_gain   = 100*eye(ndof); 
    
elseif strcmp(robot_name,'bigman_only_legs') == 1
    
    % reduction ratio
    p                    = CONFIG.p;

    %% Motor dynamics
    % motor inertia
    ELASTICITY.B_xi      = 5.480e-5*(p^2)*eye(ndof);

    % stiffness
    ELASTICITY.KS        = 100*eye(ndof);

    % damping
    ELASTICITY.KD        = 5*eye(ndof);

    % control gains
    ELASTICITY.KD_gain   = 100*eye(ndof);  

end

end
