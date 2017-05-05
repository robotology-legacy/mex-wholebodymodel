function [B_xi,KS,KD,damping_xi] = addElasticJoints(MODEL)
%ADDELASTICJOINTS configures the motor dynamics in case joints elasticity
%                 is considered in the model.
%
% Format: [B_xi,KS,KD,damping_xi] = ADDELASTICJOINTS(MODEL)
%
% Inputs:  - MODEL is a structure defining the robot model;
%
% Output:  - B_xi motor intertia matrix (R^(ndof x ndof))
%          - KS joint stiffness (R^(ndof x ndof))
%          - KD joint damping (R^(ndof x ndof))
%          - damping_xi motors velocity control gain (R^(ndof x ndof))
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% Configure parameters
ndof                 = MODEL.ndof;
robot_name           = MODEL.CONFIG.robot_name;
% transmission ratio
eta                  = MODEL.eta;

%% Stiffness, damping and motor inertia (expressed w.r.t. the joint side)
if strcmp(robot_name,'bigman')
    % motor inertia [Kgm^2]
    B_xi       = 5.480e-5./(eta^2).*eye(ndof);
    % joint stiffness [Nm/rad]
    KS         = 50*eye(ndof); 
    % joint damping [Nms/rad]
    KD         = 5*eye(ndof); 
    % control gains
    damping_xi = 100*eye(ndof);

elseif strcmp(robot_name,'icubGazeboSim') == 1
    % motor inertia [Kgm^2]
    B_xi       = 1e-5./(eta^2).*eye(ndof); 
    % joint stiffness [Nm/rad]
    KS         = 350*eye(ndof);    
    % joint damping [Nms/rad]
    KD         = 1*eye(ndof);   
    % control gains
    damping_xi = 200*eye(ndof);
    
elseif strcmp(robot_name,'bigman_only_legs') == 1
    % motor inertia [Kgm^2]
    B_xi       = 5.480e-5./(eta^2).*eye(ndof);
    % joint stiffness [Nm/rad]
    KS         = 50*eye(ndof); 
    % joint damping [Nms/rad]
    KD         = 5*eye(ndof);
    % control gains
    damping_xi = 100*eye(ndof);
end

end
