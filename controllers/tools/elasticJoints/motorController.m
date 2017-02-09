function [tau_xi,dxi_ref] = motorController(dxi,xi,ELASTICITY,STATE,controlParam,CONFIG)
%MOTORCONTROLLER computes desired motor torques for controlling floating
%                base robots with elastic joiints.
%
% tau = MOTORCONTROLLER(dtheta,theta,ELASTICITY,STATE,controlParam,CONFIG) 
% takes as input the parameters from stack of task controllers, the robot
% state, the motor state and dynamics and configuration.
% The output are the desired motor torques tau_m and the motor reference 
% velocities, dtheta_ref.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
%% Configure parameters
ndof = CONFIG.ndof;
qj   = STATE.qj;
dqj  = STATE.dqj;

% desired references for motor velocity
dxi_ref       = ELASTICITY.KD\(controlParam.tauModel+controlParam.Sigma*controlParam.fcDes);
ddxi_ref      = zeros(ndof,1);

% compensation for stability (proper backstepping)
error_compens = controlParam.error_compens;

% feedback linearization of motor dynamics
u      = ddxi_ref - ELASTICITY.KD_gain*(dxi-dxi_ref) + error_compens;
tau_xi = ELASTICITY.B_xi*u + ELASTICITY.KD*(dxi-dqj) + ELASTICITY.KS*(xi-qj); 

end

