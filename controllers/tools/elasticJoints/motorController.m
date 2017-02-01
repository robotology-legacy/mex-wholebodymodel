function [tau_m,dtheta_ref] = motorController(dtheta,theta,ELASTICITY,STATE,controlParam)
%MOTORCONTROLLER computes desired motor torques for controlling floating
%                base robots with elastic joiints.
%
% tau = MOTORCONTROLLER(dtheta,theta,ELASTICITY,STATE,controlParam) takes
% as input the parameters from stack of task controllers, the robot state,
% the motor state and dynamics.
% The output are the desired motor torques tau_m and the motor reference 
% velocities, dtheta_ref.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
%% Configure parameters
qj  = STATE.qj;
dqj = STATE.dqj;

% desired reference for motor velocity
dtheta_ref  = ELASTICITY.KD\(controlParam.tauModel+controlParam.Sigma*controlParam.fcDes);
ddtheta_ref = ELASTICITY.B/ELASTICITY.KD*controlParam.ddtheta_ref;

% feedback linearization of motor dynamics
u     = ddtheta_ref - ELASTICITY.KD_gain*(dtheta-dtheta_ref);
tau_m = u + ELASTICITY.KD*(dtheta-dqj) + ELASTICITY.KS*(theta-qj); 

end

