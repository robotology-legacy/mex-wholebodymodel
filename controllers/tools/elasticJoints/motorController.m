function [tau_xi,dxi_ref] = motorController(DYNAMICS,STATE,GAINS,CONTROLLER)
%MOTORCONTROLLER computes desired motor torques for controlling floating
%                base robots with elastic joints.
%
% Format: [tau_xi,dxi_ref] = MOTORCONTROLLER(DYNAMICS,STATE,GAINS,CONTROLLER)
%
% Inputs:  - DYNAMICS contains current robot dynamics;
%          - STATE contains the current system state;
%          - GAINS is a structure containing all control gains;
%          - CONTROLLER is a structure containing the control torques, 
%            and other parameters for controlling the robot.
%
% Output:  - tau_xi control torques [ndof x 1] 
%          - dxi_ref motors reference velocity [ndof x 1] 
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% state parameters
qj   = STATE.qj;
dqj  = STATE.dqj;
dxi  = STATE.dxi;
xi   = STATE.xi;

% dynamic parameters
KS   = DYNAMICS.KS;
KD   = DYNAMICS.KD;
B_xi = DYNAMICS.B_xi;

% desired references for motor velocity
dxi_ref = KD\(CONTROLLER.tauModel + CONTROLLER.Sigma*CONTROLLER.fcDes);

%% Feedback linearization of motor dynamics
% control input
u      =  -GAINS.damping_xi*(dxi-dxi_ref);
% control torques
tau_xi =   B_xi*u + KD*(dxi-dqj) + KS*(xi-qj); 

end

