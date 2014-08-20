function [ qvDot ] = forwardDynamics(t, qv, param )
%FORWARDDYNAMICS Forward dynamics of WBI ICub
%   This is the forward dynamics of the WBI Icub meant for integration in a
%   ode pkg function. 

n = param.ndof;
tau = param.tau;

q = qv(1:7+n,:);
p_base = q(1:3,:); % Linear Position of Floating Base
Q_base = q(4:7,:); % Orientation of Floating Base (quaternions)
qj = q(8:end,:); % Joint angles

v = qv(8+n:end,:);
pDot_base = v(1:3,:); % Linear velocity of Floating Base
omega_base = v(4:6,:); % Rotational velocity of Floating base
qjDot = v(7:end,:);

%Mex-WholeBodyModel Components
wholeBodyModel('update-state',qj,qjDot,[pDot_base;omega_base]);
M = wholeBodyModel('mass-matrix');    
H = wholeBodyModel('generalised-forces');   

MInv = M^(-1);
% RobotModel components
[Fc,J] = constraintBothFeetOnGround(tau(t), M, H);

%getting qDot
QDot_base = quaternionDerivative(omega_base, Q_base);%,param.QuaternionDerivativeParam);

qDot = [pDot_base;QDot_base;qjDot];


vDot = MInv * ( J'*Fc + [zeros(6,1);tau(t)] - H);

%qvDot = zeros(13+2*n,1);
qvDot = [qDot;vDot];

end

