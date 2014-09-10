function [ qvDot ] = forwardDynamics_t(t, qv,qj_init, param )
%FORWARDDYNAMICS Forward dynamics of WBI ICub
%   This is the forward dynamics of the WBI Icub meant for integration in a
%   ode pkg function. 

constraintLink1 = 'l_sole';
constraintLink2 = 'r_sole';


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
%MTilde = wholeBodyModel('mass-matrix',qj);

H = wholeBodyModel('generalised-forces');   
g = wholeBodyModel('generalised-forces',qj,zeros(25,1),zeros(6,1)); 
RightFoot = wholeBodyModel('forward-kinematics',qj,constraintLink2);
PosRightFoot = RightFoot(1:3);

Jleft  = reshape(wholeBodyModel('jacobian',constraintLink1),31,6)';
Jright = reshape(wholeBodyModel('jacobian',constraintLink2),31,6)';
feetJacobians = [Jleft;Jright];

djdq1 = wholeBodyModel('djdq',constraintLink1);
djdq2 = wholeBodyModel('djdq',constraintLink2);

feetJacobians = [Jleft;Jright];
feetJdqd = [djdq1;djdq2];

com_t = wholeBodyModel('forward-kinematics',qj,'com');
xcom = com_t(1:3);
Jcom  = reshape(wholeBodyModel('jacobian','com'),31,6)';

Desired_x_dx_ddx_CoM = [0.4 0 0 ; 0 0 0 ; 0 0 0];

Gains  = [  70    0   0  4];

% Impadances acting in the null space of the desired contact forces 

kImpTorso = [12 8 12]; 
kImpArms  = [10 10 10 10 5];
kImpLegs  = [35 50 0.1 30 2 10]; 

Impedances  = [kImpTorso,kImpArms,kImpArms,kImpLegs,kImpLegs];

IntErrorCoM = [0;0;0];
%MInv = M^(-1);
% RobotModel components

qD=v;
Hcm = zeros(6,1);
[tau,CoMError] = cont_fcn_controller(qj,qj_init,qD, M, H, g, Hcm, PosRightFoot, feetJacobians, feetJdqd, xcom,Jcom, Desired_x_dx_ddx_CoM, Gains, Impedances, IntErrorCoM);
tau = zeros(25,1);

% [Fc,J] = constraintBothFeetOnGround(qj,qjDot,tau(t), M, H);
    JMInv = feetJacobians/M;
    djdq = feetJdqd;
    
    c_fr = 0;
    fric = [zeros(6,31);c_fr*eye(25,31)]*[zeros(6,1);qjDot];
    
    J=feetJacobians;

% with controller    
    Fc = ( JMInv * J')\ ( (-JMInv * ( -H  - fric + [zeros(6,1);tau])) - djdq);
% without controller

%     Fc = ( JMInv * J')\ ( (-JMInv * ( -H  - fric + [zeros(6,1);tau(t)])) - djdq);
    

%getting qDot
QDot_base = quaternionDerivative(omega_base, Q_base);%,param.QuaternionDerivativeParam);

qDot = [pDot_base;QDot_base;qjDot];

c_fr = 0;
fric = [zeros(6,31);c_fr*eye(25,31)]*[zeros(6,1);qjDot];

% without controller
% vDot = M\ ( J'*Fc + [zeros(6,1);tau(t)] - H - fric);
% with controller    
vDot = M\ ( J'*Fc + [zeros(6,1);tau] - H - fric);

%qvDot = zeros(13+2*n,1);
qvDot = [qDot;vDot];

end

