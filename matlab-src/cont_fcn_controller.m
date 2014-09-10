function [tau,CoMError] = fcn_controller(q,qInit,qD, M, h, g, H, PosRightFoot, feetJacobians, feetJdqd, xcom,Jcom, Desired_x_dx_ddx_CoM, Gains, Impedances, IntErrorCoM)
%#codegen
PINV_TOL = 1e-5;
gravAcc        = 9.81;

n = 25;
St    = [zeros(6,n);eye(n,n)];

m     = M(1,1);
grav = [zeros(2,1);-m*gravAcc;zeros(3,1)];

% Definition of reference accelerations for CoM and Joints that ensure
% stabilization of desired trajectories xDcomDes and qDes 

xDcom      = Jcom(1:3,:)*qD;
xDDcomStar = Desired_x_dx_ddx_CoM(:,3) - Gains(1)*(xcom - Desired_x_dx_ddx_CoM(:,1)) - Gains(2)*IntErrorCoM - Gains(3)*(xDcom - Desired_x_dx_ddx_CoM(:,2));

Pr =  PosRightFoot(1:3) - xcom; % Application point of the contact force on the right foot w.r.t. CoM

A  = [ eye(3),   zeros(3),eye(3), zeros(3);
      -Sf(xcom),  eye(3), Sf(Pr), eye(3) ];
pinvA = pinv(A, PINV_TOL);
  
HDotDes  = [ m*xDDcomStar ;
            -Gains(4)*H(4:end)]; 

Jc    =   feetJacobians;
           
JcDqD = feetJdqd;
            
desiredFeetContactForces = pinvA*(HDotDes-grav);    

Minv = inv(M);
JcMinv        = Jc*Minv;
PInv_JcMinvSt = pinv(JcMinv*St, PINV_TOL);
N0            = eye(n) - PInv_JcMinvSt*JcMinv*St;

measuredFeetContactForces = desiredFeetContactForces;

tauForDesiredFeetForces = PInv_JcMinvSt*(JcMinv*h - JcDqD - JcMinv*Jc'*measuredFeetContactForces);
% 
% size(g(7:end))
% size(Jc(:,7:end)')
% size(measuredFeetContactForces)
% size(Impedances)
% size((q-qInit))
% 
% size((Impedances'.*(q-qInit)))

tauForImpedenceBehaviour = g(7:end)-Jc(:,7:end)'*measuredFeetContactForces - (Impedances'.*(q-qInit));

tau = tauForDesiredFeetForces + N0*tauForImpedenceBehaviour;

CoMError    = xcom - Desired_x_dx_ddx_CoM(:,1);
normCoMError  = norm(CoMError);


