function [tau,CoMError,xDDcomStar] = controller_BalancingTorque(q,qD,  IntErrorCoM,prm)
%#codegen

t = prm.current_time;
Desired_x_dx_ddx_CoM = prm.Desired_x_dx_ddx_CoM(t);
M           = prm.MassMatrix;
h           = prm.Coriolis;
g           = prm.Gravitation;
H           = prm.CentroidalMom;
PosRightFoot= prm.fkin.rightfoot(1:3);
feetJacobians=prm.jacobian.feet;
feetJdqd    = prm.jdqd.feet;
xcom        = prm.fkin.com(1:3);
Jcom        = prm.jacobian.com;
Gains       = prm.Gains;
Impedances  = prm.Impedances;
qInit       = prm.qjInit;

PINV_TOL = 1e-5;
gravAcc        = 9.81;

n = 25;
St    = [zeros(6,n);eye(n,n)];

m     = M(1,1);
grav = [-m*gravAcc;zeros(2,1);zeros(3,1)];


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

JcMinv        = prm.JMinvconst ;
PInv_JcMinvSt = pinv(JcMinv*St, PINV_TOL);
N0            = eye(n) - PInv_JcMinvSt*JcMinv*St;




measuredFeetContactForces = desiredFeetContactForces;

% tauForDesiredFeetForces = PInv_JcMinvSt*(JcMinv*h - JcDqD - JcMinv*Jc'*measuredFeetContactForces);
tauForDesiredFeetForces = PInv_JcMinvSt*(JcMinv*h - JcDqD - prm.JMinvJt*measuredFeetContactForces);




tauForImpedenceBehaviour = g(7:end)-Jc(:,7:end)'*measuredFeetContactForces - (Impedances'.*(q-qInit));

tau = tauForDesiredFeetForces + N0*tauForImpedenceBehaviour;

CoMError    = xcom - Desired_x_dx_ddx_CoM(:,1);


normCoMError  = norm(CoMError);


