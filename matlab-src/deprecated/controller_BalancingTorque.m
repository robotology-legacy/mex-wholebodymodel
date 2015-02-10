% function [tau,CoMError,xDDcomStar] = controller_BalancingTorque(q,v,  IntErrorCoM,prm)
function [tau,CoMError,xDDcomStar] = controller_BalancingTorque(qj,v,m,h,g,H,Jc,JcMinv,JcMinvJct,JcDqD,J_CoM,pos_rightFoot,pos_CoM,IntErrorCoM,qjInit,Gains,Impedances,n_dof,n_constraint,t,prm)

Desired_x_dx_ddx_CoM = prm.Desired_x_dx_ddx_CoM(t);

PINV_TOL = 1e-5;

gravAcc        = 9.81;

St    = [zeros(6,n_dof);eye(n_dof,n_dof)];

grav = [-m*gravAcc;zeros(2,1);zeros(3,1)];

xDcom      = J_CoM(1:3,:)*v;

xDDcomStar = Desired_x_dx_ddx_CoM(:,3) - Gains(1)*(pos_CoM - Desired_x_dx_ddx_CoM(:,1)) - Gains(2)*IntErrorCoM - Gains(3)*(xDcom - Desired_x_dx_ddx_CoM(:,2));

Pr =  pos_rightFoot(1:3) - pos_CoM; % Application point of the contact force on the right foot w.r.t. CoM

switch n_constraint
    case 1 % on left foot
        A  = [ eye(3),   zeros(3);
             -skew(pos_CoM),  eye(3)];
    case 2 % on both feet
        A  = [ eye(3),   zeros(3),eye(3), zeros(3);
             -skew(pos_CoM),  eye(3), skew(Pr), eye(3) ];
    otherwise
        disp('Choose number of constraints properly (1 or 2)');
        return
end   
        
pinvA = pinv(A, PINV_TOL);
  
HDotDes  = [ m*xDDcomStar ;
            -Gains(4)*H(4:end)]; 

PInv_JcMinvSt = pinv(JcMinv*St, PINV_TOL);

N0            = eye(n_dof) - PInv_JcMinvSt*JcMinv*St;

desiredFeetContactForces = pinvA*(HDotDes-grav);

measuredFeetContactForces = desiredFeetContactForces;

tauForDesiredFeetForces = PInv_JcMinvSt*(JcMinv*h - JcDqD - JcMinvJct*measuredFeetContactForces);

tauForImpedenceBehaviour = g(7:end)-Jc(:,7:end)'*measuredFeetContactForces - (Impedances'.*(qj-qjInit));

tau = tauForDesiredFeetForces + N0*tauForImpedenceBehaviour;

CoMError    = pos_CoM - Desired_x_dx_ddx_CoM(:,1);

% normCoMError  = norm(CoMError);

