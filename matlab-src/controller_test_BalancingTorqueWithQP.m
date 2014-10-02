function [tau,CoMError,xDDcomStar] = controller_test_BalancingTorqueWithQP(q,v,m,h,g,H,Jc,JcMinv,JcMinvJct,JcDqD,J_CoM,pos_rightFoot,pos_CoM,IntErrorCoM,qjInit,Gains,Impedances,n_dof,n_constraint,t,prm)


% t                       = prm.current_time;
Desired_x_dx_ddx_CoM    = prm.Desired_x_dx_ddx_CoM(t);
% n_constraint            = prm.n_constraint;
% n_dof                   = prm.n_dof;

% pos_rightFoot            = prm.fkin.rightfoot(1:3);
% Jc                      = prm.jacobian.feet;
% JcDqD                  = prm.jdqd.feet;
% pos_CoM                    = prm.fkin.com(1:3);
% Jcom                    = prm.jacobian.com;
% Gains                   = prm.Gains;
% Impedances              = prm.Impedances;


PINV_TOL                = 1e-5;
gravAcc                 = 9.81;


St    = [zeros(6,n_dof);eye(n_dof,n_dof)];

grav = [-m*gravAcc;zeros(2,1);zeros(3,1)];

xDcom      = J_CoM(1:3,:)*v;
xDDcomStar = Desired_x_dx_ddx_CoM(:,3) - Gains(1)*(pos_CoM - Desired_x_dx_ddx_CoM(:,1)) - Gains(2)*IntErrorCoM - Gains(3)*(xDcom - Desired_x_dx_ddx_CoM(:,2));

Pr =  pos_rightFoot - pos_CoM; % Application point of the contact force on the right foot w.r.t. CoM

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

% % this line is to be replaced by QP
% desiredFeetContactForces = pinvA*(HDotDes-grav);

%% QP implementation
options = optimset('Algorithm','active-set','Display','off');
% options = optimset('Algorithm','interior-point-convex','Display','off');

% INEQUALITY CONSTRAINTS 

% friction cone
staticFrictionCoefficient = 0.45;
numberOfPoints = 4; %number of points in a quadrant for cone

[Aineq,bineq] = constraint_fcone(staticFrictionCoefficient,numberOfPoints,n_constraint);


% Aineq = Aineq(1:6,1:6);
% bineq = bineq(1:6);

% add constraints on moments
% bineq = [bineq;0;0;0;0];
% 
% Aineq = [Aineq;
%         -0.1,0,0,0,1,0,zeros(1,6);
%         -0.1,0,0,0,-1,0,zeros(1,6);
%         -0.1,0,0,0,0,1,zeros(1,6)
%         -0.1,0,0,0,0,-1,zeros(1,6)];

% end INEQUALITY CONSTRAINTS

% for torque minimization
% Ht1 = PInv_JcMinvSt*(-prm.JcMinvJct+JcMinv*St*(Jc(:,7:end)'))-eye(25)*Jc(:,7:end)';
Ht1 = PInv_JcMinvSt*(-JcMinvJct) - N0*Jc(:,7:end)';

alpha_qptau = 0;
Ht2 = PInv_JcMinvSt*(JcMinv*h-JcDqD)+N0*(g(7:end) - (Impedances'.*(q-qjInit)));


% EQUALITY CONSTRAINTS

% Aeq = JcMinv*(St*Ht1+transpose(Jc));
% Aeq = [Aeq;-Aeq];
% beq = -JcDqD -JcMinv*(St*Ht2-h);
% beq = [beq+1e-10;-beq+1e-10];
% 
% Aineq = [Aineq;Aeq];
% bineq = [bineq;beq];
% end EQUALITY CONSTRAINTS

lb = -200 * ones(6*n_constraint,1);

lb(1) = 0; %fx (vertical)
if n_constraint==2
    lb(7) = 0; %fx (vertical)
end
ub = [];

x0 = pinvA*(HDotDes-grav);%[];
% ub = 1e+4 * ones(12, 1);



% quadraticTerm = transpose(A)*A+eye(12)*1e-8;
% linearTerm = transpose(A)*(grav-HDotDes);

% torque minimization added (along with equality constraint)
quadraticTerm = transpose(A)*A + alpha_qptau*(transpose(Ht1)*Ht1)+eye(6*n_constraint)*1e-8;
linearTerm = transpose(A)*(grav-HDotDes)+alpha_qptau*(transpose(Ht1)*Ht2);

% eig(quadraticTerm)

% eig(quadraticTerm)

[desiredFeetContactForces, objVal, exitFlag, output, lambda] = ... 
...% [desiredFeetContactForces, ~, ~, ~, ~] = ...
quadprog(quadraticTerm, linearTerm, ...
          Aineq, bineq, ... %inequalities
          [], [], ... %equalities
          lb, ub, ... %bounds
          x0,     ... %initial solution
          options);
% clc
% objVal+0.5*(grav-HDotDes)'*(grav-HDotDes)
% exitFlag
if exitFlag~=1
    disp('qp sicti');
end
% output
% [x0 , desiredFeetContactForces]
%%
% desiredFeetContactForces = pinvA*(HDotDes-grav); % to ignore QP
measuredFeetContactForces = desiredFeetContactForces;

tauForDesiredFeetForces = PInv_JcMinvSt*(JcMinv*h - JcDqD - JcMinvJct*measuredFeetContactForces);
tauForImpedenceBehaviour = g(7:end)-Jc(:,7:end)'*measuredFeetContactForces - (Impedances'.*(q-qjInit));

tau = tauForDesiredFeetForces + N0*tauForImpedenceBehaviour;

CoMError    = pos_CoM - Desired_x_dx_ddx_CoM(:,1);


normCoMError  = norm(CoMError);
end

function [Aineq,bineq]=constraint_fcone(staticFrictionCoefficient,numberOfPoints,n_constraint)
%compute friction cones contraints

%approximation with straight lines

%split the pi/2 angle into numberOfPoints - 1;
segmentAngle = pi/2 / (numberOfPoints - 1);

%define angle
angle = 0 : segmentAngle : (2 * pi - segmentAngle);
points = [cos(angle); sin(angle)];
numberOfEquations = size(points, 2);
assert(size(points, 2) == (4 * (numberOfPoints - 2) + 4));

%Aineq*x <= b, with b is all zeros.
Aineq = zeros(numberOfEquations, 6);

%define equations
for i = 1 : numberOfEquations
   firstPoint = points(:, i);
   secondPoint = points(:, rem(i, numberOfEquations) + 1);
   
   %define line passing through the above points
   angularCoefficients = (secondPoint(2) - firstPoint(2)) / (secondPoint(1) - firstPoint(1));
   
   offsets = firstPoint(2) - angularCoefficients * firstPoint(1);

   inequalityFactor = +1;
   %if any of the two points are between pi and 2pi, then the inequality is
   %in the form of y >= m*x + q, and I need to change the sign of it.
   if (angle(i) > pi || angle(rem(i, numberOfEquations) + 1) > pi)
       inequalityFactor = -1;
   end
   
   %a force is 6 dimensional f = [fx, fy, fz, mux, muy, muz]'
   %I have constraints on fy and fz, and the offset will be multiplied by
   %mu * fx
   
   Aineq(i,:) = inequalityFactor .* [-offsets * staticFrictionCoefficient,-angularCoefficients, 1, 0, 0, 0];  
   
end

switch n_constraint
    case 1
        bineq = zeros(size(Aineq,1), 1);
    case 2    
        %duplicate the matrices and vector for the two feet
        Aineq = [Aineq, zeros(size(Aineq));
                zeros(size(Aineq)), Aineq];
          
        bineq = zeros(size(Aineq,1), 1);
    otherwise
end

end
