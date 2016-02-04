function accParam = jointAccCalculation(Rb, position, qj, linParam)
%% jointAccCalculation
%  it calculates from a given states the terms in joint space acceleration which don't depend
%  from postural or CoM gains. The output is:
%
%  ddqj_noGains [ndof x 1] which is a part of joint acceleration equation
%
%% Parameters definition
gBar        = linParam.gBar;
e3          = linParam.e3;
S           = linParam.S;
S6          = linParam.S6;
ndof        = linParam.ndof;
toll        = linParam.toll;
nameLink    = linParam.nameLink;

%% Define all the necessary terms
% feet jacobian
Jc           = wbm_jacobian(Rb,position,qj,nameLink);

% mass matrix
M            = wbm_massMatrix(Rb,position,qj);

% matrix A at CoM
x_sole       = wbm_forwardKinematics(Rb,position,qj,nameLink);
CoM          = wbm_forwardKinematics(Rb,position,qj,'com');
posFoot      = x_sole(1:3);
xCoM         = CoM(1:3);
r            = posFoot-xCoM;

A            = [eye(3)   zeros(3);
                skew(r)  eye(3) ];
            
% constant values
St           = transpose(S);
S6t          = transpose(S6);
m            = M(1,1);
fgrav        = [0; 0; -m*gBar; zeros(3,1)];

% other variables
Minv         = eye(ndof+6)/M;
Mbar         = M(7:end,7:end) - M(7:end,1:6)/M(1:6,1:6)*M(1:6,7:end); 
Mbar_inv     = Mbar'/(Mbar*Mbar' + toll*eye(size(Mbar,1)));

invA         = eye(6)/A;
Jct          = transpose(Jc);

invS6MS6t    = eye(6)/(S6*M*S6t);
D            = St - (St*M*S6t)*invS6MS6t*S6;
Dbar         = eye(ndof+6) - S*D;
Lambda       = Jc*Minv*S;

%pinvL       = pinv(Lambda,toll);
 pinvL       = Lambda'/(Lambda*Lambda' + toll*eye(size(Lambda,1)));
 
 NL          = eye(ndof) - pinvL*Lambda;
 
%% Gravity acceleration at joints in steady state
 vec                  = Jc*Minv*Dbar*M*gBar*e3 + Jc*Minv*Dbar*Jct*invA*fgrav;
 R1                   = pinvL*Jc*Minv*Dbar*Jct*invA;
 pinvL                = Mbar_inv*pinvL;

 accParam.NL          = Mbar_inv*NL;
 accParam.R1          = Mbar_inv*R1;
 accParam.m           = m;
 accParam.Jc          = Jc;
 accParam.acc_steady  = pinvL*vec;
 
end
