function ddqj_noGains = steadyAccCalculation(Rb, position, qj, linParam)
%% steadyAccCalculation
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
fgrav       = linParam.fgrav;
ndof        = linParam.ndof;
toll        = linParam.toll;

%% Define all the necessary terms
% feet jacobian
Jc           = wbm_jacobian(Rb,position,qj,'lsole');

% mass matrix
M            = wbm_massMatrix(Rb,position,qj);

% matrix A at CoM
x_lsole      = wbm_forwardKinematics(Rb,position,qj,'l_sole');
CoM          = wbm_forwardKinematics(Rb,position,qj,'com');
posLeftFoot  = x_lsole(1:3);
xCoM         = CoM(1:3);
r            = posLeftFoot-xCoM;

A            = [eye(3)   zeros(3);
                skew(r)  eye(3) ];
            
% constant values
m            = M(1,1);
St           = transpose(S);
S6t          = transpose(S6);

% other variables
Minv         = eye(ndof+6)/M;
invA         = eye(6)/A;
Jct          = transpose(Jc);

invS6MS6t    = eye(6)/(S6*M*S6t);
D            = St - (St*M*S6t)*invS6MS6t*S6;
Dbar         = eye(ndof+6) - S*D;
Lambda       = Jc*Minv*S;

%pinvL       = pinv(Lambda,toll);
 pinvL       = Lambda'/(Lambda*Lambda' + toll*eye(size(Lambda,1)));
 
%% Gravity acceleration at joints in steady state
 vec         = Jc*Minv*Dbar*M*gBar*e3 + Jc*Minv*Dbar*Jct*invA*fgrav;
 
ddqj_noGains = pinvL*vec;

end
