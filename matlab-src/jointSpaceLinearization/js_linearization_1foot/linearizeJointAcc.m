%% linearizeJointAcc
%  it linearizes the joint space equations of motion when the robot is
%  controlled with the "Stack of Task" approach. The linearized system gives
%  informations on the local stability of the robot in joint space.
clear all
close all
clc

%% Path and model initialization
addpath('./../../../mex-wholebodymodel/matlab/utilities');
addpath('./../../../mex-wholebodymodel/matlab/wrappers');
addpath('./../../../../../build/');

wbm_modelInitialise('icubGazeboSim');

% this model is avaliable only for 1 foot on ground
feet_on_ground = [1 0];      % feet_on_ground(1) = left foot

%% Desired joints position
if     feet_on_ground(1) == 1 && feet_on_ground(2) == 0
    
leftLegInit  = [  25.5   15.0   0.0  -18.5  -5.5  0.0]';
rightLegInit = [  25.5   5.0    0.0  -40    -5.5  0.0]'; 

elseif feet_on_ground(2) == 1 && feet_on_ground(1) == 0
    
rightLegInit = [  25.5   15.0   0.0  -18.5  -5.5  0.0]';
leftLegInit  = [  25.5   5.0    0.0  -40    -5.5  0.0]'; 
    
end

torsoInit    = [ -10.0   0.0    0.0]';
leftArmInit  = [ -20     30     0.0   45     0.0]';          
rightArmInit = [ -20     30     0.0   45     0.0]';
 
qjDes        = [torsoInit; leftArmInit; rightArmInit; leftLegInit; rightLegInit]*(pi/180);

%% Update the robot state
ndof         = length(qjDes);

wbm_updateState(qjDes,zeros(ndof,1),zeros(6,1));

if     feet_on_ground(1) == 1 && feet_on_ground(2) == 0
    
[Rb,position] = wbm_getWorldFrameFromFixedLink('l_sole',qjDes);

elseif feet_on_ground(2) == 1 && feet_on_ground(1) == 0
    
[Rb,position] = wbm_getWorldFrameFromFixedLink('r_sole',qjDes);

end
    
%% Parameters definition
% feet jacobian
Jc           = wbm_jacobian(Rb,position,qjDes,'lsole');

% mass matrix
M            = wbm_massMatrix(Rb,position,qjDes);

% matrix A at CoM
x_lsole      = wbm_forwardKinematics(Rb,position,qjDes,'l_sole');
CoM          = wbm_forwardKinematics(Rb,position,qjDes,'com');
posLeftFoot  = x_lsole(1:3);
xCoM         = CoM(1:3);
r            = posLeftFoot-xCoM;

A            = [eye(3)   zeros(3);
                skew(r)  eye(3) ];
            
% constant values
toll         = 1e-8;
gBar         = 9.81;
m            = M(1,1);
e3           = zeros(ndof+6,1);
e3(3)        = 1;

S            = [zeros(6,ndof); eye(ndof)];
St           = transpose(S);
S6           = [eye(6) zeros(6,ndof)];
S6t          = transpose(S6);
fgrav        = [0; 0; -m*gBar; zeros(3,1)];

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
 
 NL          = eye(ndof) - pinvL*Lambda;
 
%% Gravity acceleration at joints in steady state
 vec         = Jc*Minv*Dbar*M*gBar*e3 + Jc*Minv*Dbar*Jct*invA*fgrav;
 R1          = pinvL*Jc*Minv*Dbar*Jct*invA;
 
 acc_steady  = pinvL*vec;
 
%% Gains definition
gainsPCoM         = diag([50 50 50]);
gainsDCoM         = diag([1  1  1]);
gainMomentum      = 1;

impTorso          = [20  20  20
                      0   0   0]; 

impArms           = [ 13  13   13   5   5
                       0    0    0   0   0 ];

impLeftLeg        = [ 70  70  65  30  10  10
                       0   0   0   0   0   0]; 
                         
impRightLeg       = [ 20  20  20  10  10  10
                       0   0   0   0   0   0];
                         
impedances        = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)]; 
dampings          =  0.5*ones(1,ndof);

%% Base frame orientation conversion to Euler angles
[T_bar,~]  = parametrization(Rb);

T_tilde    = [  eye(3)  zeros(3);
              zeros(3)   T_bar ];

% conversion matrix from angular velocity to Euler angles time derivative
T          = [    T_tilde           zeros(6,ndof);
              zeros(ndof,6)          eye(ndof)  ];

% CoM jacobian
JCoM_total =  wbm_jacobian(Rb, position, qjDes, 'com');
JCoM       =  JCoM_total(1:3,:);
JCoM_base  =  JCoM(:,1:6);
JCoM_qj    =  JCoM(:,7:end);

% angular momentum jacobian
Jw_base    = zeros(3,6);
Jw_qj      = zeros(3,ndof);

for ii = 1:6
    
Nu_base       = zeros(6,1);
Nu_base(ii)   = 1;

H             = wbm_centroidalMomentum(Rb, position, qjDes, zeros(ndof,1), Nu_base);

Jw_base(:,ii) = H(4:end);

end

for ii = 1:ndof

dqj         = zeros(ndof,1);
dqj(ii)     = 1;

H           = wbm_centroidalMomentum(Rb, position, qjDes, dqj, zeros(6,1));

Jw_qj(:,ii) = H(4:end);

end

% feet jacobian
Jc_base = Jc(1:6,1:6);
Jc_qj   = Jc(1:6,7:end);

% conversion term between Nu_base and dqj, obtained from contact
% constraints equations
Nu_baseFrom_dqj  = -(eye(6)/Jc_base)*Jc_qj;

%% Numerical derivative computation of a part of joint acceleration equation
% define parameters
linParam.delta       = 0.00001;
linParam.qjDes       = qjDes;
linParam.gBar        = gBar;
linParam.e3          = e3;
linParam.S           = S;
linParam.S6          = S6;
linParam.fgrav       = fgrav;
linParam.ndof        = ndof;
linParam.toll        = toll;
linParam.m           = m;

ddqj_noGains         = zeros(ndof);

for jj = 1:ndof
    
num_der              = numericalDerivative(jj,linParam);
ddqj_noGains(:,jj)   = num_der;

end

%% Analytical derivative with respect of joint position
JCoM_analitical    = JCoM*T;
JCoM_analitBase    = JCoM_analitical(1:3,1:6);
JCoM_analitJoint   = JCoM_analitical(1:3,7:end);

xCoM_posDerivative = JCoM_analitBase*(eye(6)/T_tilde)*Nu_baseFrom_dqj + JCoM_analitJoint;

HDot_posDerivative = [-m*gainsPCoM*xCoM_posDerivative; zeros(3,ndof)];

% partial derivative and linearized stiffness matrix
ddqj_posDerivative =  ddqj_noGains -R1*HDot_posDerivative -NL*diag(impedances);
KS                 = -ddqj_posDerivative;

%% Analytical derivative with respect of joint velocity
dxCoM_velDerivative = JCoM_base*Nu_baseFrom_dqj + JCoM_qj;
Hw_velDerivative    = Jw_base*Nu_baseFrom_dqj + Jw_qj;

HDot_velDerivative  = [-m*gainsDCoM*dxCoM_velDerivative; -gainMomentum*Hw_velDerivative];

% partial derivative and linearized damping matrix
ddqj_velderivative = -R1*HDot_velDerivative -NL*diag(dampings);
KD                 = -ddqj_velderivative;

%% Stability verify and visualization
% symmetrized stiffness matrix
KS_sym     = (KS+transpose(KS))/2;
eig_KS_sym =  eig(KS_sym);
flag       =  0;

for ii = 1:length(eig_KS_sym)
    
    if eig_KS_sym(ii) <= 0
        
    flag = 1;
        
    end
    
end

if flag == 1
    
  disp('the stiffness matrix is NOT positive definite')
        
else
        
  disp('the stiffness matrix is positive definite')
        
end      

% symmetrized damping matrix
KD_sym     = (KD+transpose(KD))/2;
eig_KD_sym =  eig(KD_sym);
flag       =  0;

for ii = 1:length(eig_KD_sym)
    
    if eig_KD_sym(ii) <= 0
        
    flag = 1;
     
    end
    
end

if flag == 1
  
 disp('the damping matrix is NOT positive definite')
        
else
        
 disp('the damping matrix is positive definite')
        
end      

% eigenvalues visualization
disp('eigenvalues from symmetrized KS and KD')
disp([eig_KS_sym eig_KD_sym])

%% Gains tuning with Kronecher product
% desired derivative for position
ddqjPosDer_desired = -ddqj_noGains -diag(impedances);

% parameters definition
R11 = -R1*[-m*eye(3);zeros(3)];
R12 = xCoM_posDerivative;

S11 = -NL;
S12 =  eye(ndof);

[Kp,~,Kimp] = KronecherProd(ddqjPosDer_desired, R11, R12, S11, S12, linParam, 'position');

% new stiffness matrix
KS_corr = -(ddqj_noGains -R1*[-m*Kp*xCoM_posDerivative; zeros(3,ndof)] -NL*Kimp);

% desired derivative for velocity
ddqjVelDer_desired = -diag(dampings);

%parameters definition

R21 = -R1;
R22 =  [dxCoM_velDerivative; Hw_velDerivative];

S21 = -NL;
S22 =  eye(ndof);

[Kd,Kw,Kdamp] = KronecherProd(ddqjVelDer_desired, R21, R22, S21, S22, linParam, 'velocity');

% new damping matrix
KD_corr = -(-R1*[-m*Kd*dxCoM_velDerivative; -Kw*Hw_velDerivative] -NL*Kdamp);

% KS_corr and KD_corr visualization
disp('eigenvalues from symmetrized KS_corr and KD_corr')
disp([eig((KS_corr+transpose(KS_corr))/2) eig((KD_corr+transpose(KD_corr))/2)])

