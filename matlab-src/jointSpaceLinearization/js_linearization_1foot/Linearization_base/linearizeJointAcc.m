%% linearizeJointAcc
%  linearizes the joint space equations of motion of robot iCub when it's
%  controlled with "stack of task" control approach. This version is for
%  the robot balancing on one foot. 
clear all
close all
clc

%% Path and model initialization
addpath('./../../../../mex-wholebodymodel/matlab/utilities');
addpath('./../../../../mex-wholebodymodel/matlab/wrappers');
addpath('./../../../../../../build/');

wbm_modelInitialise('icubGazeboSim');

% this model is avaliable only for 1 foot on ground
feet_on_ground = [1 0];       % feet_on_ground(1) = left foot

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
 
qj           = [torsoInit; leftArmInit; rightArmInit; leftLegInit; rightLegInit]*(pi/180);

%% Update the robot state
ndof         = length(qj);

wbm_updateState(qj,zeros(ndof,1),zeros(6,1));

if     feet_on_ground(1) == 1 && feet_on_ground(2) == 0
    
nameLink = 'l_sole';

elseif feet_on_ground(2) == 1 && feet_on_ground(1) == 0
    
nameLink = 'r_sole';

end

[Rb,position] = wbm_getWorldFrameFromFixedLink(nameLink,qj);
    
%% Parameters definition
toll         = 1e-10;

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
m            = M(1,1);

% other variables
invA         = eye(6)/A;
Mb           = M(1:6,1:6);
Mbj          = M(1:6,7:end);
Mjb          = M(7:end,1:6);
Mj           = M(7:end,7:end);

Jb           = Jc(1:6,1:6);
Jj           = Jc(1:6,7:end);

Mbar         = Mj - Mjb/Mb*Mbj;
Mbar_inv     = Mbar'/(Mbar*Mbar' + toll*eye(size(Mbar,1)));

%% Gains definition
% gainsPCoM         = diag([50 50 50]);
gainsPCoM         = diag([ 1 1 1]);
gainsDCoM         = sqrt(gainsPCoM);
gainMomentum      = 1;
                       
impTorso          = [20  20  20
                      0   0   0]; 

impArms           = [ 13  13   13   5   5
                       0   0    0   0   0 ];

impLeftLeg        = [ 70  70  65  30  10  10
                       0   0   0   0   0   0]; 
                         
impRightLeg       = [ 20  20  20  10  10  10
                       0   0   0   0   0   0];

impedances        = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)]; 
dampings          =  0.5*ones(ndof,1); 

%% Parameters from HDotDes

% CoM jacobian
JCoM_total =  wbm_jacobian(Rb, position, qj, 'com');
JCoM       =  JCoM_total(1:3,:);
JCoM_b     =  JCoM(:,1:6);
JCoM_j     =  JCoM(:,7:end);

% angular momentum jacobian
Jw_b       = zeros(3,6);
Jw_j       = zeros(3,ndof);

for ii = 1:6
    
Nu_base       = zeros(6,1);
Nu_base(ii)   = 1;

H             = wbm_centroidalMomentum(Rb, position, qj, zeros(ndof,1), Nu_base);

Jw_b(:,ii)    = H(4:end);

end

for ii = 1:ndof

dqj         = zeros(ndof,1);
dqj(ii)     = 1;

H           = wbm_centroidalMomentum(Rb, position, qj, dqj, zeros(6,1));

Jw_j(:,ii)  = H(4:end);

end

% conversion term between Nu_base and dqj, obtained from contact
% constraints equations
Nu_baseFrom_dqj  = -(eye(6)/Jb)*Jj;

%% Analytical derivative with respect of joint position
xCoM_posDerivative  = JCoM_b*Nu_baseFrom_dqj + JCoM_j;
angularOrientation  = zeros(3,ndof);

%%%% CLOSED LOOP %%%%

% angularOrientation  = -(Jw_b*Nu_baseFrom_dqj + Jw_j);

%%%%%%%%%%%%%%%%%%%%%

HDot_posDerivative = [-m*gainsPCoM*xCoM_posDerivative; angularOrientation];

%% Analytical derivative with respect of joint velocity
dxCoM_velDerivative = JCoM_b*Nu_baseFrom_dqj + JCoM_j;
Hw_velDerivative    = Jw_b*Nu_baseFrom_dqj + Jw_j;

HDot_velDerivative  = [-m*gainsDCoM*dxCoM_velDerivative; -gainMomentum*Hw_velDerivative];
 
%% Other terms 
B2      =  (Jj - Jb/Mb*Mbj)*Mbar_inv;
B3      =  Jb/Mb*transpose(Jb)*invA;
pinvB2  =  pinv(B2,toll);

B       =  pinvB2*B3;
Nb      =  eye(ndof) - pinvB2*B2; 

%% Stiffness
KS      =  B*HDot_posDerivative + Nb*diag(impedances);

%% Damping
KD      =  B*HDot_velDerivative + Nb*diag(dampings);

%% if you want to add Mbar:
KS      = Mbar_inv*KS;
KD      = Mbar_inv*KD;

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
% disp('eigenvalues: KSsym, KDsym, KS, KD')
% disp([eig_KS_sym eig_KD_sym eig(KS) eig(KD)])

%% State matrix verification
A_state = [zeros(ndof) eye(ndof);
             -KS          -KD];

disp('eigenvalues of the state matrix')
disp(eig(A_state))


