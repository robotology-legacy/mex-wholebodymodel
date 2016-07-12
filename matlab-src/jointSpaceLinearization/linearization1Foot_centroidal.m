%% linearization1Foot_centroidal
%  linearizes the joint space equations of motion of robot iCub when it's
%  controlled with "stack of task" control approach. This version is for
%  the robot balancing on one foot, and the equations of motion are espressed 
%  in centroidal coordinates.
%
clear all
close all
clc

%% Model initialization
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

% CoM jacobian
JCoM_total   = wbm_jacobian(Rb, position, qj, 'com');

% momentum jacobian
Jw_b         = zeros(6,6);
Jw_j         = zeros(6,ndof);

for ii = 1:6
    
Nu_base       = zeros(6,1);
Nu_base(ii)   = 1;

H             = wbm_centroidalMomentum(Rb, position, qj, zeros(ndof,1), Nu_base);

Jw_b(:,ii)    = H;

end

for ii = 1:ndof

dqj          = zeros(ndof,1);
dqj(ii)      = 1;

H            = wbm_centroidalMomentum(Rb, position, qj, dqj, zeros(6,1));

Jw_j(:,ii)   = H;

end

Jw           =  [Jw_b Jw_j];

% constant values
m            = M(1,1);
h            = wbm_generalisedBiasForces(Rb,position,qj,zeros(ndof,1),zeros(6,1));
g            = h;
CoM          = wbm_forwardKinematics(Rb,position,qj,'com');
xCoM         = CoM(1:3);

%% Centroidal conversion
[Tr, dT]           = centroidalTransformationT_TDot(xCoM,position,zeros(3,1),zeros(3,1),M);
[M_c,~,~,Jc_c,~,~] = fromFloatingToCentroidalDynamics(M, h, g, Jc, zeros(6,1), zeros(ndof+6,1), Tr, dT);
[~,~,~,Jw_c,~,~]   = fromFloatingToCentroidalDynamics(M, h, g, Jw, zeros(6,1), zeros(ndof+6,1), Tr, dT);
[~,~,~,JCoM_c,~,~] = fromFloatingToCentroidalDynamics(M, h, g, JCoM_total, zeros(6,1), zeros(ndof+6,1), Tr, dT);

%% Centroidal switching
M          = M_c;
Jc         = Jc_c;
Jw         = Jw_c;
JCoM_total = JCoM_c;

% other variables
Mb            = M(1:6,1:6);
Mj            = M(7:end,7:end);

Jb            = Jc(1:6,1:6);
Jj            = Jc(1:6,7:end);

Mbar          = Mj;

%Mbar_inv     = eye(ndof)/Mj;
 Mbar_inv     = Mbar'/(Mbar*Mbar' + toll*eye(size(Mbar,1)));

%% Other terms 
B2      =  Jj*Mbar_inv;
B3      =  Jb/Mb;
pinvB2  =  pinv(B2,toll);

B       =  pinvB2*B3;
Nb      =  eye(ndof) - pinvB2*B2; 

%% Gains definition
gainsPCoM         = diag([40 45 40]);
gainsDCoM         = sqrt(gainsPCoM);
gainPhi           = 5;
gainMomentum      = 2*sqrt(gainPhi);
                       
impTorso          = [20  20  20
                      0   0   0]; 

impArms           = [ 15  15   15   5   5
                       0   0    0   0   0 ];

impLeftLeg        = [ 70  70  65  30  10  10
                       0   0   0   0   0   0]; 
                         
impRightLeg       = [ 20  20  20  10  10  10
                       0   0   0   0   0   0];

impedances        = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)]; 
dampings          = 0.5*ones(ndof,1); 

%% Parameters from HDotDes

% CoM jacobian
JCoM       =  JCoM_total(1:3,:);
JCoM_b     =  JCoM(:,1:6);
JCoM_j     =  JCoM(:,7:end);

Jw_b       =  Jw(:,1:6);
Jw_j       =  Jw(:,7:end);

% conversion term between Nu_base and dqj, obtained from contact
% constraints equations
Nu_baseFrom_dqj     = -(eye(6)/Jb)*Jj;

%% Analytical derivative with respect of joint position

xCoM_posDerivative  = JCoM_b*Nu_baseFrom_dqj + JCoM_j;

%%%% CENTROIDAL ORIENTATION %%%%

% open loop
% angularOrientation  = zeros(3,ndof);
% Kimp                = diag(impedances); 
% Kdamp               = diag(dampings);

% closed loop and new postural task
angularOrientation  = -(Jw_b(4:6,:)*Nu_baseFrom_dqj+Jw_j(4:6,:));
Kimp                =  diag(impedances)*Nb*Mbar; 
Kdamp               =  diag(dampings)*Nb*Mbar;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

HDot_posDerivative  = [-m*gainsPCoM*xCoM_posDerivative; gainPhi*angularOrientation];

%% Analytical derivative with respect of joint velocity
HDot_velDerivative  = -[gainsDCoM zeros(3); zeros(3) gainMomentum*eye(3)]*(Jw_b*Nu_baseFrom_dqj + Jw_j);

%% Stiffness
KS      =  Mbar_inv*(B*HDot_posDerivative + Nb*Kimp);

%% Damping
KD      =  Mbar_inv*(B*HDot_velDerivative + Nb*Kdamp);

%% State matrix verification
A_state = [zeros(ndof) eye(ndof);
             -KS          -KD];

disp('eigenvalues of the state matrix')
disp(eig(A_state))