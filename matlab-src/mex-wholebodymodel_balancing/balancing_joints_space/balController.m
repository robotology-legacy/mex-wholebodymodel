function  [tau_c, f_c, cVisualParam] = balController(t,param,controlParam)

% this is the main function for the balancing controller. it contains the definition of every
% parameters necessary for the controller and calls the other functions
% (gains, constraints, ecc...) whose are organized as in simulink code

%% user-defined variables
USE_QP_SOLVER               = param.QP_solver;
DEMO_LEFT_AND_RIGHT         = param.demo_left_and_right;  
LEFT_RIGHT_FOOT_IN_CONTACT  = param.numConstraints;

%% initial variables
DOF           = param.ndof;
q             = controlParam.qj;
qDes          = param.qjInit + 0.75*pi/180*sin(2*pi*0.35*t) ;

v             = controlParam.v;

M             = controlParam.M;
h             = controlParam.h;
H             = controlParam.H;

Jc            = controlParam.Jc;
dJcDv         = controlParam.dJcDq;
J_CoM         = controlParam.Jcom;

pos_feet      = controlParam.pos_feet;
posLeftFoot   = controlParam.lsole;
posRightFoot  = controlParam.rsole;

xcom          = controlParam.com(1:3);
xcomDes       = param.com_ini(1:3);

lfoot_ini     = param.lfoot_ini;
rfoot_ini     = param.rfoot_ini;

gen = controlParam.gen;

%% gains definition
[gainsPCOM, gainsDCOM, gainMomentum, impedances_ini, dampings, referenceParams, directionOfOscillation, noOscillationTime,...
 forceFrictionCoefficient, numberOfPoints, torsionalFrictionCoefficient,...
 footSize, fZmin, increasingRatesImp, qTildeMax] = gains (DOF,LEFT_RIGHT_FOOT_IN_CONTACT,DEMO_LEFT_AND_RIGHT);

%% com trajectory generator
desired_x_dx_ddx_CoM = generTraj (xcomDes,t,referenceParams,directionOfOscillation,noOscillationTime);

%% stability test
% this creates a perturbation on the desired position of center of mass
% const     = 2.5;
% amplitude = 0.035;
% T=1;
% if t>const
% desired_x_dx_ddx_CoM(1,1)   = desired_x_dx_ddx_CoM(1,1) + amplitude*(exp(-(t-const)/T));
% end
 
%% correction of impedances with a non linear part
qMin=0;
qMax=0;
impedances = nonLinImp (qDes,q,qMin,qMax,impedances_ini,increasingRatesImp,qTildeMax);

%% constraints at foot
[ConstraintsMatrix,bVectorConstraints] = constraints (forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,footSize,fZmin);

%% balancing with noQpcontroller
qDes2 = controlParam.qDes2 ;

[tauModel,Sigma,NA,fHdotDesC1C2,errorCoM,f0,tau_c, f_c, dqj_des]   =  ...
 controllerFCN   (LEFT_RIGHT_FOOT_IN_CONTACT,DOF,USE_QP_SOLVER,ConstraintsMatrix,bVectorConstraints,...
                  q,qDes,v, M, h, H, posLeftFoot, posRightFoot,footSize, Jc, dJcDv, xcom, J_CoM, desired_x_dx_ddx_CoM,...
                  gainsPCOM, gainsDCOM, gainMomentum, impedances, dampings, pos_feet, lfoot_ini, rfoot_ini,qDes2,gen);      
  
cVisualParam.dqj_des = dqj_des;
              
%% calculating tau and fc
% fc  = fHdotDesC1C2 + NA*f0;
% 
% tau = tauModel + Sigma*fc;

%%  parameters for the visualization
cVisualParam.f0    = f0;
cVisualParam.e_com = errorCoM;

end
