function  [tau_ol,f_c, cVisualParam] = balController(t,param,controlParam)

% this is the main function for the balancing controller. it contains the definition of every
% parameters necessary for the controller and calls the other functions
% (gains, constraints, ecc...) whose are organized as in simulink code

%% user-defined variables
% USE_QP_SOLVER               = param.QP_solver;
DEMO_LEFT_AND_RIGHT         = param.demo_left_and_right;  
LEFT_RIGHT_FOOT_IN_CONTACT  = param.numConstraints;

%% initial variables
DOF           = param.ndof;
q             = controlParam.qj;

amp           = 0.25*pi/180;
freq          = 0.1;
qDes          = param.qjInit + amp*sin(2*pi*freq*t);

qDes2 = param.qjInit;

v             = controlParam.v;

M             = controlParam.M;
h             = controlParam.h;
% H             = controlParam.H;

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

%% gains definition
[gainsPCOM, gainsDCOM, ~ , impedances_ini, dampings, referenceParams, directionOfOscillation, noOscillationTime,...
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
[~,~]      = constraints (forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,footSize,fZmin);

%% balancing with noQpcontroller
[errorCoM, f_c, tau_ol,f0,tau2]   =  controllerFCN (LEFT_RIGHT_FOOT_IN_CONTACT, DOF, ...
                                            q, qDes, v, M, h, posLeftFoot, posRightFoot, Jc, dJcDv, xcom, J_CoM, desired_x_dx_ddx_CoM,...
                                            gainsPCOM, gainsDCOM, impedances, dampings, pos_feet, lfoot_ini, rfoot_ini,xcomDes,qDes2);

%%  parameters for the visualization
cVisualParam.f0    = f0;
cVisualParam.e_com = errorCoM;
cVisualParam.tau2  = tau2;

end
