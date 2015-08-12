function  [tau,fc, cVisualParam] = balController(t,param,controlParam)

%this is the main function for the balancing controller. it contains the definition of every
%parameters necessary for the controller and calls the other functions
%(gains, constraints, ecc...) which are the same that in simulink code

%% user-defined variables
USE_QP_SOLVER               = param.QP_solver;
USE_HD_SOLVER               = param.HDes_solver;
DEMO_LEFT_AND_RIGHT         = param.demo_left_and_right;  

if     param.numConstraints == 2

   LEFT_RIGHT_FOOT_IN_CONTACT  = [1 1];
    
elseif param.numConstraints == 1
    
   LEFT_RIGHT_FOOT_IN_CONTACT  = [1 0];
    
end

%% initial variables
DOF  = param.ndof;
q    = controlParam.qj;
qDes = param.qjDes;
v    = controlParam.v;

M    = controlParam.M;
h    = controlParam.h;
H    = controlParam.H;

Jc    = controlParam.Jc;
dJcDv = controlParam.dJcDq;
J_CoM = controlParam.Jcom;

pos_feet     = controlParam.pos_feet;
posLeftFoot  = controlParam.x_lsole;
posRightFoot = controlParam.x_rsole;

xcom         = controlParam.com(1:3);
xcomDes      = param.com(1:3);

%% gains definition
[gainsPCOM, gainsICOM, gainsDCOM, gainMomentum, impedances_ini, dampings,referenceParams,directionOfOscillation,noOscillationTime,...
 forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,...
 footSize,fZmin, increasingRatesImp, qTildeMax] = gains (DOF, LEFT_RIGHT_FOOT_IN_CONTACT,DEMO_LEFT_AND_RIGHT);

%% com trajectory generator
desired_x_dx_ddx_CoM = generTraj (xcomDes,t,referenceParams,directionOfOscillation,noOscillationTime);

%% stability test
% this creates a perturbation on the desired position of center of mass
%  const     = 5;
%  amplitude = 0.05;
%  T=1;
%  if t>const
%  desired_x_dx_ddx_CoM(1,1)   = desired_x_dx_ddx_CoM(1,1)+amplitude*(exp(-(t-const)/T));
%  end
 
%% correction of impedances with a non linear part
qMin=0;
qMax=0;
impedances = nonLinImp (qDes,q,qMin,qMax,impedances_ini,increasingRatesImp,qTildeMax);

%% constraints at foot
[ConstraintsMatrix,bVectorConstraints] = constraints (forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,footSize,fZmin);

%% balancing with noQpcontroller
% other parameters
intErrorCoM   = 0*xcom;
ki_int_qtilde = zeros(DOF,1);
constraint    = LEFT_RIGHT_FOOT_IN_CONTACT;

[tauModel,Sigma,NA,fHdotDesC1C2, ...
 HessianMatrixQP1Foot,gradientQP1Foot,ConstraintsMatrixQP1Foot,bVectorConstraintsQp1Foot,...
 HessianMatrixQP2Feet,gradientQP2Feet,ConstraintsMatrixQP2Feet,bVectorConstraintsQp2Feet,...
 ~,~,f0]   =  ...
 controllerFCN   (constraint,DOF,ConstraintsMatrix,bVectorConstraints,...
                  q,qDes,v, M, h, H, posLeftFoot, posRightFoot,footSize, Jc, dJcDv, xcom, J_CoM, desired_x_dx_ddx_CoM,...
                  gainsPCOM, gainsICOM, gainsDCOM, gainMomentum, impedances, dampings, intErrorCoM, ki_int_qtilde,USE_HD_SOLVER,pos_feet);      
      
%% balancing with Qpcontroller 
 if USE_QP_SOLVER ==1 && sum(LEFT_RIGHT_FOOT_IN_CONTACT)==2
     
   f0_qp_2_feet = QP2FEET(HessianMatrixQP2Feet,gradientQP2Feet,...
                          ConstraintsMatrixQP2Feet,bVectorConstraintsQp2Feet);
   f0=f0_qp_2_feet;
                   
 elseif USE_QP_SOLVER ==1 && sum(LEFT_RIGHT_FOOT_IN_CONTACT)==1
     
   f0_qp_1_foot = QP1FOOT(HessianMatrixQP1Foot,gradientQP1Foot,...
                          ConstraintsMatrixQP1Foot,bVectorConstraintsQp1Foot);
   fc_1f=f0_qp_1_foot;
   
 end

%% HD controller correction
% this part is necessary to select the correct solution of angular
% momentum reference and to add a smoothing function in the controller
 TT = 1.5;
 
 if USE_HD_SOLVER==1 && sum(LEFT_RIGHT_FOOT_IN_CONTACT)==2
     
     f0(13:end) = -f0(13:end)*(1-exp(-t/TT));
       
 elseif USE_HD_SOLVER==0 && sum(LEFT_RIGHT_FOOT_IN_CONTACT)==2
     
     f0(13:end) = [0;0;0];
     
 elseif USE_HD_SOLVER==1 && sum(LEFT_RIGHT_FOOT_IN_CONTACT)==1
     
     f0(7:end)  = -f0(7:end)*(1-exp(-t/TT));
     
 end
  
%% calculating tau and fc
% corrections for only one foot on the ground. these correction are
% necessary to switch the balancing form one foot to 2 feet, otherwise
% there are dimensional errors in some matrices in "controllerFCN" function.
if sum(LEFT_RIGHT_FOOT_IN_CONTACT)==1 && USE_QP_SOLVER ==0
    
  fc  = fHdotDesC1C2 +NA*f0; 
  
elseif sum(LEFT_RIGHT_FOOT_IN_CONTACT)==1 && USE_QP_SOLVER ==1

  fc  = fc_1f +NA*f0; 

elseif sum(LEFT_RIGHT_FOOT_IN_CONTACT)==2

  fc  = fHdotDesC1C2 +NA*f0; 

end

tau = tauModel + Sigma*fc;

%%  parameters for the visualization
cVisualParam.f0    = f0;
cVisualParam.e_com = xcom-desired_x_dx_ddx_CoM(:,1);

