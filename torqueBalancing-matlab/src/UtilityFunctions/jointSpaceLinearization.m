function linearization = jointSpaceLinearization(CONFIG,qjConfig,mode)    
%JOINTSPACELINEARIZATION linearizes the joint space dynamics of the iCub robot
%                        around an equilibrium point.
%   JOINTSPACELINEARIZATION assumes that the robot is balancing on one foot
%   or two feet, and the feedback controller is momentum-based. The solution is
%   analytical, i.e. it is not necessary to compute numerical derivatives. 
%     
%   linearization = JOINTSPACELINEARIZATION(config,qjConfig,mode) takes as an input the 
%   robot dynamics through the structure CONFIG, and the joint configuration 
%   around which the system is linearized, qjConfig. The variable MODE is a
%   string, and can be either 'normal' mode or 'stability'. The only
%   difference is that the 'stability' mode verifies the eigenvalues of the
%   state matrix and returns a message about system's stability properties.
%   It returns the structure LINEARIZATION which contains all the parameters
%   related to the linearized system (i.e. the stiffness and damping
%   matrix).
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
%% Config parameters
gainsInit          = CONFIG.gainsInit;
pinv_tol           = CONFIG.pinv_tol;
feet_on_ground     = CONFIG.feet_on_ground;     
ndof               = CONFIG.ndof;
pinv_damp          = CONFIG.pinv_damp;

%% Initialize the robot configuration
if  feet_on_ground(1) == 1

    [RotBase,PosBase] = wbm_getWorldFrameFromFixedLink('l_sole',qjConfig);
else
    [RotBase,PosBase] = wbm_getWorldFrameFromFixedLink('r_sole',qjConfig);    
end

wbm_setWorldFrame(RotBase,PosBase,[0 0 -9.81]')

[~,BasePose,~,~]   = wbm_getState();
chi                = [BasePose; qjConfig; zeros(6,1); zeros(ndof,1)];

% robot state, dynamics and forward kinematics
STATE              = robotState(chi,CONFIG);
DYNAMICS           = robotDynamics(STATE,CONFIG);
FORKINEMATICS      = robotForKinematics(STATE,DYNAMICS);

%% Dynamics parameters
Jc                 = DYNAMICS.Jc;
JH                 = DYNAMICS.JH;
M                  = DYNAMICS.M;
Mb                 = M(1:6,1:6);
Mbj                = M(1:6,7:end);
Mjb                = M(7:end,1:6);
Mj                 = M(7:end,7:end);
Jb                 = Jc(:,1:6);
Jj                 = Jc(:,7:end);
Mbar               = Mj - Mjb/Mb*Mbj;
invMbar            = eye(ndof)/Mbar;
%invMbar           = Mbar'/(Mbar*Mbar' + pinv_damp*eye(size(Mbar,1)));

%% Forward kinematics parameters
posLFoot           = FORKINEMATICS.LFootPoseQuat(1:3);
posRFoot           = FORKINEMATICS.RFootPoseQuat(1:3);
xCoM               = FORKINEMATICS.xCoM;

if sum(feet_on_ground) == 1
    
if feet_on_ground(1) == 1
posFoot      = posLFoot;
else
posFoot      = posRFoot;
end

r            = posFoot - xCoM;
A            = [eye(3)   zeros(3);
                skew(r)  eye(3) ];
pinvA        = eye(6)/A;

else    
Pr           = posRFoot - xCoM;  
Pl           = posLFoot - xCoM;

AL           = [ eye(3),  zeros(3);
                 skew(Pl),  eye(3)];
AR           = [ eye(3),  zeros(3);
                 skew(Pr),  eye(3)];   
A            = [AL, AR];
pinvA        = pinv(A,pinv_tol);
end
    
%% Initial gains (before optimization)
impedances         = gainsInit.impedances; 
dampings           = gainsInit.dampings;
MomentumGains      = gainsInit.MomentumGains;
intMomentumGains   = gainsInit.intMomentumGains;

%% DEFINE THE LINEARIZED JOINT SPACE DYNAMICS
Lambda             =  (Jj - Jb/Mb*Mbj)*invMbar;
MultFirstTask      =  Jb/Mb*transpose(Jb)*pinvA;
% pinvLambda         =  pinv(Lambda,pinv_tol);
pinvLambda       =  Lambda'/(Lambda*Lambda' + 0.01*eye(size(Lambda,1)));
NullLambda         =  eye(ndof) - pinvLambda*Lambda;
JG                 =  JH(:,7:end)-JH(:,1:6)*(eye(6)/Jb(1:6,1:6))*Jj(1:6,:);

% Postural task correction
if CONFIG.postCorrection == 1
pinvLambdaDamp     =  Lambda'/(Lambda*Lambda' + pinv_damp*eye(size(Lambda,1)));
NullLambdaDamp     =  eye(ndof) - pinvLambdaDamp*Lambda;
posturalCorr       =  NullLambdaDamp*Mbar;
else
posturalCorr       =  eye(ndof);
end

%% Stiffness matrix
KS     = invMbar*(-pinvLambda*MultFirstTask*intMomentumGains*JG + NullLambda*impedances*posturalCorr);

%% Damping matrix
KD     = invMbar*(-pinvLambda*MultFirstTask*MomentumGains*JG + NullLambda*dampings*posturalCorr);

%% Verify the state matrix (only for stability analysis, only at time t=0)
if strcmp(mode,'stability')==1
    
AStateOld     = [zeros(ndof) eye(ndof);
                    -KS           -KD];
                
        AStateDes     = [zeros(ndof) eye(ndof);
                               -gainsInit.KSdes           -gainsInit.KDdes];

eigAStateOld  = -real(eig(AStateOld));               

toleig          = 1e-5;
flag            = 0;
logicEigOld     = eigAStateOld<toleig;

if sum(logicEigOld)>0
    
    flag = 1;
end

if flag == 1

    disp('Warning: the linearized state dynamics is NOT asymptotically stable')

else
    disp('The linearized state dynamics is asymptotically stable')
end
%     disp('Linearized system eigenvalues:')
%     disp([eig(AStateOld) eig(AStateDes)])
    
   figure(26)
   plot(real(eig(AStateDes)),imag(eig(AStateDes)),'ok')   
   hold on

end

%% Parameters for visualization and gains tuning
linearization.KS          = KS;
linearization.KD          = KD;

%% Two feet correction
% gainsInit.KSdes = eye(ndof);
% gainsInit.KDdes = 2*eye(ndof);

if sum(feet_on_ground) == 2
    
Jc_base1    = Jc(1:6,1:6);
Jc_base2    = Jc(7:end,1:6);
Jc_joint1   = Jc(1:6,7:end);
Jc_joint2   = Jc(7:end,7:end);

AA     = (-Jc_base2*Jc_base1\Jc_joint1+Jc_joint2);
pinvAA = pinv(AA,1e-5);
NullAA = eye(ndof)-pinvAA*AA;

gainsInit.KSdes= NullAA*gainsInit.KSdes;
gainsInit.KDdes= NullAA*gainsInit.KDdes;
% figure
% image(gainsInit.KDdes,'CDataMapping','scaled')
% colorbar
% figure
% image(gainsInit.KSdes,'CDataMapping','scaled')
% colorbar
end


linearization.KDdes       = gainsInit.KDdes;
linearization.KSdes       = gainsInit.KSdes;
linearization.ACartesian  = -invMbar*pinvLambda*MultFirstTask;
linearization.BCartesian  = JG;
linearization.ANull       = invMbar*NullLambda;

% Postural task correction
if CONFIG.postCorrection == 1
linearization.BNull       =  NullLambdaDamp*Mbar;
else
linearization.BNull       =  eye(ndof);
end

end