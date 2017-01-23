function linearization = jointSpaceLinearization(CONFIG,qjConfig)
%JOINTSPACELINEARIZATION linearizes the joint space dynamics of the iCub robot
%                        around an equilibrium point.
%
% JOINTSPACELINEARIZATION assumes that the robot is balancing on one foot
% or two feet, and the feedback controller is momentum-based. The solution is
% analytical, i.e. it is not necessary to compute numerical derivatives.
%
% linearization = JOINTSPACELINEARIZATION(CONFIG,qjConfig) takes as an input the
% robot dynamics through the structure CONFIG, and the joint configuration
% around which the system is linearized, qjConfig.
% It returns the structure linearization which contains all the parameters
% related to the linearized system (i.e. the stiffness and damping matrix).
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
    
    [w_R_b,x_b] = wbm_getWorldFrameFromFixedLink('l_sole',qjConfig);
else
    [w_R_b,x_b] = wbm_getWorldFrameFromFixedLink('r_sole',qjConfig);
end

wbm_setWorldFrame(w_R_b,x_b,[0 0 -9.81]')

[~,basePose,~,~]   = wbm_getState();
chi                = [basePose; qjConfig; zeros(6,1); zeros(ndof,1)];

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
x_LFoot            = FORKINEMATICS.poseLFoot_qt(1:3);
x_RFoot            = FORKINEMATICS.poseRFoot_qt(1:3);
xCoM               = FORKINEMATICS.xCoM;

if sum(feet_on_ground) == 1
    
    if feet_on_ground(1) == 1
        x_Foot     = x_LFoot;
    else
        x_Foot     = x_RFoot;
    end
    
    r              = x_Foot - xCoM;
    A              = [eye(3)   zeros(3);
                      skew(r)  eye(3) ];
    pinvA          = eye(6)/A;
    
else
    rR             = x_RFoot - xCoM;
    rL             = x_LFoot - xCoM;
    AL             = [ eye(3),  zeros(3);
                       skew(rL),  eye(3)];
    AR             = [ eye(3),  zeros(3);
                       skew(rR),  eye(3)];
    A              = [AL, AR];
    pinvA          = pinv(A,pinv_tol);
end

%% Initial gains (before optimization)
impedances         = gainsInit.impedances;
dampings           = gainsInit.dampings;
momentumGains      = gainsInit.momentumGains;
intMomentumGains   = gainsInit.intMomentumGains;

%% DEFINE THE LINEARIZED JOINT SPACE DYNAMICS
Lambda             =  (Jj - Jb/Mb*Mbj)*invMbar;
MultFirstTask      =  Jb/Mb*transpose(Jb)*pinvA;
% pinvLambda       =  pinv(Lambda,pinv_tol);
pinvLambda         =  Lambda'/(Lambda*Lambda' + pinv_damp*eye(size(Lambda,1)));
NullLambda         =  eye(ndof) - pinvLambda*Lambda;
JG                 =  JH(:,7:end)-JH(:,1:6)*(eye(6)/Jb(1:6,1:6))*Jj(1:6,:);

% Postural task correction
pinvLambdaDamp     =  Lambda'/(Lambda*Lambda' + 0.5*eye(size(Lambda,1)));
NullLambdaDamp     =  eye(ndof) - pinvLambdaDamp*Lambda;
posturalCorr       =  NullLambdaDamp*Mbar;

%% Stiffness matrix
KS     = invMbar*(-pinvLambda*MultFirstTask*intMomentumGains*JG + NullLambda*impedances*posturalCorr);

%% Damping matrix
KD     = invMbar*(-pinvLambda*MultFirstTask*momentumGains*JG + NullLambda*dampings*posturalCorr);

%% Verify the state matrix
AStateOld     = [zeros(ndof) eye(ndof);
                    -KS           -KD];

eigAStateOld  = -real(eig(AStateOld));
toleig        = 1e-5;
flag          = 0;
logicEigOld   = eigAStateOld<toleig;

if sum(logicEigOld)>0
    
    flag = 1;
end

if flag == 1
    
    disp('Warning: the linearized state dynamics is NOT asymptotically stable')
else
    disp('The linearized state dynamics is asymptotically stable')
end

%% Two feet correction for gain tuning
if sum(feet_on_ground) == 2
    
    JjBar           = (eye(size(Jb,1))-Jb*pinv(Jb,pinv_tol))*Jj;
    NullJjBar       = eye(ndof)-pinv(JjBar,pinv_tol)*JjBar;
    gainsInit.KSdes = NullJjBar*gainsInit.KSdes;
    gainsInit.KDdes = NullJjBar*gainsInit.KDdes;
end

%% Parameters for visualization and gains tuning
linearization.KS          = KS;
linearization.KD          = KD;
linearization.KDdes       = gainsInit.KDdes;
linearization.KSdes       = gainsInit.KSdes;
linearization.ACartesian  = -invMbar*pinvLambda*MultFirstTask;
linearization.BCartesian  = JG;
linearization.ANull       = invMbar*NullLambda;
linearization.BNull       = posturalCorr;

end