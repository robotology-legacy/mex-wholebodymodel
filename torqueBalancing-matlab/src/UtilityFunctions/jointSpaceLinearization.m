function linearization = jointSpaceLinearization(CONFIG)    
%JOINTSPACELINEARIZATION linearizes the joint space dynamics of the iCub robot
%                        around an equilibrium point.
%   JOINTSPACELINEARIZATION assumes that the robot is balancing on one foot
%   or two feet, and the feedback controller is momentum-based. The solution is
%   analytical, i.e. it is not necessary to compute numerical derivatives. 
%     
%   linearization = JOINTSPACELINEARIZATION(config)  
%   defines the robot dynamics through the structure CONFIG.
%   It returns the structure LINEARIZATION which contains all the parameters
%   related to the linearized system (i.e. the stiffness and damping
%   matrix).
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% Config parameters
gainsInit          = CONFIG.gains;
pinv_tol           = CONFIG.pinv_tol;
feet_on_ground     = CONFIG.feet_on_ground;     
ndof               = CONFIG.ndof;
DYNAMICS           = CONFIG.initDynamics;
FORKINEMATICS      = CONFIG.initForKinematics;
%pinv_damp         = CONFIG.pinv_damp;

% Dynamics parameters
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

% Forward kinematics parameters
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
posturalCorr       = gainsInit.posturalCorr; 
MomentumGains      = gainsInit.MomentumGains;
intMomentumGains   = gainsInit.intMomentumGains;

%% DEFINE THE LINEARIZED JOINT SPACE DYNAMICS
Lambda             =  (Jj - Jb/Mb*Mbj)*invMbar;
MultFirstTask      =  Jb/Mb*transpose(Jb)*pinvA;
pinvLambda         =  pinv(Lambda,pinv_tol);
NullLambda         =  eye(ndof) - pinvLambda*Lambda;
JG                 =  JH(:,7:end)-JH(:,1:6)*(eye(6)/Jb)*Jj;

%% Stiffness matrix
KS     = invMbar*(-pinvLambda*MultFirstTask*intMomentumGains*JG + NullLambda*impedances*posturalCorr);

%% Damping matrix
KD     = invMbar*(-pinvLambda*MultFirstTask*MomentumGains*JG + NullLambda*dampings*posturalCorr);

%% Verify the state matrix
AStateOld     = [zeros(ndof) eye(ndof);
                    -KS           -KD];

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

% parameters for visualization and gains tuning
linearization.KS          = KS;
linearization.KD          = KD;
linearization.KDdes       = gainsInit.KDdes;
linearization.KSdes       = gainsInit.KSdes;
linearization.ACartesian  = -invMbar*pinvLambda*MultFirstTask;
linearization.BCartesian  =  JG;
linearization.ANull       =  invMbar*NullLambda;

if CONFIG.postCorrection == 1
linearization.BNull      =  NullLambda*Mbar;
%Linearization.BNull     =  posturalCorr;
else
linearization.BNull      =  posturalCorr;
end

end
