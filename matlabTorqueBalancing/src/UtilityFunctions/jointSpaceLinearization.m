function linearization = jointSpaceLinearization(config,gainsInit)    
%JOINTSPACELINEARIZATION linearizes the joint space dynamics of the iCub robot
%                        around an equilibrium point.
%   JOINTSPACELINEARIZATION assumes that the robot is balancing on one foot
%   or two feet, and the feedback controller is momentum-based. The solution is
%   analytical, i.e. it is not necessary to compute numerical derivatives. 
%     
%   linearization = JOINTSPACELINEARIZATION(config,gainsInit)  
%   defines the robot dynamics through the structure CONFIG, and the feedback 
%   control gains through the structure GAINSINIT.
%   It returns the structure LINEARIZATION which contains all the parameters
%   related to the linearized system (i.e. the stiffness and damping
%   matrix).
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% Config parameters
pinv_tol           = config.pinv_tol;
feet_on_ground     = config.feet_on_ground;     
ndof               = config.ndof;
dynamics           = config.InitDynamics;
forKinematics      = config.InitForKinematics;
%pinv_damp         = params.pinv_damp;

% Dynamics parameters
Jc                 = dynamics.Jc;
JH                 = dynamics.JH;
M                  = dynamics.M;
Mb                 = M(1:6,1:6);
Mbj                = M(1:6,7:end);
Mjb                = M(7:end,1:6);
Mj                 = M(7:end,7:end);
Jb                 = Jc(:,1:6);
Jj                 = Jc(:,7:end);
Mbar               = Mj - Mjb/Mb*Mbj;
invMbar            = eye(ndof)/Mbar;
%invMbar           = Mbar'/(Mbar*Mbar' + pinv_damp*eye(size(Mbar,1)));

% Forward kinematics
posLFoot           = forKinematics.LFootPoseQuat(1:3);
posRFoot           = forKinematics.RFootPoseQuat(1:3);
xCoM               = forKinematics.xCoM;

if sum(feet_on_ground) == 1
    
if feet_on_ground(1) == 1
posFoot      = posLFoot;
else
posFoot      = posRFoot;
end

r            = posFoot-xCoM;

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
impedances       = gainsInit.impedances; 
dampings         = gainsInit.dampings;
posturalCorr     = gainsInit.posturalCorr; 
VelGainsMom      = [gainsInit.gainsDCoM zeros(3); zeros(3) gainsInit.gainsDAngMom];
PosGainsMom      = [gainsInit.gainsPCoM zeros(3); zeros(3) gainsInit.gainsPAngMom];

%% DEFINE THE LINEARIZED JOINT SPACE DYNAMICS
Lambda             =  (Jj - Jb/Mb*Mbj)*invMbar;
MultFirstTask      =  Jb/Mb*transpose(Jb)*pinvA;
pinvLambda         =  pinv(Lambda,pinv_tol);
NullLambda         =  eye(ndof) - pinvLambda*Lambda; 
JG                 =  JH(:,7:end)-JH(:,1:6)*(eye(6)/Jb)*Jj;

%% COUPLING FOR TWO FEET BALANCING
if sum(feet_on_ground) == 1

CorrMatrix              = eye(ndof);
CorrMatrixT1            = JG;

else
JL     = Jj(1:6,14:19);
JR     = Jj(7:end,20:25);
JBl    = Jb(1:6,:);
JBr    = Jb(7:end,:);
dLegs  = JR\(JBr/JBl*JL);

CorrMatrix              = eye(ndof);
CorrMatrix(20:25,14:19) = dLegs;
CorrMatrixT1            = [JG(1:3,:);JG(4:6,:)*CorrMatrix];
end

%% Stiffness matrix
KS     = invMbar*(-pinvLambda*MultFirstTask*PosGainsMom*CorrMatrixT1 + NullLambda*impedances*posturalCorr*CorrMatrix);

%% Damping matrix
KD     = invMbar*(-pinvLambda*MultFirstTask*VelGainsMom*CorrMatrixT1 + NullLambda*dampings*posturalCorr*CorrMatrix);

%% Verify the state matrix
AStateOld     = [zeros(ndof) eye(ndof);
                    -KS         -KD];

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
linearization.KS     = KS;
linearization.KD     = KD;
linearization.KDdes  = gainsInit.KDdes;
linearization.KSdes  = gainsInit.KSdes;

% parameters for gains tuning
linearization.ACartesian = -invMbar*pinvLambda*MultFirstTask;
linearization.BCartesian =  CorrMatrixT1;
linearization.ANull      =  invMbar*NullLambda;

if config.postCorrection == 1
linearization.BNull      =  NullLambda*Mbar*CorrMatrix;
%Linearization.BNull     =  posturalCorr*CorrMatrix;
else
linearization.BNull      =  posturalCorr*CorrMatrix;
end

end
