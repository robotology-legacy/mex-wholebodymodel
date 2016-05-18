function [Linearization, NewGains] = TwoFeetLinearization(params,gainsInit)    
%TWOFEETLINEARIZATION linearizes the joint space dynamics of the iCub robot
%                     around an equilibrium point.
%   TWOFEETLINEARIZATION assumes that the robot is balancing on two feet,
%   and the feedback controller is momentum-based. The solution is
%   analytical, i.e. it is not necessary to compute numerical derivatives. 
%     
%   [Linearization, NewGains]=TWOFEETLINEARIZATION(params,gainsInit)  
%   defines the robot dynamics through the structure PARAMS, and the feedback 
%   control gains through the structure GAINSINIT.
%   It returns two structure: LINEARIZATION contains all the parameters
%   related to the linearized system (i.e. the stiffness and damping
%   matrix), while NEWGAINS contains the new set of gains for the nonlinear
%   system. It will be different from GAINSINIT iff the gains tuning
%   procedure is allowed.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% general parameters
pinv_tol           = params.pinv_tol;
%pinv_damp         = params.pinv_damp;     
ndof               = params.ndof;

% dynamics
Jc                 = params.JcInit;
Jh                 = params.JhInit;
M                  = params.MInit;
Mb                 = M(1:6,1:6);
Mbj                = M(1:6,7:end);
Mjb                = M(7:end,1:6);
Mj                 = M(7:end,7:end);
Jb                 = Jc(:,1:6);
Jj                 = Jc(:,7:end);
Mbar               = Mj - Mjb/Mb*Mbj;
invMbar            = eye(ndof)/Mbar;
%invMbar           = Mbar'/(Mbar*Mbar' + pinv_damp*eye(size(Mbar,1)));

% forward kinematics
xLsole          = params.PoseLFootQuatInit ;
xRsole          = params.PoseRFootQuatInit ;
CoM             = params.CoMInit;
posLeftFoot     = xLsole(1:3);
posRightFoot    = xRsole(1:3);
xCoM            = CoM(1:3);

Pr              = posRightFoot - xCoM;  
Pl              = posLeftFoot  - xCoM;

AL = [ eye(3),  zeros(3);
       skew(Pl),  eye(3)];
AR = [ eye(3),  zeros(3);
       skew(Pr),  eye(3)];
   
A     = [AL, AR];
pinvA = pinv(A,pinv_tol);

%% Initial gains (before optimization)
NewGains.impedances       = gainsInit.impedances; 
NewGains.dampings         = gainsInit.dampings;
NewGains.posturalCorr     = gainsInit.posturalCorr; 
NewGains.KSdes            = gainsInit.KSdes;
NewGains.KDdes            = gainsInit.KDdes;
NewGains.VelGainsMom      = [gainsInit.gainsDCoM zeros(3); zeros(3) gainsInit.gainsDAngMom];
NewGains.PosGainsMom      = [gainsInit.gainsPCoM zeros(3); zeros(3) gainsInit.gainsPAngMom];

%% DEFINE THE LINEARIZED JOINT SPACE DYNAMICS
Lambda             =  (Jj - Jb/Mb*Mbj)*invMbar;
MultFirstTask      =  Jb/Mb*transpose(Jb)*pinvA;
pinvLambda         =  pinv(Lambda,pinv_tol);
NullLambda         =  eye(ndof) - pinvLambda*Lambda; 
JG                 =  Jh(:,7:end)-Jh(:,1:6)*(eye(6)/Jb)*Jj;

%% ADD THE COUPLING TERM BETWEEN THE RIGHT AND LEFT LEG
JL     = Jj(1:6,14:19);
JR     = Jj(7:end,20:25);
JBl    = Jb(1:6,:);
JBr    = Jb(7:end,:);
dLegs  = JR\(JBr/JBl*JL);

CorrMatrix = eye(ndof);
CorrMatrix(20:25,14:19) = dLegs;
CorrMatrixT1  = [JG(1:3,:);JG(4:6,:)*CorrMatrix];
    
%% Stiffness matrix
KS     = invMbar*(-pinvLambda*MultFirstTask*NewGains.PosGainsMom*CorrMatrixT1 + NullLambda*NewGains.impedances*NewGains.posturalCorr*CorrMatrix);

%% Damping matrix
KD     = invMbar*(-pinvLambda*MultFirstTask*NewGains.VelGainsMom*CorrMatrixT1 + NullLambda*NewGains.dampings*NewGains.posturalCorr*CorrMatrix);

%% Verify the state matrix
if params.linearize_for_stability_analysis == 1

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

end

% parameters for visualization
Linearization.KS     = KS;
Linearization.KD     = KD;
Linearization.KDdes  = gainsInit.KDdes;
Linearization.KSdes  = gainsInit.KSdes;

%% GAINS TUNING
if params.linearize_for_gains_tuning == 1
    
ACartesian = -invMbar*pinvLambda*MultFirstTask;
BCartesian =  JG;
ANull      =  invMbar*NullLambda;
BNull      =  NullLambda*Mbar;
algorithm  =  params.optimization_algorithm;

[Kpx,Kdx,Kpn,Kdn,KSn,KDn] = gainsTuning(ACartesian,BCartesian,ANull,BNull,params,NewGains,algorithm);

% new state matrix
AStateNew     = [zeros(ndof) eye(ndof);
                   -KSn        -KDn];

% parameters for visualization                                        
Linearization.KSn          = KSn;
Linearization.KDn          = KDn;
Linearization.Kpn          = Kpn;
Linearization.Kdn          = Kdn;
Linearization.Kpx          = Kpx;
Linearization.Kdx          = Kdx;

% gains matrices after optimization
NewGains.impedances    = Kpn; 
NewGains.dampings      = Kdn;
NewGains.VelGainsMom   = Kdx;
NewGains.PosGainsMom   = Kpx;

% state matrix verification
AStateDes     = [zeros(ndof)            eye(ndof);
                -gainsInit.KSdes   -gainsInit.KDdes];

eigAStateDes       = -real(eig(AStateDes));                
eigAStateNew       = -real(eig(AStateNew));

toleig             = 1e-5;
flag               = [0;0];

logicNewEig        = eigAStateNew<toleig;

if sum(logicNewEig)>0
    
    flag(1) = 1;
end

logicNewEig        = eigAStateDes<toleig;

if sum(logicNewEig)>0
    
    flag(2) = 1;
end

if flag(1) == 1

    disp('Warning: the new linearized state dynamics after gains tuning is NOT asymptotically stable')
    
elseif flag(2) == 1
    
    disp('Warning: your desired linearized state dynamics is NOT asymptotically stable')
    

elseif sum(flag) == 0
    
    disp('The linearized state dynamics after gains tuning is asymptotically stable')
end

end

end
