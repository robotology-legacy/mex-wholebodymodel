function LINEARIZATION  = jointSpaceLinearization(MODEL,INIT_CONDITIONS)
%JOINTSPACELINEARIZATION linearizes the joint space dynamics of a floating
%                        base robot around an equilibrium point. It assumes 
%                        that the robot is balancing on one foot or two feet, 
%                        and that the balancing controller is momentum-based. 
%                        The solution is analytical, i.e. it is not necessary 
%                        to compute numerical derivatives.
%
% Format:  LINEARIZATION  = JOINTSPACELINEARIZATION(MODEL,INIT_CONDITIONS)
%
% Inputs:  - MODEL: it is a structure defining the robot model;        
%          - INIT_CONDITIONS: is a structure containing initial conditions
%                             for forward dynamics integration.
%
% Output:  - LINEARIZATION: it is a structure containing joint space
%                           linearized system matrices, lin. state eigenvalues 
%                           and other parameters for gain tuning.   
%         
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% config parameters
pinv_tol           = MODEL.CONFIG.pinv_tol;
pinv_damp          = MODEL.CONFIG.pinv_damp;
feet_on_ground     = MODEL.CONFIG.feet_on_ground;
ndof               = MODEL.ndof;
% gain matrices
impedances         = MODEL.GAINS.impedances;
dampings           = MODEL.GAINS.dampings;
momentumGains      = MODEL.GAINS.momentumGains;
intMomentumGains   = MODEL.GAINS.intMomentumGains;

%% Retrieve the robot initial contidions, dynamics and forward kinematics   
% initial dynamics
Jc                 = INIT_CONDITIONS.INITDYNAMICS.Jc;
JH                 = INIT_CONDITIONS.INITDYNAMICS.JH;
M                  = INIT_CONDITIONS.INITDYNAMICS.M;
Mb                 = M(1:6,1:6);
Mbj                = M(1:6,7:end);
Mjb                = M(7:end,1:6);
Mj                 = M(7:end,7:end);
Jb                 = Jc(:,1:6);
Jj                 = Jc(:,7:end);
Mbar               = Mj - Mjb/Mb*Mbj;
invMbar            = eye(ndof)/Mbar;
% initial forward kinematics
x_LFoot            = INIT_CONDITIONS.INITFORKINEMATICS.poseLFoot_qt(1:3);
x_RFoot            = INIT_CONDITIONS.INITFORKINEMATICS.poseRFoot_qt(1:3);
xCoM               = INIT_CONDITIONS.INITFORKINEMATICS.xCoM;
% generate the matrix that projects the contact forces into the centroidal
% momentum dynamics according to the number of feet on ground
if sum(feet_on_ground) == 1
    % if one foot balancing, detect the contact position   
    if feet_on_ground(1) == 1
        x_Foot     = x_LFoot;
    else
        x_Foot     = x_RFoot;
    end
    % evaluate the distance between CoM and contact location
    r              = x_Foot - xCoM;
    A              = [eye(3)   zeros(3);
                      skewm(r) eye(3) ];
    pinvA          = eye(6)/A;    
else
    % if two feet balancing, both feet are contact locations
    rR             = x_RFoot - xCoM;
    rL             = x_LFoot - xCoM;
    AL             = [ eye(3),  zeros(3);
                       skewm(rL),  eye(3)];
    AR             = [ eye(3),  zeros(3);
                       skewm(rR),  eye(3)];
    A              = [AL, AR];
    pinvA          = pinv(A,pinv_tol);
end

%% JOINT SPACE LINEARIZATION. The theory behind this formulation can be found 
%% at http://ieeexplore.ieee.org/document/7759126/?reload=true
% evaluate all multipliers in the joint space dynamics
Lambda             =  (Jj -Jb/Mb*Mbj)*invMbar;
MatrixFirstTask    =  Jb/Mb*transpose(Jb)*pinvA;
pinvLambda         =  pinv(Lambda,pinv_tol);
NullLambda         =  eye(ndof) -pinvLambda*Lambda;
% reduced centroidal momentum matrix
JG                 =  JH(:,7:end)-JH(:,1:6)*(eye(6)/Jb(1:6,1:6))*Jj(1:6,:);
% postural task correction
pinvLambdaDamp     =  pinvDamped(Lambda,pinv_damp);
NullLambdaDamp     =  eye(ndof) -pinvLambdaDamp*Lambda;
posturalCorr       =  NullLambdaDamp*Mbar;

%% Linearized system dynamics: ddqj = ddqj0 -KS*(qj-qj0) -KD*(dqj-dqj0)
% stiffness matrix
KS     = invMbar*(-pinvLambda*MatrixFirstTask*intMomentumGains*JG + NullLambda*impedances*posturalCorr);
% damping matrix
KD     = invMbar*(-pinvLambda*MatrixFirstTask*momentumGains*JG + NullLambda*dampings*posturalCorr);

%% Verify the linearized system stability about the set point (qjInit, 0)
% state matrix of the linearized system
AState        = [zeros(ndof) eye(ndof);
                    -KS           -KD];
% state matrix eigenvalues
realEigAState = real(eig(AState));
% verify if all the real parts of the eigenvalues are negative (or almost zero)
verifyEig     = realEigAState<1e-5;
% if there are postitive eigenvalues, show a warning message
if sum(verifyEig)<sum(ones(length(verifyEig),1))
    % the joint space dynamics is not a.s. about the set point 
    warning('[Linerarization]: the joint space dynamics is NOT asymptotically stable about the equilibrium point')   
else
    disp('[Linearization]: the linearized joint space dynamics is asymptotically stable')
end

%% Store parameters for gain tuning, stability test and visualization
LINEARIZATION.KS          = KS;
LINEARIZATION.KD          = KD;
LINEARIZATION.eigAState   = eig(AState);
LINEARIZATION.ACartesian  = -invMbar*pinvLambda*MatrixFirstTask;
LINEARIZATION.BCartesian  = JG;
LINEARIZATION.ANull       = invMbar*NullLambda;
LINEARIZATION.BNull       = posturalCorr;
% in case of two feet balancing, the following matrix projects the desired
% impedances and damping matrix into the space of feasible solutions
Jj_bar                    = (eye(size(Jb,1))-Jb*pinv(Jb,pinv_tol))*Jj;
LINEARIZATION.NullJj_bar  = eye(ndof)-pinv(Jj_bar,pinv_tol)*Jj_bar;

end
