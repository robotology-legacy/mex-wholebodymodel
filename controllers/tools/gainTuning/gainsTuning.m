function [optimalGains,KSopt,KDopt] = gainsTuning(LINEARIZATION,MODEL)
%GAINSTUNING closed loop control gains offline optimization. It is based on
%            two step: first, the optimized matrices are obtained as solutions 
%            of a least squares problem. Then, the SPD constaints are enforced
%            through dynamical optimization.
%
% Format:  [optimalGains,KSopt,KDopt] = GAINSTUNING(LINEARIZATION,MODEL)
%
% Inputs:  - LINEARIZATION: is a structure containing joint space linearized 
%                           system matrices, lin. state eigenvalues 
%                           and other parameters for gain tuning; 
%          - MODEL: it is a structure defining the robot model.        
%
% Output:  - optimalGains: a structure containing the oiptimized control
%                          gains matrices;
%          - KSopt, KDopt: linearized joint space stiffness and damping
%                          matrices after gains optimization.
%         
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
disp('[Gains tuning]: applying gain tuning procedure...')
ndof                  = MODEL.ndof;
GAINS                 = MODEL.GAINS;
optimalGains          = GAINS;
pinv_damp             = MODEL.CONFIG.pinv_damp;
NullJj_bar            = LINEARIZATION.NullJj_bar;  
% desired linearized system stiffness and damping are chosen to be the
% user-defined impedances and dampings matrices. In case of two feet
% balancing, the desired stiffness and damping are projected into the space
% of feasible solutions
KSdes                 = GAINS.impedances*NullJj_bar;
KDdes                 = GAINS.dampings*NullJj_bar;
% setup the multipliers of gains matrices in the linearized system dynamics
ACartesian            = LINEARIZATION.ACartesian;
BCartesian            = LINEARIZATION.BCartesian;
ANull                 = LINEARIZATION.ANull;
BNull                 = LINEARIZATION.BNull;

%% First step: choose the gains matrices such that KS, KD are as close as possible 
%% GAINS.impedances and GAINS.dampings, respectively. This is done solving the 
%% following linear equation:  
%%     Kdes = ACartesian * XCart * BCartesian + ANull * XNull * BNull  (1)
%% With XCart,XNull unknown matrices. Eq. (1) is rewritten in the form:
%%     Mkron * x = vec_Kdes  (2)
%% by means of Kronecker product. No constraints on the positive definiteness
%% of XCart, XNull are imposed at this level, but the matrices are forced to
%% be symmetric. This may not guarantee the stability of the closed loop system

% apply Kronecker product to Eq. (1). Actually, in order to make the
% matrices symmetric it is necessary to rewrite Eq. (1) as:
%    Kdes = ACartesian *(XCart+XCart')/2* BCartesian + ANull *(XNull+XNull')/2* BNull 
% this implies it is neccessary to sum up the terms coming from the
% Kronecker product as follows
kronCartesian     = kron(transpose(BCartesian),ACartesian)*0.5;
kronNull          = kron(transpose(BNull),ANull)*0.5;
% reorder the indices of kronCartesian, kronNull. In this way it is
% possible to sum up these new matrices with the matrices above
indexMatrixCart   = transpose(reshape(1:36,[6,6]));
indexMatrixNull   = transpose(reshape(1:(ndof)^2,[ndof,ndof]));
indexVectorCart   = indexMatrixCart(:);
indexVectorNull   = indexMatrixNull(:);
% new matrices (reordered)
kronCartesianSymm = kronCartesian(:,indexVectorCart);
kronNullSymm      = kronNull(:,indexVectorNull);
% complete matrix Mkron
MKron             = [(kronCartesian+kronCartesianSymm) (kronNull+kronNullSymm)];
% vectorization of the desired matrices KS, KD
vec_KSdes         = KSdes(:);
vec_KDdes         = KDdes(:);
% the matrix Mkron turns out not to have enough rank for solving exactly
% this problem. The nullspace projector, however, can still be defined and
% can help to find a solution as close as possible the original impedances
% and dampings matrices.
pinvMKron         = pinvDamped(MKron,pinv_damp);
NullMKron         = eye(size(MKron,2))-pinvMKron*MKron;
% first solution of Eq. (2), with symmetry constraints but not necessarily
% positive definite (this is not enough to ensure stability)
x0_imp            = [GAINS.intMomentumGains(:); GAINS.impedances(:)];
x0_damp           = [GAINS.momentumGains(:); GAINS.dampings(:)];
x_imp             = pinvMKron*vec_KSdes + NullMKron*x0_imp;
x_damp            = pinvMKron*vec_KDdes + NullMKron*x0_damp;

%% Optimized gains matrices (first step)
intMomentumGains  = (reshape(x_imp(1:36),[6,6])+transpose(reshape(x_imp(1:36),[6,6])))/2;
impedances        = (reshape(x_imp(37:end),[ndof,ndof])+transpose(reshape(x_imp(37:end),[ndof,ndof])))/2;
momentumGains     = (reshape(x_damp(1:36),[6,6])+transpose(reshape(x_damp(1:36),[6,6])))/2;
dampings          = (reshape(x_damp(37:end),[ndof,ndof])+transpose(reshape(x_damp(37:end),[ndof,ndof])))/2;
% optimized stiffness and damping matix (first step)
KSkron             = ACartesian*intMomentumGains*BCartesian + ANull*impedances*BNull;
KDkron             = ACartesian*momentumGains*BCartesian + ANull*dampings*BNull;
% state matrix
AStateKron        = [ zeros(ndof)  eye(ndof); %#ok<NASGU>
                     -KSkron      -KDkron];   %#ok<NASGU>

%% Enforce the symmetry and positive definiteness constraints. This is done by 
%% optimizing a matrix of the form:
%%   Kgain =  O'*expm(Lambda)*O
%% where O is an orthogonal matrix and Lambda a diagonal matrix. Then, the 
%% derivative of O and Lambda are chosen such that |Kgain - Kkron| converges
%% to a minimum, where Kkron is each of the four gain matrices obtained after
%% Kronecker optimization. This procedure is done before forward dynamics 
%% integration (like ikin), so that it is still gain tuning. It may converge
%% to a local minimum.

% initial conditions. Given that the optimization problem is not convex, it
% may be useful to find a proper set of initial conditions. A way to do
% this is to use eigenvectors and eigenvalues as follows
[Oimp,eigImp]          = eig(impedances);
[Odamp,eigDamp]        = eig(dampings);
[OintMom,eigIntMom]    = eig(intMomentumGains);
[Omom,eigMom]          = eig(momentumGains);
eigImp                 = diag(eigImp);
eigDamp                = diag(eigDamp);
eigIntMom              = diag(eigIntMom);
eigMom                 = diag(eigMom);
% vectorized Lambda matrices
toll                   = 1e-3;
lambdaImp              = log(eigImp.*(eigImp>0) + (eigImp<=0).*toll);
lambdaDamp             = log(eigDamp.*(eigDamp>0) + (eigDamp<=0).*toll);
lambdaIntMom           = log(eigIntMom.*(eigIntMom>0) + (eigIntMom<=0).*toll);
lambdaMom              = log(eigMom.*(eigMom>0) + (eigMom<=0).*toll);
% vectorized orthogonal matrices
vOimp                  = Oimp(:);
vOdamp                 = Odamp(:);
vOmom                  = Omom(:);
vOintMom               = OintMom(:);
% initial conditions
initialConditions      = [lambdaIntMom;vOintMom;lambdaImp;vOimp;lambdaMom;vOmom;lambdaDamp;vOdamp];
% optimized matrices from Kronecker
KRON.impedances        = impedances;
KRON.dampings          = dampings;
KRON.intMomentumGains  = intMomentumGains;
KRON.momentumGains     = momentumGains;
MODEL.KRON             = KRON;
% set a waitbar and modify the default options
MODEL.wait             = waitbar(0,'1','Name','Applying gains tuning constraints...',...
                                 'CreateCancelBtn','setappdata(gcbf,''canceling'',1); delete(gcbf);');
set(MODEL.wait, 'Units', 'Pixels', 'Position', [800 500 365 100])

%% Gains constraint integration
vectorizedOptimalGains = integrateGains(initialConditions,MODEL);
delete(MODEL.wait)

% reconstruct now the gain matrices, considering that Kgain = O*expm(Lambda)*O'
lambdaOptIntMom = vectorizedOptimalGains(1:6);
vO_optIntMom    = vectorizedOptimalGains(7:42);
lambdaOptImp    = vectorizedOptimalGains(43:42+ndof);
vO_optImp       = vectorizedOptimalGains(43+ndof:42+ndof*(1+ndof));
lambdaOptMom    = vectorizedOptimalGains(43+ndof*(1+ndof):48+ndof*(1+ndof));
vO_optMom       = vectorizedOptimalGains(49+ndof*(1+ndof):84+ndof*(1+ndof));
lambdaOptDamp   = vectorizedOptimalGains(85+ndof*(1+ndof):84+ndof*(2+ndof));
vO_optDamp      = vectorizedOptimalGains(85+ndof*(2+ndof):84+2*ndof*(1+ndof));
% SPD matrices
LambdaIntMom    = diag(lambdaOptIntMom);
O_optIntMom     = reshape(vO_optIntMom,[6,6]);
LambdaImp       = diag(lambdaOptImp);
O_optImp        = reshape(vO_optImp,[ndof,ndof]);
LambdaMom       = diag(lambdaOptMom);
O_optMom        = reshape(vO_optMom,[6,6]);
LambdaDamp      = diag(lambdaOptDamp);
O_optDamp       = reshape(vO_optDamp,[ndof,ndof]);

% finally, optimized gain matrices and KS, KD
optimalGains.intMomentumGains    = transpose(O_optIntMom)*expm(LambdaIntMom)*O_optIntMom;
optimalGains.impedances          = transpose(O_optImp)*expm(LambdaImp)*O_optImp;
optimalGains.MomentumGains       = transpose(O_optMom)*expm(LambdaMom)*O_optMom;
optimalGains.dampings            = transpose(O_optDamp)*expm(LambdaDamp)*O_optDamp;
% optimized stiffness and damping matix (first step)
KSopt               = ACartesian*optimalGains.intMomentumGains*BCartesian + ANull*optimalGains.impedances*BNull;
KDopt               = ACartesian*optimalGains.momentumGains*BCartesian + ANull*optimalGains.dampings*BNull;
% state matrix
AStateOpt           = [ zeros(ndof)  eye(ndof);
                       -KSopt       -KDopt];
% state matrix eigenvalues
realEigAState = real(eig(AStateOpt));
% verify if all the real parts of the eigenvalues are negative (or almost zero)
verifyEig     = realEigAState<1e-5;
% if there are postitive eigenvalues, show a warning message
if sum(verifyEig)<sum(ones(length(verifyEig),1))
    % the joint space dynamics is not a.s. about the set point 
    warning('[Gains tuning]: the joint space dynamics after gains tuning is NOT asymptotically stable about the equilibrium point')   
else
    disp('[Gains tuning]: the linearized joint space dynamics after gains tuning is asymptotically stable')
end

%% Visualization
% desired state matrix
AStateDes           = [ zeros(ndof)  eye(ndof); %#ok<NASGU>
                       -KSdes       -KDdes];    %#ok<NASGU>

% append the optimized matrices to the stored data
if MODEL.CONFIG.enable_visualTool
    % initialize all variables (the advantage of this method is the
    % variables are preallocated)
    intMomentumGainsOpt       = transpose(O_optIntMom)*expm(LambdaIntMom)*O_optIntMom; %#ok<NASGU>
    impedancesOpt             = transpose(O_optImp)*expm(LambdaImp)*O_optImp;          %#ok<NASGU>
    momentumGainsOpt          = transpose(O_optMom)*expm(LambdaMom)*O_optMom;          %#ok<NASGU>
    dampingsOpt               = transpose(O_optDamp)*expm(LambdaDamp)*O_optDamp;       %#ok<NASGU>
    % save the data in a .mat file
    save('./media/storedValuesGainTuning','AStateDes','AStateKron','AStateOpt', ...
         'intMomentumGainsOpt','impedancesOpt','momentumGainsOpt','dampingsOpt','-v7.3');
end

end

