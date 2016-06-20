function gainsOpt = gainsTuning (linearization,CONFIG,mode)
%GAINSTUNING implements different algorithms to optimize the feedback control 
%            gains for the linearized joint space dynamics of iCub robot.
%   GAINSTUNING implements two different algorithms: 'lsq' uses a nonlinear
%   least square solver to optimize the gains, while 'kronecher' vectorizes
%   the gains matrices and uses a simple Moore-Penrose pseudoinverse to
%   solve the problem. The main difference is that only 'lsq' takes into
%   account the positive-definiteness constraint.
%
%   gainsOpt = GAINSTUNING(linearization,config,mode)
%   takes as input the structure LINEARIZATION that comes from the joint
%   space linearization; the structure CONFIG containing the user-defined
%   parameters. The string MODE can be either 'normal' or 'debug'. The only
%   difference is that in 'debug' mode the function also verifies the gains
%   matrices to be symmetric positive definite, and the new state matrix to
%   have all the eigenvalues with negative real part.
%
%   The output are the new gains matrices in the structure GAINSOPT.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% Config parameters
ndof                  = CONFIG.ndof;
gainsInit             = CONFIG.gainsInit;
KSdes                 = gainsInit.KSdes;
KDdes                 = gainsInit.KDdes;
ACartesian            = linearization.ACartesian;
BCartesian            = linearization.BCartesian;
ANull                 = linearization.ANull;
BNull                 = linearization.BNull;
CONFIG.positDefToll   = 0.1;

%% NONLINEAR LSQ OPTIMIZATION
CONFIG.matrixSelector = 'position';
[Kpx,Kpn] = nonLinLeastSquares(ACartesian,BCartesian,ANull,BNull,KSdes,CONFIG);

CONFIG.matrixSelector = 'velocity';
[Kdx,Kdn] = nonLinLeastSquares(ACartesian,BCartesian,ANull,BNull,KDdes,CONFIG);

%% New stiffness and damping matrices
KSn = ACartesian*Kpx*BCartesian + ANull*Kpn*BNull;
KDn = ACartesian*Kdx*BCartesian + ANull*Kdn*BNull;

if strcmp(mode,'debug') == 1
% new state matrix
AStateNew     = [zeros(ndof) eye(ndof);
                   -KSn        -KDn];

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

%% New gains verification
% verify the symmetry
SymmKpx = (Kpx+Kpx')/2;
SymmKdx = (Kdx+Kdx')/2;
SymmKpn = (Kpn+Kpn')/2;
SymmKdn = (Kdn+Kdn')/2;

tolSymm = 1e-7;
flag    = [0;0;0;0];

logicSymm = abs(SymmKpx-Kpx)>tolSymm;

if sum(sum(logicSymm))>0
    
    flag(1) = 1;
end

logicSymm = abs(SymmKdx-Kdx)>tolSymm;

if sum(sum(logicSymm))>0
    
    flag(2) = 1;
end

logicSymm = abs(SymmKpn-Kpn)>tolSymm;

if sum(sum(logicSymm))>0
    
    flag(3) = 1;
end

logicSymm = abs(SymmKdn-Kdn)>tolSymm;

if sum(sum(logicSymm))>0
    
    flag(4) = 1;
end

if sum(flag)  == 0
    
    disp('All the gains matrices are symmetric')
end

if flag(1) == 1   
    
    disp('Warning: the gains matrix on centroidal momentum pose is not symmetric')
end

if flag(2) == 1
    
    disp('Warning: the gains matrix on centroidal momentum velocity is not symmetric')
end

if flag(3) == 1
    
    disp('Warning: the gains matrix on joint position is not symmetric')
end

if flag(4) == 1
    
    disp('Warning: the gains matrix on joint velocity is not symmetric')
end

% verify the positive definiteness
eigKpx = real(eig((Kpx+Kpx')/2));
eigKdx = real(eig((Kdx+Kdx')/2));
eigKpn = real(eig((Kpn+Kpn')/2));
eigKdn = real(eig((Kdn+Kdn')/2));

toleig = 1e-5;
flag   = [0;0;0;0];

logicEig = eigKpx<toleig;

if sum(logicEig)>0
    
    flag(1) = 1;
end

logicEig = eigKdx<toleig;

if sum(logicEig)>0
    
    flag(2) = 1;
end

logicEig = eigKpn<toleig;

if sum(logicEig)>0
    
    flag(3) = 1;
end

logicEig = eigKdn<toleig;

if sum(logicEig)>0
    
    flag(4) = 1;
end

if sum(flag)  == 0
    
    disp('All the gains matrices are positive definite')
end

if flag(1) == 1
    
    disp('Warning: the gains matrix on centroidal momentum pose is not positive definite')
end

if flag(2) == 1
    
    disp('Warning: the gains matrix on centroidal momentum velocity is not positive definite')
end

if flag(3) == 1
    
    disp('Warning: the gains matrix on joint position is not positive definite')   
end

if flag(4) == 1
    
    disp('Warning: the gains matrix on joint velocity is not positive definite')   
end
end

%% Gains matrices after optimization
gainsOpt.impedances             = Kpn; 
gainsOpt.dampings               = Kdn;
gainsOpt.MomentumGains          = Kdx;
gainsOpt.intMomentumGains       = Kpx;
gainsOpt.CorrPosFeet            = gainsInit.CorrPosFeet;
gainsOpt.KSdes                  = KSdes;
gainsOpt.KDdes                  = KDdes;
gainsOpt.KSn                    = KSn;
gainsOpt.KDn                    = KDn;

end

