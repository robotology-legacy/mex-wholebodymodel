function [gains,visualizeTuning] = gainsTuning (linearization,CONFIG)
%GAINSTUNING implements different algorithms to optimize the feedback control 
%            gains for the linearized joint space dynamics of iCub robot.
%   GAINSTUNING implements two different algorithms: 'lsq' uses a nonlinear
%   least square solver to optimize the gains, while 'kronecher' vectorizes
%   the gains matrices and uses a simple Moore-Penrose pseudoinverse to
%   solve the problem. The main difference is that only 'lsq' takes into
%   account the positive-definiteness constraint.
%
%   [gains,visualizeTuning] = GAINSTUNING(linearization,config)
%   takes as input the structure LINEARIZATION that comes from the joint
%   space linearization; the structure CONFIG containing the user-defined
%   parameters.
%
%   The output are the new gains matrices in the structure GAINS and the
%   parameters for visualization, VISUALIZETUNING.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% Config parameters
ndof                  = CONFIG.ndof;
gainsInit             = CONFIG.gains;
KSdes                 = gainsInit.KSdes;
KDdes                 = gainsInit.KDdes;
ACartesian            = linearization.ACartesian;
BCartesian            = linearization.BCartesian;
ANull                 = linearization.ANull;
BNull                 = linearization.BNull;
CONFIG.positDefToll   = 0.1;

%% KRONECHER OPTIMIZATION
if strcmp(CONFIG.optimization_algorithm,'kronecher') == 1
    
[Kpx,Kpn] = kronVectorization(ACartesian,BCartesian,ANull,BNull,KSdes,CONFIG);
[Kdx,Kdn] = kronVectorization(ACartesian,BCartesian,ANull,BNull,KSdes,CONFIG);
end

%% NONLINEAR LSQ OPTIMIZATION
if strcmp(CONFIG.optimization_algorithm,'NonLinLsq') == 1
CONFIG.matrixSelector = 'position';
[Kpx,Kpn] = nonLinLeastSquares(ACartesian,BCartesian,ANull,BNull,KSdes,CONFIG);

CONFIG.matrixSelector = 'velocity';
[Kdx,Kdn] = nonLinLeastSquares(ACartesian,BCartesian,ANull,BNull,KDdes,CONFIG);
end

%% New stiffness and damping matrices
KSn = ACartesian*Kpx*BCartesian + ANull*Kpn*BNull;
KDn = ACartesian*Kdx*BCartesian + ANull*Kdn*BNull;

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

% parameters for visualization                                        
visualizeTuning.KS           = KSn;
visualizeTuning.KD           = KDn;
visualizeTuning.KSdes        = KSdes;
visualizeTuning.KDdes        = KDdes;
visualizeTuning.Kpn          = Kpn;
visualizeTuning.Kdn          = Kdn;
visualizeTuning.Kpx          = Kpx;
visualizeTuning.Kdx          = Kdx;

% gains matrices after optimization
gains.impedances        = Kpn; 
gains.dampings          = Kdn;
gains.MomentumGains     = Kdx;
gains.intMomentumGains  = Kpx;
gains.posturalCorr      = gainsInit.posturalCorr;

end

