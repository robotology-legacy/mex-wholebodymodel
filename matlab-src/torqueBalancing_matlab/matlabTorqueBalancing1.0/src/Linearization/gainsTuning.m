function [Kpx,Kdx,Kpn,Kdn,KSn,KDn] = gainsTuning(ACartesian,BCartesian,ANull,BNull,params,OldGains,algorithm)
%GAINSTUNING implements different algorithms to optimize the feedback control 
%            gains for the linearized joint space dynamics of iCub robot.
%   GAINSTUNING implements two different algorithms: 'lsq' uses a nonlinear
%   least square solver to optimize the gains, while 'kronecher' vectorizes
%   the gains matrices and uses a simple Moore-Penrose pseudoinverse to
%   solve the problem. The main difference is that only 'lsq' takes into
%   account the positive-definiteness constraint.
%
%   [Kpx,Kdx,Kpn,Kdn,KSn,KDn] =
%   GAINSTUNING(ACartesian,BCartesian,ANull,BNull,params,OldGains,algorithm)
%   takes as input the pre and post multiplier of the gains matrices in the
%   linearized dynamics, which are ACartesian, BCartesian, ANull, BNull. the 
%   structure params contains all the utility parameters. The structure NewGains
%   contains the system's gains before the optimization. The variable
%
%   The output are the new gains matrices Kpx, Kdx [6x6] and Kpn, Kdn 
%   [ndof x ndof], as well as the new linearized system dynamics KSn and
%   KDn.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% setup general parameters
KSdes            = OldGains.KSdes;
KDdes            = OldGains.KDdes;
params.OldGains  = OldGains;
params.PosToll   = 0.1;
UseKron          = strcmp(algorithm,'kronecher');
UseLsq           = strcmp(algorithm,'lsq');

%% KRONECHER OPTIMIZATION
if UseKron == 1
[Kpx,Kpn] = kronProd(ACartesian,BCartesian,ANull,BNull,KSdes,params);
[Kdx,Kdn] = kronProd(ACartesian,BCartesian,ANull,BNull,KSdes,params);
end

%% NONLINEAR LSQ OPTIMIZATION
if UseLsq == 1
params.matrixSelector = 'position';
[Kpx,Kpn] = leastSquareConstr(ACartesian,BCartesian,ANull,BNull,KSdes,params);

params.matrixSelector = 'velocity';
[Kdx,Kdn] = leastSquareConstr(ACartesian,BCartesian,ANull,BNull,KDdes,params);
end

%% New stiffness and damping matrices
KSn = ACartesian*Kpx*BCartesian + ANull*Kpn*BNull;
KDn = ACartesian*Kdx*BCartesian + ANull*Kdn*BNull;

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

