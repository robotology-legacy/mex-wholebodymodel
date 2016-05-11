%% gainsTuning
% current procedure used to generate the desired gains for the linearized
% system
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function [Kpx,Kdx,Kpn,Kdn,KSn,KDn] = gainsTuning(Mbar_inv,B,m,xCoM_posDerivative,angularOrientation,Nb,Mbar,dxCoM_velDerivative,...
                                                 Hw_velDerivative,gainsInit,ndof,toll,damp)
% isolate the gains: stiffness
Ap_cartesian_tot = Mbar_inv*B;
Bp_cartesian_tot = [-m*xCoM_posDerivative; angularOrientation];

Ap_null_tot      = Mbar_inv*Nb;
Bp_null_tot      = Nb*Mbar;

% isolate the gains: damping
Ad_cartesian_tot = Mbar_inv*B;
Bd_cartesian_tot = [-m*dxCoM_velDerivative; -Hw_velDerivative];

Ad_null_tot      = Mbar_inv*Nb;
Bd_null_tot      = Nb*Mbar;

% Define the desired KS,KD
KSdes            = gainsInit.KSdes;
KDdes            = gainsInit.KDdes;

% Setup for the Kronecker product
KronParam.ndof = ndof;
KronParam.toll = toll;
KronParam.damp = damp;

%% GENERATE THE GAINS BY MEANS OF THE KRONECHER PRODUCT
[Kpx,Kpn] = kron_prod_decoupled(Ap_cartesian_tot,Bp_cartesian_tot,Ap_null_tot,Bp_null_tot,KSdes,KronParam);
[Kdx,Kdn] = kron_prod_decoupled(Ad_cartesian_tot,Bd_cartesian_tot,Ad_null_tot,Bd_null_tot,KDdes,KronParam);

% Gains correction
tolPosDef = 0.1;
Kpn = Kpn+tolPosDef.*eye(ndof);
Kdn = Kdn+tolPosDef.*eye(ndof);

% New stiffness and damping matrices
KSn = Ap_cartesian_tot*Kpx*Bp_cartesian_tot + Ap_null_tot*Kpn*Bp_null_tot;
KDn = Ad_cartesian_tot*Kdx*Bd_cartesian_tot + Ad_null_tot*Kdn*Bd_null_tot;

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
    
    disp('Warning: the gains matrix on joints position is not symmetric')
    
end

if flag(4) == 1
    
    disp('Warning: the gains matrix on joints velocity is not symmetric')
    
end

% verify the positive definitess
ReigKpx = real(eig((Kpx+Kpx')/2));
ReigKdx = real(eig((Kdx+Kdx')/2));
ReigKpn = real(eig((Kpn+Kpn')/2));
ReigKdn = real(eig((Kdn+Kdn')/2));

toleig  = 1e-5;
flag    = [0;0;0;0];

logicEig = ReigKpx<toleig;
if sum(logicEig)>0
    
    flag(1) = 1;

end

logicEig = ReigKdx<toleig;
if sum(logicEig)>0
    
    flag(2) = 1;

end

logicEig = ReigKpn<toleig;
if sum(logicEig)>0
    
    flag(3) = 1;

end

logicEig = ReigKdn<toleig;
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
    
    disp('Warning: the gains matrix on joints position is not positive definite')
    
end

if flag(4) == 1
    
    disp('Warning: the gains matrix on joints velocity is not positive definite')
    
end
 
end

