%% kron_prod_decoupled
%
% This function implements a gains optimization for both the high level
% task gains and the low level task gains.
%
function [Kx,Kn] = kron_prod_decoupled(Ax,Bx,An,Bn,Kdesired,param)

ndof  = param.ndof;
toll  = param.toll;
damp  = param.damp;

%% Decoupling between CoM position and angular momentum integral
AxCoMPos = Ax(:,1:3);
AxOri    = Ax(:,4:6);

BxCoMPos = Bx(1:3,:);
BxOri    = Bx(4:6,:);

% vectorization
MxOri    = kron(BxOri',AxOri);
Mn       = kron(Bn',An);

%% SYMMETRIZATION
Mn_symm         = zeros(ndof^2);
MxOri_symm      = zeros(ndof^2,3^2); 

% null space
trivialVec  = 1:(ndof^2);
trivialMatr = zeros(ndof);

g      = 0;

for kk = 1:ndof
    
    trivialMatr(:,kk) = trivialVec(g+1:g+ndof);
    
    g  = g + ndof;
    
end

trivMatrTranspose = trivialMatr';
trivVecTranspose  = trivMatrTranspose(:);

for ii = 1:length(trivialVec)
    
    for jj = 1:length(trivialVec)
       
        if trivialVec(ii)==trivVecTranspose(jj)
            
        Mn_symm (:,ii) = Mn(:,ii)+Mn(:,jj);
        
        end
    end
end

% CoM matrix diagonalization
FirstTerm  = kron(BxCoMPos(1,:)', AxCoMPos(:,1));
SecondTerm = kron(BxCoMPos(2,:)', AxCoMPos(:,2));
ThirdTerm  = kron(BxCoMPos(3,:)', AxCoMPos(:,3));

MxCoMPos_symm = [FirstTerm SecondTerm ThirdTerm];

% angular momentum integral matrix
trivialVec  = 1:(3^2);
trivialMatr = zeros(3);

g      = 0;

for kk = 1:3
    
    trivialMatr(:,kk) = trivialVec(g+1:g+3);
    
    g  = g + 3;
    
end

trivMatrTranspose = trivialMatr';
trivVecTranspose  = trivMatrTranspose(:);

for ii = 1:length(trivialVec)
    
    for jj = 1:length(trivialVec)
       
        if trivialVec(ii)==trivVecTranspose(jj)
            
        MxOri_symm (:,ii) = MxOri(:,ii)+MxOri(:,jj);
        
        end
    end
end

%% Pseudoinverse
kdes = Kdesired(:);

% linear problem
Mtot      = [MxCoMPos_symm MxOri_symm Mn_symm];
pinvMtot  = pinv(Mtot, toll);
%pinvMtot = Mtot'/(Mtot*Mtot' + damp*eye(size(Mtot,1)));

kxn       = pinvMtot*kdes;

% get the gain matrices
KxOri        = zeros(3);
Kn           = zeros(ndof);

vettKxCoMPos = kxn(1:3);
vettKxOri    = kxn(4:12);
vettKn       = kxn(13:end);

% CoM position matrix
KxCoMPos = diag(vettKxCoMPos);

% angular momentum orientation matrix
g       = 0;

for kk = 1:3
    
    KxOri(:,kk) = vettKxOri(g+1:g+3);
    
    g  = g + 3;
    
end
g      = 0;

for kk = 1:ndof
    
    Kn(:,kk) = vettKn(g+1:g+ndof);
    
    g  = g + ndof;
    
end

KxOri    = KxOri+KxOri';
Kx       = [KxCoMPos zeros(3); zeros(3) KxOri];
Kn       = Kn+Kn';

end

