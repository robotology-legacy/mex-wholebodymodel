function [Kx,Kn] = kronProd(Ax,Bx,An,Bn,Kdes,params)
%KRONPROD gains optimization for the iCub linearized joint space dynamics 
%         through vectorization and Kronecher product.
%   KRONPROD solves the least square problem: x = pinv(M)*xdes where x is a
%   vectorization of the feedback control gains, xdes is the vectorization
%   of the optimization objective and M is a proper matrix.
%
%   KRONPROD also takes into account the following constraints on the gains
%   matrices: Kx should be block diagonal; Kx(1:3,1:3) should be diagonal;
%   Kx(4:6,4:6) should be symmetric. Kn should be symmetric. The positive
%   definiteness constraint is only verified a-posteriori. in particular,
%   the matrix Kn is enforced to be positive definite with the addition of
%   a regulation term.
%    
%   [Kx,Kn] = KRONPROD(Ax,Bx,An,Bn,Kdes,params) takes as inputs the pre and
%   post multiplier of the gains matrices in the linearized system dynamics, 
%   i.e. Ax,Bx,An,Bn. Kdes is the objective matrix. params. is a structure
%   containing all the utility parameters.
%
%   The output are the optimized gains matrices, Kx [6x6] and Kn [ndof x 
%   ndof]
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% setup parameters
ndof       = params.ndof;
PosToll    = params.PosToll;
%pinv_toll = param.pinv_toll;
pinv_damp  = params.pinv_damp;

%% Decoupling between linear and angular momentum
ALin    = Ax(:,1:3);
AAng    = Ax(:,4:6);
BLin    = Bx(1:3,:);
BAng    = Bx(4:6,:);

% vectorization of both the angular momentum and the postural task matrix
MAng    = kron(BAng',AAng);
Mn      = kron(Bn',An);

%% SYMMETRIZATION PROCEDURE
MnSymm         = zeros(ndof^2);
MAngSymm       = zeros(ndof^2,3^2); 

% postural task gains symmetrization
trivialVec     = 1:(ndof^2);
trivialMatr    = zeros(ndof);
g              = 0;

for kk = 1:ndof
    
    trivialMatr(:,kk) = trivialVec(g+1:g+ndof);
    g  = g + ndof;    
end

trivialMatrTranspose = trivialMatr';
trivialVecTranspose  = trivialMatrTranspose(:);

for ii = 1:length(trivialVec)
    
    for jj = 1:length(trivialVec)
       
        if trivialVec(ii)==trivialVecTranspose(jj)
            
        MnSymm(:,ii) = Mn(:,ii)+Mn(:,jj);
        end
    end
end

% angular momentum gains symmetrization
trivialVec  = 1:(3^2);
trivialMatr = zeros(3);
g           = 0;

for kk = 1:3
    
    trivialMatr(:,kk) = trivialVec(g+1:g+3);
    g  = g + 3;    
end

trivialMatrTranspose = trivialMatr';
trivialVecTranspose  = trivialMatrTranspose(:);

for ii = 1:length(trivialVec)
    
    for jj = 1:length(trivialVec)
       
        if trivialVec(ii)==trivialVecTranspose(jj)
            
        MAngSymm(:,ii) = MAng(:,ii)+MAng(:,jj);
        end
    end
end

%% Linear momentum gains diagonalization
FirstTerm  = kron(BLin(1,:)', ALin(:,1));
SecondTerm = kron(BLin(2,:)', ALin(:,2));
ThirdTerm  = kron(BLin(3,:)', ALin(:,3));

MLinSymm   = [FirstTerm SecondTerm ThirdTerm];

%% VECTORIZATION
xdes       = Kdes(:);
MKron      = [MLinSymm MAngSymm MnSymm];
%pinvMKron = pinv(MKron, pinv_toll);
pinvMKron  = MKron'/(MKron*MKron' + pinv_damp*eye(size(MKron,1)));
x          = pinvMKron*xdes;

%% Matrices generation
KAng        = zeros(3);
Kn          = zeros(ndof);
vettKLin    = x(1:3);
vettKAng    = x(4:12);
vettKn      = x(13:end);

% linear momentum gains
KLin        = diag(vettKLin);

% angular momentum gains
g           = 0;

for kk = 1:3
    
    KAng(:,kk) = vettKAng(g+1:g+3);
    g  = g + 3;    
end

% postural task gains
g           = 0;

for kk = 1:ndof
    
    Kn(:,kk) = vettKn(g+1:g+ndof);
    g  = g + ndof;    
end

%% Final gains matrices
KAng    = KAng+KAng';
Kx      = [KLin zeros(3); zeros(3) KAng];
Kn      = Kn+Kn';

% posture correction to get the positive definiteness
Kn = Kn + PosToll.*eye(ndof);

end

