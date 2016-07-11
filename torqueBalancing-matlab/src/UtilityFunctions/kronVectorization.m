function [Kx,Kn] = kronVectorization(Ax,Bx,An,Bn,Kdes,CONFIG)
%KRONVECTORIZATION is a gains optimization for the iCub linearized joint 
%                  space dynamics through vectorization and Kronecher product.
%   KRONVECTORIZATION solves the least square problem: x = pinv(M)*xdes 
%   where x is a vectorization of the feedback control gains, xdes is the 
%   vectorization of the optimization objective and M is a proper matrix.
%
%   KRONVECTORIZATION also takes into account the following constraints on
%   the gains matrices: Kx should be block diagonal; Kx(1:3,1:3) should be 
%   diagonal; Kx(4:6,4:6) should be symmetric. Kn should be symmetric. The
%   positive definiteness constraint is only verified a-posteriori. in 
%   particular, the matrix Kn is enforced to be positive definite with the 
%   addition of a regulation term.
%    
%   [Kx,Kn] = KRONVECTORIZATION(Ax,Bx,An,Bn,Kdes,config) takes as inputs 
%   the pre and post multiplier of the gains matrices in the linearized 
%   system dynamics, i.e. AX,BX,AN,BN. KDES is the objective matrix. CONFIG 
%   is a structure containing all the utility parameters.
%
%   The output are the optimized gains matrices, Kx [6x6] and Kn [ndof x 
%   ndof]
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% setup parameters
ndof       = CONFIG.ndof;
pinv_toll = CONFIG.pinv_tol;

kronMom        =   kron(Bx',Ax);
kronNull       =   kron(Bn',An);

MKron          =   [kronMom kronNull];
% pinvMKron      =   pinv(MKron,pinv_toll);
pinvMKron      =   pinvDamped(MKron,0.01);

% position gains
xdes           = Kdes(:);

x              = pinvMKron*xdes;

vettKMom       = x(1:36);
vettKn         = x(37:end);

Kx          = reshape(vettKMom,[6,6]);
Kn         = reshape(vettKn,[ndof,ndof]);

end

