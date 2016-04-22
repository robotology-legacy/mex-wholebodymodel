function [Kx,Kn] = kron_prod(Ax,Bx,An,Bn,Kdesired,param)

ndof  = param.ndof;

% vectorization
Mx = kron(Bx',Ax);
Mn = kron(Bn',An);

kdes = Kdesired(:);

% linear problem
toll     = 1e-5;
Mtot     = [Mx Mn];
pinvMtot = pinv(Mtot, toll);

kxn      = pinvMtot*kdes;

% get the gain matrices
Kx     = zeros(6);
Kn     = zeros(ndof);

vettKx = kxn(1:36);
vettKn = kxn(37:end);

g      = 0;

for kk = 1:6
    
    Kx(:,kk) = vettKx(g+1:g+6);
    
    g  = g + 6;
    
end

g      = 0;

for kk = 1:ndof
    
    Kn(:,kk) = vettKn(g+1:g+ndof);
    
    g  = g + ndof;
    
end

end

