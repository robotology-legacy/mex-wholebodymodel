function [dotLambda,dotR,Xopt,V,Vdot]  =  gainConstr(Lambda,R,Xini,ndofMatrix,KL,KO)
%% config parameters
ndof         = size(ndofMatrix,1);
omegaDof     = (ndof*(ndof-1))/2;
tol          = 0.1;

% multipliers of derivatives
Xt           = Xini';
normTrace    = abs(trace(Xini'*Xini))+tol;
LambdaMatr   = diag(Lambda);
Rt           = transpose(R);

ATilde       = 2*(expm(2*LambdaMatr) - (R*Xt*Rt*expm(LambdaMatr)) + LambdaMatr)./normTrace;
BTilde       = 2*((Rt*expm(LambdaMatr)*R*Xt) - (Xt*Rt*expm(LambdaMatr)*R))./normTrace;
skewB        = (BTilde-transpose(BTilde))/2;

%% Derivative of lambda
a            =  diag(ATilde);
dotLambda    = -KL.*a;

%% Omega for the skew-symm matrix
g          = 1;
u          = zeros(omegaDof,1);

for i = 1:ndof
    
    for j = 1:ndof
        
        if j<i
            
        u(g) = skewB(i,j);
        g    = g+1;
        end
    end
end

%% Generate omega
omega     =  KO.*u;
g         =  1;
Lomega    =  zeros(ndof);

for i = 1:ndof
    
    for j = 1:ndof
        
        if j<i
            
            Lomega(i,j)   = omega(g);
            g             = g+1;           
        end
    end
end

% generate the skew-symm matrix
Uomega    = transpose(Lomega);
skewOmega = Lomega-Uomega;

% Rotation matrix derivative
kCorr     = 1;
dotR      = R*skewOmega +kCorr*(eye(ndof)-R*Rt)*R;

%% Optimized matrix
Xopt      = Rt*expm(LambdaMatr)*R;
T         = Xini-Xopt;
V         = trace(T'*T)./normTrace;
Vdot      = trace(ATilde*diag(dotLambda)+BTilde*skewOmega);

end

