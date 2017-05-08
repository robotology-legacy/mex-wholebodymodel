function [dlambda,dO] = gainsDerivative(lambda,O,MODEL)
%GAINSDERIVATIVE computes the time derivative of Lambda, O such that the SPD
%                matrix K = O'*expm(Lambda)*O converges to the one that minimizes 
%                the error norm: |Kkron-K|
%
% Format:  [dlambda,dO] = GAINSDERIVATIVE(lambda,O,MODEL)
%
% Inputs:  - lambda: a vector representing the diagonal of matrix Lambda;
%          - O: orthogonal matrix;
%          - MODEL: is a structure defining the robot model.        
%
% Output:  - dlambda: time derivative of lambda;
%          - dO: time derivative of O.
%         
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% configuration parameters
ndof         = size(O,1);
omegaDof     = (ndof*(ndof-1))/2;
tol          = 0.1;
Ot           = transpose(O);
Lambda       = diag(lambda);
O_init       = reshape(MODEL.vO_init,[ndof,ndof]);
Lambda_init  = diag(MODEL.lambda_init);
Xkron        = MODEL.Xkron;
Xkron_t      = transpose(Xkron);
% integration gains
KLambda      = MODEL.KLambda;
KO           = MODEL.KO;
% normalize the formula on the initial matrix. This is a way to avoid the gains 
% dynamics to be too fast for being controlled properly
Xini         = transpose(O_init)*expm(Lambda_init)*O_init;
Xini_t       = transpose(Xini);
trace_init   = abs(trace(Xini_t*Xini))+tol;

%% Compute the cost function V and its time derivative. The cost function V is of the form:
%%     V = trace(T'*T)/trace_init
%% where matrix T is the error (Xkron - Ot*expm(Lambda)*O), i.e. the error between 
%% the desired optimized matrix from Kronecker and a SPD matrix. The time derivative 
%% of V is then of the form:
%%     dV = trace(A*dLambda + B*dO)/trace_init
%% it is then possible to choose dLambda, dO, such that dV <= 0, i.e. the cost
%% function V converges to a local minimum

% multiplier of dLambda
A            = 2*(expm(2*Lambda) - (O*Xkron_t*Ot*expm(Lambda)))/trace_init;
% multipliers of dO, after recalling that dO = O*skew(omega), and that
% trace(B*skew(omega)) = trace(Bskew*skew(omega))
BTilde       = 2*((Ot*expm(Lambda)*O*Xkron_t) - (Xkron_t*Ot*expm(Lambda)*O))/trace_init;
Bskew        = (BTilde-transpose(BTilde))/2;

%% Derivative of lambda (vector)
a            =  diag(A);
dlambda      = -KLambda.*(expm(-Lambda)*a);

%% Compute the derivative of O. This is dome by choosing the vector omega
g            = 1;
u            = zeros(omegaDof,1);
% this is a particular mapping of the matrix Bskew into vector u
for i = 1:ndof    
    for j = 1:ndof        
        if j<i
            u(g) = Bskew(i,j);
            g    = g+1;
        end
    end
end
% omega is chosen to be equal to u multiplied by a positive gain
omega       =  KO.*u;
g           =  1;
Lomega      =  zeros(ndof);
% compute the lower triangular matrix Lomega (inverse of the previous
% mapping)
for i = 1:ndof
    for j = 1:ndof
        if j<i
            Lomega(i,j)   = omega(g);
            g             = g+1;
        end
    end
end
% generate the skew-symm matrix from the lower triangular matrix Lomega
Uomega      = transpose(Lomega);
skewOmega   = Lomega-Uomega;
% Finally, compute the rotation matrix derivative. A correction term is
% used to constrain O to belong to the orthogonal group
Kcorr       = 1;
dO          = O*skewOmega +Kcorr*(eye(ndof)-O*Ot)*O;

end

