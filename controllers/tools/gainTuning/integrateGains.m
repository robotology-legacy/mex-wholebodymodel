function vectorizedOptimalGains = integrateGains(initialConditions,MODEL)
%INTEGRATEGAINS numerical fixed step integration of control gains in the
%               space of SPD matrices.
%
% Format:  vectorizedOptimalGains = INTEGRATEGAINS(initialConditions,MODEL)
%
% Inputs:  - initialConditions: it is a vector of initial control gains;
%          - MODEL: it is a structure defining the robot model.        
%
% Output:  - vectorizedOptimalGains: a vector of optimized control gains.
%         
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% the integration time and the integration step are chosen arbitrarly
ndof                  = MODEL.ndof;
tStep                 = 0.01;
tEnd                  = 10;
t                     = transpose(0:tStep:tEnd);
dimTime               = length(t);
dimState              = length(initialConditions);
% variable to be integrated
vectorOfGains         = zeros(dimState,dimTime);
vectorOfGains(:,1)    = initialConditions;
% extract the variables from the initial vector
lambdaIntMom          = vectorOfGains (1:6,1);
vO_IntMom             = vectorOfGains (7:42,1);
lambdaImp             = vectorOfGains (43:42+ndof,1);
vO_Imp                = vectorOfGains (43+ndof:42+ndof*(1+ndof),1);
lambdaMom             = vectorOfGains (43+ndof*(1+ndof):48+ndof*(1+ndof),1); 
vO_Mom                = vectorOfGains (49+ndof*(1+ndof):84+ndof*(1+ndof),1);
lambdaDamp            = vectorOfGains (85+ndof*(1+ndof):84+ndof*(2+ndof),1);
vO_Damp               = vectorOfGains (85+ndof*(2+ndof):84+2*ndof*(1+ndof),1);
% reshape orthogonal matrices
O_IntMom              = reshape(vO_IntMom,[6,6]);
O_Imp                 = reshape(vO_Imp,[ndof,ndof]);
O_Mom                 = reshape(vO_Mom,[6,6]);
O_Damp                = reshape(vO_Damp ,[ndof,ndof]);
% gains for numerical integration
MODEL.KLambda         = 35;
MODEL.KO              = 35;

%% Eulero forward integrator (fixed step)
for kk = 2:dimTime
    % report current time in the waitbar's message field
    waitbar(t(kk)/tEnd,MODEL.wait,['Current step: ',sprintf('%0.0f',t(kk))])
    % derivative of integral of momentum gains
    MODEL.lambda_init          = initialConditions(1:6);
    MODEL.vO_init              = initialConditions(7:42);
    MODEL.Xkron                = MODEL.KRON.intMomentumGains;
    [dlambdaIntMom,dO_IntMom]  = gainsDerivative(lambdaIntMom,O_IntMom,MODEL);
    % derivative of impedances gains
    MODEL.lambda_init          = initialConditions(43:42+ndof);
    MODEL.vO_init              = initialConditions(43+ndof:42+ndof*(1+ndof));
    MODEL.Xkron                = MODEL.KRON.impedances;
    [dlambdaImp,dO_Imp]        = gainsDerivative(lambdaImp,O_Imp,MODEL);
    % derivative of momentum gains
    MODEL.lambda_init          = initialConditions(43+ndof*(1+ndof):48+ndof*(1+ndof));
    MODEL.vO_init              = initialConditions(49+ndof*(1+ndof):84+ndof*(1+ndof));
    MODEL.Xkron                = MODEL.KRON.momentumGains;
    [dlambdaMom,dO_Mom]        = gainsDerivative(lambdaMom,O_Mom,MODEL);
    % derivative of dampings gains
    MODEL.lambda_init          = initialConditions(85+ndof*(1+ndof):84+ndof*(2+ndof));
    MODEL.vO_init              = initialConditions(85+ndof*(2+ndof):84+2*ndof*(1+ndof));
    MODEL.Xkron                = MODEL.KRON.dampings;
    [dlambdaDamp,dO_Damp]      = gainsDerivative(lambdaDamp,O_Damp,MODEL);   
    % Reshape orthogonal matrices into vectors
    v_dOintMom                 = dO_IntMom(:);
    v_dOimp                    = dO_Imp(:);
    v_dOmom                    = dO_Mom(:);
    v_dOdamp                   = dO_Damp(:);
    % gains vector derivative
    dvectorOfGains             = [dlambdaIntMom;v_dOintMom;dlambdaImp;v_dOimp;dlambdaMom;v_dOmom;dlambdaDamp;v_dOdamp];
    % vector of gains at step kk
    vectorOfGains(:,kk)        = vectorOfGains(:,kk-1) + tStep.*dvectorOfGains;
    % reshape and extract values (overwrite previous valuse)
    lambdaIntMom               = vectorOfGains (1:6,kk);
    vO_IntMom                  = vectorOfGains (7:42,kk);
    lambdaImp                  = vectorOfGains (43:42+ndof,kk);
    vO_Imp                     = vectorOfGains (43+ndof:42+ndof*(1+ndof),kk);
    lambdaMom                  = vectorOfGains (43+ndof*(1+ndof):48+ndof*(1+ndof),kk); 
    vO_Mom                     = vectorOfGains (49+ndof*(1+ndof):84+ndof*(1+ndof),kk);
    lambdaDamp                 = vectorOfGains (85+ndof*(1+ndof):84+ndof*(2+ndof),kk);
    vO_Damp                    = vectorOfGains (85+ndof*(2+ndof):84+2*ndof*(1+ndof),kk);
    % reshape orthogonal matrices
    O_IntMom                   = reshape(vO_IntMom,[6,6]);
    O_Imp                      = reshape(vO_Imp,[ndof,ndof]);
    O_Mom                      = reshape(vO_Mom,[6,6]);
    O_Damp                     = reshape(vO_Damp ,[ndof,ndof]);
end

%% Optimized vector of gains
vectorizedOptimalGains         = vectorOfGains(:,end);

end
