function optimizedGains = gainsConstraints(gainsFromKronecker,CONFIG)
%GAINSCONSTRAINTS applies positive definiteness constraints to the gain
%                 matrices that come from Kronecker optimization.
%
% optimizedGains = GAINSCONSTRAINTS(gainsFromKronecker,CONFIG) takes as
% input the optimized gain matrices gainsFromKronecker, and the
% configuration variables.The output are the optimized and constrained
% gains matrices, inside the structure optimizedGains.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, July 2016

% ------------Initialization----------------
ndof           = CONFIG.ndof;

%% Initial conditions for integration
optimizedGains = gainsFromKronecker;

% vectorized initial conditions
vImp           = log(diag(CONFIG.gainsInit.impedances));
vDamp          = log(diag(CONFIG.gainsInit.dampings));
vMom           = log(diag(CONFIG.gainsInit.momentumGains));
vIntM          = log(diag(CONFIG.gainsInit.intMomentumGains));

% orthogonal matrices
R1ini          = eye(6);
R2ini          = eye(ndof);
R3ini          = eye(6);
R4ini          = eye(ndof);

% vectorized orthogonal matrices
vR1ini         = R1ini(:);
vR2ini         = R2ini(:);
vR3ini         = R3ini(:);
vR4ini         = R4ini(:);

initialConditions = [vIntM;vR1ini;vImp;vR2ini;vMom;vR3ini;vDamp;vR4ini];

%% Gains fixed step integrator
CONFIG.wait       = waitbar(0,'Applying gains constraints...');
vectorOfGains     = integrateGains(initialConditions,gainsFromKronecker,CONFIG);
delete(CONFIG.wait)

%% Constrained Gains
% hoping that the integrator converged to a solution, take the last values
vectorOfGainsEnd = vectorOfGains(:,end);

% reconstruct the gain matrices, considering that Kgain = R'*exp(diag(v))*R
vL1 = vectorOfGainsEnd(1:6);
vR1 = vectorOfGainsEnd(7:42);
vL2 = vectorOfGainsEnd(43:67);
vR2 = vectorOfGainsEnd(68:692);
vL3 = vectorOfGainsEnd(693:698);
vR3 = vectorOfGainsEnd(699:734);
vL4 = vectorOfGainsEnd(735:759);
vR4 = vectorOfGainsEnd(760:1384);

L1  = diag(vL1);
R1  = reshape(vR1,[6,6]);
L2  = diag(vL2);
R2  = reshape(vR2,[ndof,ndof]);
L3  = diag(vL3);
R3  = reshape(vR3,[6,6]);
L4  = diag(vL4);
R4  = reshape(vR4,[ndof,ndof]);

% final matrices
optimizedGains.intMomentumGains       = R1'*expm(L1)*R1;
optimizedGains.impedances             = R2'*expm(L2)*R2;
optimizedGains.MomentumGains          = R3'*expm(L3)*R3;
optimizedGains.dampings               = R4'*expm(L4)*R4;

end
