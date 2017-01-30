function vectorOfGains = integrateGains(initialConditions,gainsKronecker,CONFIG)
%INTEGRATEGAINS integrates the time derivative of decomposed gains matrices
%               and tries to make them converge to a solution.
%
% vectorOfGains = INTEGRATEGAINS(initialConditions,gainsKronecker,CONFIG)
% takes as input the initial contitions for integration, the gains matrices
% coming from Kronecker optimization and the configuration parameters. The
% output are the vectorized gains coming from fixed step Euler integration.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, July 2016

% ------------Initialization----------------
%% Initial conditions
% the integration time and the integration step are chosen arbitrarly
tStep            = 0.01;
CONFIG.tEndGain  = 50;
t                = transpose(0:tStep:CONFIG.tEndGain);
dimTime          = length(t);
dimState         = length(initialConditions);

% variable to be integrated
vectorOfGains         = zeros(dimState,dimTime);
vectorOfGains(:,1)    = initialConditions;

%% Function to be integrated
integratedFunction   = @(t,gainsVect) gainsDynamics(t,gainsVect,gainsKronecker,CONFIG);

%% Euler forward integrator (fixed step)
for kk = 2:dimTime
    % state at step k
    vectorOfGains(:,kk)   = vectorOfGains(:,kk-1) + tStep.*integratedFunction(t(kk-1),vectorOfGains(:,kk-1));
end

end
