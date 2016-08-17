function vectorOfGains = integrateGains(initialConditions,gainsKronecker,CONFIG)
%% Initial conditions
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
