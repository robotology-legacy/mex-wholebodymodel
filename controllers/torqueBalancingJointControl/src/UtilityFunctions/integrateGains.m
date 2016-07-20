function gainsVectOpt =integrateGains(gainsVectInit,gainsKron,CONFIG)

%% Initial conditions
tStep            = 0.01;
CONFIG.tEndgain  = 50;
t                = transpose(0:tStep:CONFIG.tEndgain);
dimTime          = length(t);
dimState         = length(gainsVectInit);

% variable to be integrated
gainsVectOpt         = zeros(dimState,dimTime);
gainsVectOpt(:,1)    = gainsVectInit; 

%% Function to be integrated
integratedFunction   = @(t,gainsVect) gainConstrAll(t,gainsVect,gainsKron,CONFIG);

%% Euler forward integrator (fixed step)
for kk = 2:dimTime
% state at step k
gainsVectOpt(:,kk)   = gainsVectOpt(:,kk-1) + tStep.*integratedFunction(t(kk-1),gainsVectOpt(:,kk-1));
end

end
