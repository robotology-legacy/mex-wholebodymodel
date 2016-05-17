function [t,chi] = integrateForwardDynEulero(func,chiInit,tmax,tmin,tstep)
%INTEGRATEFORWARDDYNEULERO is a fixed step integrator for integrating the
%                          forward dynamics of the robot iCub in MATLAB.
%   [t,chi] = integrateForwardDynEulero(func,chiInit,tmax,tmin,tstep) takes
%   as input the function to be integrated, func; the initial state of the
%   robot, chiInit; the final and initial time and the time step, tmax,
%   tmin, tstep. The outputs are the vector of integration time t and the
%   state of the robot, chi.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% initial parameters
t                    = transpose(tmin:tstep:tmax);
dimT                 = length(t);
variablesToIntegrate = length(chiInit);
chi                  = zeros(variablesToIntegrate,dimT);
chi(:,1)             = chiInit;

%% Euler fixed step integrator
for k = 2:dimT
    
 chi(:,k) = chi(:,k-1) + tstep.*func(t(k-1),chi(:,k-1)); 
end

chi = chi.';
end