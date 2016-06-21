function [t,chi] = euleroForward(func,chiInit,tmax,tmin,tstep)
%EULEROFORWARD is a fixed step integrator for integrating the forward 
%              dynamics of the robot iCub in MATLAB.
%
%   [t,chi] = EULEROFORWARD(func,chiInit,tmax,tmin,tstep) takes as input 
%   the function to be integrated, FUNC; the initial state of the robot, 
%   CHIINIT; the final and initial time and the time step, TMAX,TMIN,TSTEP. 
%   The outputs are the vector of integration time T and the state of the 
%   robot, CHI.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
%% Initial conditions
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