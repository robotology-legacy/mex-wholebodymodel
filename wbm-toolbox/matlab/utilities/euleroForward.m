function [t,chi] = euleroForward(func,chiInit,tmax,tmin,tstep)
%EULEROFORWARD is a fixed step integrator for integrating the forward
%              dynamics of the robot iCub in MATLAB.
%
% Format: [t,chi] = EULEROFORWARD(func,chiInit,tmax,tmin,tstep)
%
% Inputs:  - func function to be integrated;
%          - chiInit initial state [13+2*ndof x 1];
%          - tmax max integration time;
%          - tmin initial integration time;
%          - tstep integration time step;
%
% Output:  - time vector t;
%          - state vector chi at each instant [13+4*ndof x lenght(t)];
%
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
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

chi = transpose(chi);

end