%% euleroForward
%
% simple fixed step integrator
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function [t,chi] = euleroForward(func,chiInit,tmax,tmin,tstep)

%Euler forward integrator of the robot state
t   = tmin:tstep:tmax;
t   = t.';

dimT                 = length(t);
variablesToIntegrate = length(chiInit);

chi      = zeros(variablesToIntegrate,dimT);

chi(:,1) = chiInit;

% Setup integration
for k = 2:dimT
    
 chi(:,k) = chi(:,k-1) + tstep.*func(t(k-1),chi(:,k-1));
    
end

chi = chi.';

end