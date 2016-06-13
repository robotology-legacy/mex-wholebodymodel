function [] = jointLimitsCheck(qj,t)
%JOINTLIMITSCHECK verifies the joints are inside the joint limits.
%
%                 [] = JOINTLIMITSCHECK(qj,t) take as an input the joint
%                 positions qj and the current time step t. The output is 
%                 an error message only if a joint reaches the limits.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% LIMITS CHECK
[l_min,l_max]  = wbm_jointLimits();
tol            = 0.01;
res            = qj < l_min + tol | qj > l_max - tol;
res            = sum(res);

if res == 0
else
disp('Joint limits reached at time:')    
disp(t)
error('Joint limits reached '); 
end

end