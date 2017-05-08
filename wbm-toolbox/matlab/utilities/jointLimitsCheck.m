function [] = jointLimitsCheck(qj,t)
%JOINTLIMITSCHECK verifies the joints are inside the joint limits.
%
% Format: [] = JOINTLIMITSCHECK(qj,t)
%
% Inputs:  - joint positions qj [ndof x 1]; 
%          - current time t [s];
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017
%
%% ------------Initialization----------------
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