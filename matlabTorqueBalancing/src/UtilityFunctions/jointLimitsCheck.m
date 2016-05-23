function [] = jointLimitsCheck(qj,t)
%JOINTLIMITSCHECK verify the joints are inside the joint limits.
%                 [] = JOINTLIMITSCHECK(qj,t) take as an input the joint
%                 positions QJ and the current time step T. The output is 
%                 an error message when the robot reaches the joint limits.
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