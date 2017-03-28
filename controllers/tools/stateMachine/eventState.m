function [value,isterminal,direction] = eventState(t,chi)
%EVENTSTATE detect finit events during the numerical integration.
%
% Format: [value,isterminal,direction] = EVENTSTATE(t,chi)
%
% Inputs (not used):  - current time t [s];
%                     - state vector chi [13+4*ndof x 1]. 
%
% Output:  - value       the value of the cost funtion at which the event is
%                        detected;
%          - isteminal   if it is == 1, it stops the integration;
%          - direction   1 positive, -1 negative, 0 all directions.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% actually, the finite event is detected using global variables
global state diffState;
% detect value = 0
value      = diffState; 
% stop the integration only in state 3,7,9,13
isterminal = 0; 

if state == 3 || state == 7 ||state ==9 || state ==13
   isterminal = 1;            
end
% all directions
direction  = 0;             
   
end