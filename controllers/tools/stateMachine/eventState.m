function [value,isterminal,direction] = eventState(t,chi)
   
global state diffState;

value      = diffState;        % detect value = 0
isterminal = 0;                % stop the integration

if state == 3 || state == 6
   isterminal = 1;             % stop the integration
end

direction  = 0;                % all directions
   
end