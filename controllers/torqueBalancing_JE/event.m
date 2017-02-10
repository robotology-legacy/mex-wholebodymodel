function [value,isterminal,direction] = event(t,chi)
   
  global diffState

  value      = diffState        % detect y = 0
  isterminal = 1;               % stop the integration
  direction  = 0;               % all directions
end