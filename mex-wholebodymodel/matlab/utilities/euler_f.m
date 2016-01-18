function [t,u] = euler_f(f,y0,tmax,tmin,h)
%% euler_f
%  Euler forward integrator. it can be used instead of ode15s to test new
%  programs.
%% integration code
t   = tmin:h:tmax;
t   = t.';

dim       = length(t);
variables = length(y0);

u = zeros(variables,dim);

u(:,1) = y0;

for k = 2:dim
    
 u(:,k) = u(:,k-1) + h.*f(t(k-1),u(:,k-1));
    
end

u = u.';
end
    
    

