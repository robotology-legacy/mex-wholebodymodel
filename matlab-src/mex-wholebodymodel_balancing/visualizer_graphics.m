function [] = visualizer_graphics(t,chi,params)

%Plot all the desired quantities
%for now, these are the parameters calculated in ForwardDynamics.m
fc     = [];
f0     = [];
tau    = [];
ecom   = [];
pos    = [];

for tt=1:length(t)
    
[~,c] = forwardDynamics(t(tt), chi(tt,:).', params);

pos_t   = c.pos_feet(1:3);
fc_t    = c.fc;
f0_t    = c.f0;
tau_t   = c.tau;
ecom_t  = c.error_com;

norm_tau(tt) = norm(tau_t);

pos   = [pos pos_t];
ecom  = [ecom ecom_t];
tau   = [tau tau_t];
fc    = [fc fc_t];
f0    = [f0 f0_t];

end

%% Visualizer
for k=1:3

figure(4)
hold all
grid on
plot(t,pos(k,:))
title('left foot position')

end

if params.feet_on_ground == 1
    
for k=1:6

figure(5)
hold all
grid on
plot(t,fc(k,:))
title('contact forces')

end

elseif params.feet_on_ground == 2

for k=1:12

figure(5)
hold all
grid on
plot(t,fc(k,:))
title('contact forces')

figure(6)
hold all
grid on
plot(t,f0(k,:))
title('f0')

end

end

%%
for k=1:25

figure(7)
hold all
grid on
plot(t,tau(k,:))
title('torques at joints')

end

for k=1:3

figure(8)
hold all
grid on
plot(t,ecom(k,:))
title('CoM error')

end

figure(9)
hold on
grid on
plot(t,norm_tau)
title('norm of joints torques')

end


