function [] = visualizer_graphics(t,chi,params)

%Plot all the desired quantities
%for now, these are the parameters calculated in ForwardDynamics.m
qj     = [];
fc     = [];
f0     = [];
tau    = [];
ecom   = [];
pos    = [];

for tt=1:length(t)
    
[~,c] = forwardDynamics(t(tt), chi(tt,:).', params);

qj_t    = c.qj;
pos_t   = c.pos_feet(1:7);
fc_t    = c.fc;
f0_t    = c.f0;
tau_t   = c.tau;
ecom_t  = c.error_com;

norm_tau(tt) = norm(tau_t);

qj    = [qj   qj_t];
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

figure(5)
hold all
grid on
plot(t,pos(k+4,:))
title('left foot orientation')

end

if params.feet_on_ground == 1
    
for k=1:6

figure(6)
hold all
grid on
plot(t,fc(k,:))
title('contact forces')

end

elseif params.feet_on_ground == 2

for k=1:12

figure(6)
hold all
grid on
plot(t,fc(k,:))
title('contact forces')

end

for k=1:6
    
figure(7)
hold all
grid on
plot(t,f0(k,:))
title('f0')

end

end

%% Torques and CoM
for k=1:25

figure(8)
hold all
grid on
plot(t,tau(k,:))
title('torques at joints')

end

for k=1:3

figure(9)
hold all
grid on
plot(t,ecom(k,:))
title('CoM error')

end

figure(10)
hold on
grid on
plot(t,norm_tau)
title('norm of joints torques')

%% Joints positions
for k=1:5
   
figure(11)
subplot(3,2,k)
plot(t,qj(k+3,:))
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('l_arm',k);
title(name)

figure(12)
subplot(3,2,k)
plot(t,qj(k+3+5,:))
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('r_arm',k);
title(name)

end

for k=1:6

figure(13)
subplot(3,2,k)
plot(t,qj(k+13,:))
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('l_leg',k);
title(name)

figure(14)
subplot(3,2,k)
plot(t,qj(k+13+6,:))
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('r_leg',k);
title(name)

end

for k=1:3
    
figure(15)
subplot(3,1,k)
plot(t,qj(k,:))
hold on
grid on
xlabel('s')
ylabel('rad')

name = whatname('torso',k);
title(name)  

end

end
