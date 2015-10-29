function [] = visualizer_graphics(t,chi,params)

%Plot all the desired quantities
%for now, these are the parameters calculated in ForwardDynamics.m
qj     = [];
fc     = [];
f0     = [];
tau    = [];
ecom   = [];
pos    = [];
traj   = [];
delta  = [];
CoP    = [];

for tt=1:length(t)
    
[~,c] = forwardDynamics(t(tt), chi(tt,:).', params);

qj_t    = c.qj;
pos_t   = c.pos_feet(1:7);
fc_t    = c.fc;
f0_t    = c.f0;
tau_t   = c.tau;
ecom_t  = c.error_com;
traj_t  = c.traj;
delta_t = c.delta;
CoP_t   = c.CoP;

norm_tau(tt) = norm(tau_t);

qj    = [qj   qj_t];
pos   = [pos pos_t];
ecom  = [ecom ecom_t];
tau   = [tau tau_t];
fc    = [fc fc_t];
f0    = [f0 f0_t];
traj  = [traj traj_t];
delta = [delta delta_t];
CoP   = [CoP CoP_t];

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

figure(16)
subplot(3,1,k)
plot(t,traj(k,:),'b')
hold on
grid on
plot(t,traj(k+9,:),'r')
xlabel('s')
ylabel('m')
legend('desired CoM pos','obtained CoM pos')

figure(17)
subplot(3,1,k)
plot(t,traj(k+3,:),'b')
hold on
grid on
plot(t,traj(k+9+3,:),'r')
xlabel('s')
ylabel('m/s')
legend('desired CoM vel','obtained CoM vel')

figure(18)
subplot(3,1,k)
plot(t,traj(k+6,:),'b')
hold on
grid on
plot(t,traj(k+9+6,:),'r')
xlabel('s')
ylabel('m/s^2')
legend('desired CoM acc','obtained CoM acc')

end

for k = 1:3
    
figure(19)
subplot(3,1,k)
plot(t,delta(k,:),'b')
hold on
grid on

end


if params.feet_on_ground == 1
    
for k=1:6

figure(20)
subplot(3,2,k)
plot(t,delta(k,:))
grid on
title('Left foot delta')

end

elseif params.feet_on_ground == 2

for k=1:6

figure(20)
subplot(3,2,k)
plot(t,delta(k,:))
grid on
title('Left foot delta')

figure(21)
subplot(3,2,k)
plot(t,delta(k+6,:))
grid on
title('Right foot delta')

end

end

for k=1:2

figure(22)
plot(t,CoP(k,:))
hold on
grid on
title('Left foot CoP')  

if params.feet_on_ground == 2

figure(23)
plot(t,CoP(k+2,:))
hold on
grid on
title('Right foot CoP')  

end

end

end
