% clear all
  close all
  clc
  
%% This is a program for visualizing old results saved in the "stored test trajectory" file.

%BE CAREFUL! THE PROGRAM MUST BE INITIALIZED USING
%integrateForwardDynamics.m IN ORDER TO SET ALL THE INITIAL PARAMETERS
%CORRECTLY. 

load('storedTestTrajectory.mat')

%% Plot all the quantities desired 
%for now, these are the parameters calculated in ForwardDynamics.m
 tau     = [];
 tau_lin = [];

for tt=1:length(t)
    
[a,c] = forwardDynamics(t(tt), chi(tt,:).', params);

tau_t     = c.tau;
tau_lint  = c.tau_lin;

tau     = [tau tau_t];
tau_lin = [tau_lin tau_lint];

end

%% For linearization
for k=1:5
   
figure(9)
subplot(3,2,k)
plot(t,tau_lin(k+3,:),'r')
hold on
plot(t,tau(k+3,:))
grid on
xlabel('s')
ylabel('Nm')

name = whatname('l_arm',k);
title(name)
legend('Linear','NonLinear')

print -depsc t1

figure(10)
subplot(3,2,k)
plot(t,tau_lin(k+3+5,:),'r')
hold on
plot(t,tau(k+3+5,:))
grid on
xlabel('s')
ylabel('Nm')

name = whatname('r_arm',k);
title(name)
legend('Linear','NonLinear')

print -depsc t2

end

for k=1:6

figure(11)
subplot(3,2,k)
plot(t,tau_lin(k+13,:),'r')
hold on
plot(t,tau(k+13,:))
grid on
xlabel('s')
ylabel('Nm')

name = whatname('l_leg',k);
title(name)
legend('Linear','NonLinear')

print -depsc t3

figure(12)
subplot(3,2,k)
plot(t,tau_lin(k+13+6,:),'r')
hold on
plot(t,tau(k+13+6,:))
grid on
xlabel('s')
ylabel('Nm')

name = whatname('r_leg',k);
title(name)
legend('Linear','NonLinear')

print -depsc t4

end

for k=1:3
    
figure(13)
subplot(3,1,k)
plot(t,tau_lin(k,:),'r')
hold on
plot(t,tau(k,:))
grid on
xlabel('s')
ylabel('Nm')

name = whatname('torso',k);
title(name)  
legend('Linear','NonLinear')

print -depsc t5
end
