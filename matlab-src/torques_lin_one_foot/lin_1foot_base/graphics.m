function [] = graphics(t,tau,tau_lin,t0,tau_reg)

% For linearization
for k=1:5
   
figure(9)
subplot(3,2,k)
plot(t(k+3,:),tau_lin(k+3,:),'r')
hold on
plot(t(k+3,:),tau(k+3,:))
plot(t0(k+3),tau_reg(k+3),'ok')
grid on
xlabel('deg')
ylabel('Nm')

name = whatname('l_arm',k);
title(name)
legend('Linear','NonLinear','qj0')

figure(10)
subplot(3,2,k)
plot(t(k+3+5,:),tau_lin(k+3+5,:),'r')
hold on
plot(t(k+3+5,:),tau(k+3+5,:))
plot(t0(k+3+5),tau_reg(k+3+5),'ok')
grid on
xlabel('deg')
ylabel('Nm')

name = whatname('r_arm',k);
title(name)
legend('Linear','NonLinear','qj0')

end

for k=1:6

figure(11)
subplot(3,2,k)
plot(t(k+13,:),tau_lin(k+13,:),'r')
hold on
plot(t(k+13,:),tau(k+13,:))
plot(t0(k+13),tau_reg(k+13),'ok')
grid on
xlabel('deg')
ylabel('Nm')

name = whatname('l_leg',k);
title(name)
legend('Linear','NonLinear','qj0')

figure(12)
subplot(3,2,k)
plot(t(k+13+6,:),tau_lin(k+13+6,:),'r')
hold on
plot(t(k+13+6,:),tau(k+13+6,:))
plot(t0(k+13+6),tau_reg(k+13+6),'ok')
grid on
xlabel('deg')
ylabel('Nm')

name = whatname('r_leg',k);
title(name)
legend('Linear','NonLinear','qj0')

end

for k=1:3
    
figure(13)
subplot(3,1,k)
plot(t(k,:),tau_lin(k,:),'r')
hold on
plot(t(k,:),tau(k,:))
plot(t0(k),tau_reg(k),'ok')
grid on
xlabel('deg')
ylabel('Nm')

name = whatname('torso',k);
title(name)  
legend('Linear','NonLinear','qj0')


end