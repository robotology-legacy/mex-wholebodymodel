%% visLinResults
%
% Visualization of the results on the stability analysis and the gains
% scheduling procedure
%
function [] =  visLinResults(t,params,qj,qjInit,dqj,ddqjIdeal,ddqjRef)
%% Stability analysis
if params.visualize_stability_analysis_plot == 1
    
% Linearized joints accelerations
if params.linearize_for_gains_tuning == 1
    
ddqjLin = ddqjRef - params.linearization.KSn*(qj-qjInit) - params.linearization.KDn*dqj;

else
    
ddqjLin = ddqjRef  - params.linearization.KS*(qj-qjInit) - params.linearization.KD*dqj;

end

for k=1:5
  
% Robot arms    
figure(22)
subplot(3,2,k)
plot(t,ddqjLin(k+3,:))
hold on
plot(t,ddqjIdeal(k+3,:),'r')
grid on
xlabel('s')
ylabel('rad')
name = whatname('left_arm',k);
title(name)
legend('LinAcc','NonLinAcc')

figure(23)
subplot(3,2,k)
plot(t,ddqjLin(k+3+5,:))
hold on
plot(t,ddqjIdeal(k+3+5,:),'r')
grid on
xlabel('s')
ylabel('rad')
name = whatname('right_arm',k);
title(name)
legend('LinAcc','NonLinAcc')

end

for k=1:6

% Robot legs
figure(24)
subplot(3,2,k)
plot(t,ddqjLin(k+13,:))
hold on
plot(t,ddqjIdeal(k+13,:),'r')
grid on
xlabel('s')
ylabel('rad')
name = whatname('left_leg',k);
title(name)
legend('LinAcc','NonLinAcc')

figure(25)
subplot(3,2,k)
plot(t,ddqjLin(k+13+6,:))
hold on
plot(t,ddqjIdeal(k+13+6,:),'r')
grid on
xlabel('s')
ylabel('rad')
name = whatname('right_leg',k);
title(name)
legend('LinAcc','NonLinAcc')

end

for k=1:3
    
% Robot torso
figure(26)
subplot(3,1,k)
plot(t,ddqjLin(k,:))
hold on
plot(t,ddqjIdeal(k,:),'r')
grid on
xlabel('s')
ylabel('rad')
name = whatname('torso',k);
title(name)  
legend('LinAcc','NonLinAcc')

end

end
           
%% Gains Scheduling
if params.visualize_gains_tuning_plot  == 1 && params.linearize_for_gains_tuning == 1

figure(27)
subplot(1,2,1)
surf(params.linearization.KSdes)
hold on
title('Desired KS')
subplot(1,2,2)
surf(params.linearization.KDdes)
hold on
title('Desired KD')

figure(28)
subplot(1,2,1)
surf(params.linearization.KS)
hold on
title('Original KS')
subplot(1,2,2)
surf(params.linearization.KD)
hold on
title('Original KD')

figure(29)
subplot(1,2,1)
surf(params.linearization.KSn)
hold on
title('Obtained KS')
subplot(1,2,2)
surf(params.linearization.KDn)
hold on
title('Obtained KD')

figure(30)
subplot(1,2,1)
surf(params.linearization.Kpn)
hold on
title('impedances')
subplot(1,2,2)
surf(params.linearization.Kdn)
hold on
title('dampings')

figure(31)
subplot(1,2,1)
surf(params.linearization.Kpx)
hold on
title('first task position gains')
subplot(1,2,2)
surf(params.linearization.Kdx)
hold on
title('first task velocity gains')

end

end
