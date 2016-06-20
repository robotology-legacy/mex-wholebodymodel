function figureCont = visualizeLinearizAndGains(t,CONFIG,ddqjNonLin,ddqjLin,gainTun)
%VISUALIZELINEARIZANDGAINS visualizes the results on the linearized joint space
%                          dynamics of robot iCub.
%
%   figureCont = VISUALIZELINEARIZANDGAINS(t,CONFIG,ddqjNonLin,ddqjLin,gainTun)
%   takes as input the integration time T, the structure CONFIG containing all
%   the utility parameters, the joint linear and nonlinear accelerations and
%   all the gains matrices (gainTun). It generates all the plots related to the
%   stability analysis, it verifies the soundness of the linearization
%   procedure and visualizes the gains matrices after optimization. The 
%   output is a counter for the automatic correction of figures numbers in 
%   case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% initial parameters
figureCont = CONFIG.figureCont;
ndof       = CONFIG.ndof; 

%% Linearized joint dynamics
if CONFIG.linearizationDebug  == 1

for k=1:5
  
% LEFT ARM    
figure(figureCont)
subplot(3,2,k)
plot(t,ddqjLin(k+3,:))
hold on
plot(t,ddqjNonLin(k+3,:),'r')
grid on
xlabel('Time [s]')
ylabel('Joint Acc [rad/s^2]')
name = whatname('left_arm',k);
title(name)
legend('Lin Acc','NonLin Acc')

% RIGHT ARM
figure(figureCont+1)
subplot(3,2,k)
plot(t,ddqjLin(k+3+5,:))
hold on
plot(t,ddqjNonLin(k+3+5,:),'r')
grid on
xlabel('Time [s]')
ylabel('Joint Acc [rad/s^2]')
name = whatname('right_arm',k);
title(name)
legend('Lin Acc','NonLin Acc')
end

figureCont = figureCont +2;

for k=1:6

% LEFT LEG
figure(figureCont)
subplot(3,2,k)
plot(t,ddqjLin(k+13,:))
hold on
plot(t,ddqjNonLin(k+13,:),'r')
grid on
xlabel('Time [s]')
ylabel('Joint Acc [rad/s^2]')
name = whatname('left_leg',k);
title(name)
legend('Lin Acc','NonLin Acc')

% RIGHT LEG
figure(figureCont+1)
subplot(3,2,k)
plot(t,ddqjLin(k+13+6,:))
hold on
plot(t,ddqjNonLin(k+13+6,:),'r')
grid on
xlabel('Time [s]')
ylabel('Joint Acc [rad/s^2]')
name = whatname('right_leg',k);
title(name)
legend('Lin Acc','NonLin Acc')
end

figureCont  = figureCont +2;

for k=1:3
    
% TORSO
figure(figureCont)
subplot(3,1,k)
plot(t,ddqjLin(k,:))
hold on
plot(t,ddqjNonLin(k,:),'r')
grid on
xlabel('Time [s]')
ylabel('Joint Acc [rad/s^2]')
name = whatname('torso',k);
title(name)
legend('Lin Acc','NonLin Acc')
end

figureCont = figureCont +1;
end

%% Gains Tuning results
if CONFIG.gains_tuning == 1 && CONFIG.visualize_gains_tuning_results  == 1

figureGains   = [figureCont+1;figureCont+2;figureCont+3;figureCont+4];

% STATE MATRIX
AStateDes     = [zeros(ndof)      eye(ndof);
                -gainTun.KSdes   -gainTun.KDdes];
                

AStateNew     = [zeros(ndof)     eye(ndof);
                -gainTun.KSn     -gainTun.KDn];

% MATRIX SHAPE VERIFICATION            
figure(figureGains(1))
% subplot(1,2,1)
% surf(-AStateDes)
% hold on
% title('Desired State Matrix (-State)')
% subplot(1,2,2)
surf(-AStateNew)
% image(-AStateNew,'CDataMapping','scaled')
% hold on
title('Obtained State Matrix (-State)')

% EIGENVALUES
% figure(figureGains(2))
% plot(real(eig(AStateDes)),imag(eig(AStateDes)),'xb')
% hold on
% grid on
% plot(real(eig(AStateNew)),imag(eig(AStateNew)),'xr')
% title('Root Locus')
% legend('Des eigenval','Real eigenval')
% xlabel('Real')
% ylabel('Imag')

% NEW GAINS MATRICES
% % % figure(figureGains(1))
% % % subplot(2,2,1) 
% % % image(gainTun.impedances,'CDataMapping','scaled')
% % % % colorbar
% % % % surf(gainTun.impedances)
% % % % zlim([-30 90])
% % % % hold on
% % % title('Opt Impedances')
% % % subplot(2,2,2) 
% % % image(gainTun.dampings,'CDataMapping','scaled')
% % % % colorbar
% % % % surf(gainTun.dampings)
% % % zlim([-10 15])
% % % % hold on
% % % title('Opt Dampings')
% % % % 
% % % % figure(figureGains(4))
% % % subplot(2,2,3)
% % % image(gainTun.intMomentumGains,'CDataMapping','scaled')
% % % % colorbar
% % % % surf(gainTun.intMomentumGains)
% % % % hold on
% % % title('Opt Momentum Integral Gains')
% % % subplot(2,2,4)
% % % image(gainTun.MomentumGains,'CDataMapping','scaled')
% % % % colorbar
% % % % surf(gainTun.MomentumGains)
% % % % hold on
% % % title('Opt Momentum Gains')
end

end