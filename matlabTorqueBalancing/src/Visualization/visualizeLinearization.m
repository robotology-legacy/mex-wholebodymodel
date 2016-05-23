function ContFig = visualizeLinearization(t,config,qjErr,dqjErr,ddqjNonLin,ddqjRef)
%VISUALIZELINEARIZATION visualizes the results on the linearized joint space
%                       dynamics of robot iCub.
%   ContFig = VISUALIZELINEARIZATION(t,config,qjErr,dqjErr,ddqjNonLin,ddqjRef) 
%   takes as input the integration time T, the structure CONFIG containing all
%   the utility parameters, the joint error dynamics and the joint linear
%   and nonlinear accelerations. It generates all the plots related to the
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
ContFig = config.ContFig;
ndof    = config.ndof; 

%% Linearized joint dynamics
if config.visualize_stability_analysis_results == 1
    
if config.linearize_for_gains_tuning == 1
    
ddqjLin = ddqjRef - config.visualizeTuning.KSn*qjErr - config.visualizeTuning.KDn*dqjErr;
else 
ddqjLin = ddqjRef - config.linearization.KS*qjErr  - config.linearization.KD*dqjErr;
end

for k=1:5
  
% LEFT ARM    
figure(ContFig)
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
figure(ContFig+1)
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

ContFig = ContFig +2;

for k=1:6

% LEFT LEG
figure(ContFig)
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
figure(ContFig+1)
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

ContFig  = ContFig +2;

for k=1:3
    
% TORSO
figure(ContFig)
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

ContFig = ContFig +1;
end
           
%% Gains Tuning results
if config.visualize_gains_tuning_results  == 1 && config.linearize_for_gains_tuning == 1

% STATE MATRIX
AStateDes     = [zeros(ndof)                     eye(ndof);
                -config.visualizeTuning.KSdes   -config.visualizeTuning.KDdes];
                

AStateNew     = [zeros(ndof)                     eye(ndof);
                -config.visualizeTuning.KSn     -config.visualizeTuning.KDn];

% MATRIX SHAPE VERIFICATION            
figure(ContFig)
subplot(1,2,1)
surf(-AStateDes)
hold on
title('Desired State Matrix (-State)')
subplot(1,2,2)
surf(-AStateNew)
hold on
title('Obtained State Matrix (-State)')

ContFig = ContFig +1;

% EIGENVALUES
figure(ContFig)
plot(real(eig(AStateDes)),imag(eig(AStateDes)),'xb')
hold on
grid on
plot(real(eig(AStateNew)),imag(eig(AStateNew)),'xr')
title('Root Locus')
legend('Des eigenval','Real eigenval')
xlabel('Real')
ylabel('Imag')

ContFig = ContFig +1;

% NEW GAINS MATRICES
figure(ContFig)
subplot(1,2,1)
surf(config.visualizeTuning.Kpn)
hold on
title('Opt Impedances')
subplot(1,2,2)
surf(config.visualizeTuning.Kdn)
hold on
title('Opt Dampings')

ContFig = ContFig +1;

figure(ContFig)
subplot(1,2,1)
surf(config.visualizeTuning.Kpx)
hold on
title('Opt Momentum Integral Gains')
subplot(1,2,2)
surf(config.visualizeTuning.Kdx)
hold on
title('Opt Momentum Gains')

ContFig = ContFig +1;
end

end
