function figureCont = visualizeJointDynamics(t,CONFIG,qj,qjRef,tAss)
%VISUALIZEJOINTDYNAMICS visualizes the joint space dynamics of the iCub robot
%                       from forward dynamics integration.
%
%   figureCont = VISUALIZEJOINTDYNAMICS(t,config,qj,qjRef) takes as inputs the
%   integration time T, a structure CONFIG which contains all the utility
%   parameters, the joint position QJ and the desired joint position QJREF.
%   The output is a counter for the automatic correction of figures numbers
%   in case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% setup parameters
figureCont  = CONFIG.figureCont;

for k = 1:length(tAss)
    
    index(k) = sum(t>tAss(k))+1;
end

qj     = 180/pi*qj;
qjRef  = 180/pi*qjRef;
step   = qjRef(:,end)-qj(:,1);

%% Joints dynamics
for k=1:5
  
% LEFT ARM    
figure(figureCont)
subplot(3,2,k)
plot(t,qj(k+3,:))
hold on
plot(tAss(k+3),qj(k+3,end-index(k+3)),'ok')
hold on
plot(t,qjRef(k+3,:),'k')
plot(t,qjRef(k+3,:)+0.05*step(k+3),'-')
plot(t,qjRef(k+3,:)-0.05*step(k+3),'-')
grid on
xlabel('Time [s]')
ylabel('Angle [rad]')
name = whatname('left_arm',k);
title(name)
legend('qj','qjRef')

% RIGHT ARM
figure(figureCont+1)
subplot(3,2,k)
plot(t,qj(k+3+5,:))
hold on
plot(tAss(k+3+5),qj(k+3+5,end-index(k+3+5)),'ok')
hold on
plot(t,qjRef(k+3+5,:),'k')
plot(t,qjRef(k+3+5,:)+0.05*step(k+3+5),'-')
plot(t,qjRef(k+3+5,:)-0.05*step(k+3+5),'-')
grid on
xlabel('Time [s]')
ylabel('Angle [rad]')
name = whatname('right_arm',k);
title(name)
legend('qj','qjRef')
end

figureCont = figureCont +2;

for k=1:6

% LEFT LEG
figure(figureCont)
subplot(3,2,k)
plot(t,qj(k+13,:))
hold on
plot(tAss(k+13),qj(k+13,end-index(k+13)),'ok')
hold on
plot(t,qjRef(k+13,:),'k')
grid on
xlabel('Time [s]')
ylabel('Angle [rad]')
name = whatname('left_leg',k);
title(name)
legend('qj','qjRef')

% RIGHT LEG
figure(figureCont+1)
subplot(3,2,k)
plot(t,qj(k+13+6,:))
hold on
plot(tAss(k+13+6),qj(k+13+6,end-index(k+13+6)),'ok')
hold on
plot(t,qjRef(k+13+6,:),'k')
grid on
xlabel('Time [s]')
ylabel('Angle [rad]')
name = whatname('right_leg',k);
title(name)
legend('qj','qjRef')
end

figureCont = figureCont +2;

for k=1:3
    
% TORSO
figure(figureCont)
subplot(3,1,k)
plot(t,qj(k,:))
hold on
plot(tAss(k),qj(k,end-index(k)),'ok')
hold on
plot(t,qjRef(k,:),'k')
plot(t,(qjRef(k,:)+0.05*step(k)),'g')
plot(t,(qjRef(k,:)-0.05.*step(k)),'g')
grid on
xlabel('Time [s]')
ylabel('Angle [rad]')
name = whatname('torso',k);
title(name)
legend('qj','qjRef')
end

figureCont = figureCont +1;

end
