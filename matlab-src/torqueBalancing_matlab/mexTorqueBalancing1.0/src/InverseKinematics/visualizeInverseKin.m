%% visualizeInverseKin
% visualizes the results of the inverse kinematic integration. In
% particular, the first and second task are considered: it will be
% evaluated if the joints references allow to reach the first and second
% task
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function [] = visualizeInverseKin(params,ikinParam)

set(0,'DefaultFigureWindowStyle','Docked');
t                 = ikinParam.t;
CoMTrajectory     = ikinParam.CoMTrajectory;
CoM_Feet_Error    = ikinParam.CoM_Feet_Error;

%% CoM trajectory error
title_graphics  = {'X direction' 'Y direction' 'Z direction'};
title2          = {'Error on desired x CoM trajectory' 'Error on desired y CoM trajectory' 'Error on desired z CoM trajectory'};

for k = 1:3
% desired and obtained trajectory at CoM
% position
figure(32)
subplot(3,1,k)
plot(t,CoMTrajectory(k,:),'b')
hold on
grid on
plot(t,CoMTrajectory(k+9,:),'r')

xlabel('s')
ylabel('m')
title(title_graphics(k))
legend('desired CoM pos','obtained CoM pos')

% velocity
figure(33)
subplot(3,1,k)
plot(t,CoMTrajectory(k+3,:),'b')
hold on
grid on
plot(t,CoMTrajectory(k+9+3,:),'r')

xlabel('s')
ylabel('m/s')
title(title_graphics(k))
legend('desired CoM vel','obtained CoM vel')

% acceleration
figure(34)
subplot(3,1,k)
plot(t,CoMTrajectory(k+6,:),'b')
hold on
grid on
plot(t,CoMTrajectory(k+9+6,:),'r')

xlabel('s')
ylabel('m/s^2')
title(title_graphics(k))
legend('desired CoM acc','obtained CoM acc')

% error on desired CoM trajectory
figure(35)
subplot(3,1,k)
plot(t,CoM_Feet_Error(k,:),'b')
hold on
grid on
title(title2(k))
xlabel('s')
ylabel('m')
end

%% error on desired feet pos and orient
tit3 = {'Error on desired left foot pos along x axis','Error on desired left foot pos along y axis','Error on desired left foot pos along z axis',...
        'Error on desired left foot orient along x axis','Error on desired left foot orient along y axis','Error on desired left foot orient along z axis',...
        'Error on desired right foot pos along x axis','Error on desired right foot pos along y axis','Error on desired right foot pos along z axis',...
        'Error on desired right foot orient along x axis','Error on desired right foot orient along y axis','Error on desired right foot orient along z axis'};
    
tit4 = {'m' 'm' 'm' 'rad' 'rad' 'rad'};

if     params.numConstraints == 1
    
for k=1:6

figure(36)
subplot(3,2,k)
plot(t,CoM_Feet_Error(k,:))
grid on
title(tit3(k))
xlabel('s')
ylabel(tit4(k))
end

elseif params.numConstraints  == 2

for k=1:6

figure(36)
subplot(3,2,k)
plot(t,CoM_Feet_Error(k,:))
grid on
title(tit3(k))
xlabel('s')
ylabel(tit4(k))

figure(37)
subplot(3,2,k)
plot(t,CoM_Feet_Error(k+6,:))
grid on
title(tit3(k+6))
xlabel('s')
ylabel(tit4(k))
end
end    

set(0,'DefaultFigureWindowStyle','Normal');
end
