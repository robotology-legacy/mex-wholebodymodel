%% visIntegrationPlot
% generates graphics from the forward dynamics integration of the robot iCub.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function [] = visIntegrationPlot(t,params,xCoM,pos_feet,phi_lfoot,phi_rfoot,fc,f0,tau,error_CoM,norm_tau,CoP,norm_qjErr,norm_HErr)
%% Plot CoM trajectory
figure(7)
plot3(xCoM(1,:),xCoM(2,:),xCoM(3,:));
hold on
plot3(xCoM(1,1),xCoM(2,1),xCoM(3,1),'ro');
grid on;
title('CoM position')

%axis square;
axis equal
    
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');

%% Feet position and orientation
for k=1:3

if params.feet_on_ground(1) == 1  
    
figure(8)
subplot(1,2,1)
hold all
grid on
plot(t,pos_feet(k,:))
title('Left foot position')
xlabel('s')
ylabel('m')

figure(8)
subplot(1,2,2)
hold all
grid on
plot(t,phi_lfoot(k,:))
title('Left foot orientation')
xlabel('s')
ylabel('rad')
else
    
figure(8)
subplot(1,2,1)
hold all
grid on
plot(t,pos_feet(k+7,:))
title('Right foot position')
xlabel('s')
ylabel('m')

figure(8)
subplot(1,2,2)
hold all
grid on
plot(t,phi_rfoot(k,:))
title('Right foot orientation')
xlabel('s')
ylabel('rad')
end
end

%% Contact forces
if sum(params.feet_on_ground) == 1

for k=1:6

figure(9)
hold all
grid on
plot(t,fc(k,:))
title('Contact forces and moments')
xlabel('s')
ylabel('N,Nm')
end

elseif sum(params.feet_on_ground) == 2

for k=1:12

figure(9)
hold all
grid on
plot(t,fc(k,:))
title('Contact forces and moments')
xlabel('s')
ylabel('N,Nm')

figure(10)
hold all
grid on
plot(t,f0(k,:))
title('Contact forces and moments null space')
xlabel('s')
ylabel('N,Nm')
end
end

%% Control torques and CoM error
for k=1:params.ndof

figure(11)
hold all
grid on
plot(t,tau(k,:))
title('Torques at joints')
xlabel('s')
ylabel('Nm')
end

for k=1:3

figure(12)
hold all
grid on
plot(t,error_CoM(k,:))
title('CoM error')
xlabel('s')
ylabel('m')
end

figure(13)
hold on
grid on
plot(t,norm_tau)
title('Square norm of joints torques')
xlabel('s')
ylabel('Nm')

%% CoP at feet 
for k=1:2

if sum(params.feet_on_ground) == 2

figure(14)
subplot(1,2,1)
plot(CoP(1,:),CoP(2,:))
hold on
grid on 
subplot(1,2,1)
plot(CoP(1,1),CoP(2,1),'or')
title('Left foot CoP')
xlabel('Y direction (m)')
ylabel('X direction (m)')
axis([-0.1 0.1 -0.1 0.1])

figure(14)
subplot(1,2,2)
plot(CoP(3,:),CoP(4,:))
hold on
grid on
subplot(1,2,2)
plot(CoP(3,1),CoP(4,1),'or')
title('Right foot CoP')
xlabel('Y direction (m)')
ylabel('X direction (m)')
axis([-0.1 0.1 -0.1 0.1])

else
figure(14)
plot(CoP(1,:),CoP(2,:))
hold on
grid on
plot(CoP(1,1),CoP(2,1),'or')
title('Foot CoP')
xlabel('Y direction (m)')
ylabel('X direction (m)')
axis([-0.1 0.1 -0.1 0.1])
end
end

%% Square norm of joints positions
figure(15)
hold on
grid on
plot(t,norm_qjErr)
xlabel('s')
ylabel('rad')
title('Square norm of joints pos error')

%% Square norm of H error
figure(16)
hold on
grid on
plot(t,norm_HErr)
xlabel('s')
ylabel('|H-H_{ref}|')
title('Square norm of H error')

end
