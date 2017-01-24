function figureCont = visualizeForwardDyn(t,CONFIG,xCoM,poseFeet,fc,f0,tau_norm,CoP,HErr)
%VISUALIZEFORWARDDYN visualizes the results of the iCub forward dynamics
%                    integration.
%
% VISUALIZEFORWARDDYN plots some outputs of the forward dynamics integrator, 
% such as the robot state, the contact forces, the control torques, the error 
% on centroidal momentum.
%     
% figureCont = VISUALIZEFORWARDDYN(t,CONFIG,xCoM,poseFeet,fc,f0,tau_norm,CoP,HErr)
% takes as input the integration time t, the structure CONFIG which
% contains all the utility parameters and a list of results from the forward
% dynamics integration (from xCoM to HErr). The output is a counter for
% the automatic correction of figures numbers in case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% setup parameters
figureCont = CONFIG.figureCont;

%% CoM trajectory
figure(figureCont)
set(gcf,'numbertitle','off','name','CoM traj')
plot3(xCoM(1,:),xCoM(2,:),xCoM(3,:));
hold on
plot3(xCoM(1,1),xCoM(2,1),xCoM(3,1),'ro');
grid on;
title('CoM Trajectory')

axis equal
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
figureCont = figureCont +1;

%% Feet position and orientation
if CONFIG.feet_on_ground(1) == 1
    
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Lfoot errors')
    subplot(1,2,1)
    hold all
    grid on
    plot(t,poseFeet(1:3,:))
    title('Left Foot Position')
    xlabel(' Time [s]')
    ylabel('Position [m]')
    
    figure(figureCont)
    subplot(1,2,2)
    hold all
    grid on
    plot(t,poseFeet(4:6,:))
    title('Left Foot Orientation')
    xlabel('Time [s]')
    ylabel('Angle [rad]')
else
    
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Rfoot errors')
    subplot(1,2,1)
    hold all
    grid on
    plot(t,poseFeet(7:9,:))
    title('Right Foot Position')
    xlabel('Time [s]')
    ylabel('Position [m]')
    
    figure(figureCont)
    subplot(1,2,2)
    hold all
    grid on
    plot(t,poseFeet(10:12,:))
    title('Right Foot Orientation')
    xlabel('Time [s]')
    ylabel('Angle [rad]')
end

figureCont = figureCont +1;

%% Contact forces
figure(figureCont)
set(gcf,'numbertitle','off','name','Contact forces')
hold all
grid on
plot(t,fc)
title('Contact Wrenches')
xlabel('Time [s]')
ylabel('f_{c} [wrench]')

figureCont = figureCont +1;

if sum(CONFIG.feet_on_ground) == 2
    
    figure(figureCont)
    set(gcf,'numbertitle','off','name','f null')
    hold all
    grid on
    plot(t,f0)
    title('Contact Wrenches Nullspace')
    xlabel('Time [s]')
    ylabel('f_{0} [wrench]')
    
    figureCont = figureCont +1;
end

%% Control torques
figure(figureCont)
set(gcf,'numbertitle','off','name','Norm of torques')
hold on
grid on
plot(t,tau_norm)
title('Square norm of joints torques')
xlabel('Time [s]')
ylabel('Torque [Nm]')

figureCont = figureCont +1;

%% Centers of Pressure at feet
if sum(CONFIG.feet_on_ground) == 2
    
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Feet CoP')
    subplot(1,2,1)
    plot(CoP(1,:),CoP(2,:))
    hold on
    grid on
    subplot(1,2,1)
    plot(CoP(1,1),CoP(2,1),'or')
    title('Left foot CoP')
    xlabel('Y direction [m]')
    ylabel('X direction [m]')
    axis([CoP(1,1)+CONFIG.footSize(2,1) CoP(1,1)+CONFIG.footSize(2,2) CoP(2,1)+CONFIG.footSize(1,1) CoP(2,1)+CONFIG.footSize(1,2)])
    
    figure(figureCont)
    subplot(1,2,2)
    plot(CoP(3,:),CoP(4,:))
    hold on
    grid on
    subplot(1,2,2)
    plot(CoP(3,1),CoP(4,1),'or')
    title('Right foot CoP')
    xlabel('Y direction [m]')
    ylabel('X direction [m]')
    axis([CoP(3,1)+CONFIG.footSize(2,1) CoP(3,1)+CONFIG.footSize(2,2) CoP(4,1)+CONFIG.footSize(1,1) CoP(4,1)+CONFIG.footSize(1,2)])
    
else
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Foot CoP')
    plot(CoP(1,:),CoP(2,:))
    hold on
    grid on
    plot(CoP(1,1),CoP(2,1),'or')
    title('Foot CoP')
    xlabel('Y direction [m]')
    ylabel('X direction [m]')
    axis([CoP(1,1)+CONFIG.footSize(2,1) CoP(1,1)+CONFIG.footSize(2,2) CoP(2,1)+CONFIG.footSize(1,1) CoP(2,1)+CONFIG.footSize(1,2)])
    
end

figureCont = figureCont +1;

%% Centroidal momentum error
figure(figureCont)
set(gcf,'numbertitle','off','name','H error')
subplot(2,1,1)
hold all
grid on
plot(t,HErr(1:3,:))
xlabel('Time [s]')
ylabel('H_{Lin}-H_{Lin}^{d}')
title('Linear Momentum Error')

figure(figureCont)
subplot(2,1,2)
hold all
grid on
plot(t,HErr(4:6,:))
xlabel('Time [s]')
ylabel('H_{Ang}-H_{Ang}^{d}')
title('Angular Momentum Error')

figureCont = figureCont +1;

end
