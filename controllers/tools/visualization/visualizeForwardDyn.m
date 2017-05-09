function figureCounter = visualizeForwardDyn(timeTot,fc,H,HRef,poseFeet,qj,qjRef,tau_xi,xCoM,xCoMDes,...
                                             positiveIndex,listPlot,figureCounter)
%VISUALIZEFORWARDDYN visualizes the results of the robot forward dynamics
%                    integration, such as the robot state, the contact forces,
%                    the control torques, the error on centroidal momentum.
%
% Format: figureCounter = visualizeForwardDyn(timeTot,fc,H,HRef,poseFeet,qj,qjRef,tau_xi,xCoM,
%                                             xCoMDes,positiveIndex,listPlot,figureCounter)
%
% Inputs:  - timeTot: time vector;
%          - fc: contact forces;
%          - H: centroidal momentum;
%          - HRef: centroidal momentum reference;
%          - qj: joint positions;
%          - qjRef: joint positions reference;
%          - tau_xi: control torques;
%          - xCoM: CoM position;
%          - xCoMDes: CoM desired position;
%          - positiveIndex: index of the values that are going to be plotted;
%          - listPlot: list of figures to show;
%          - figureCounter: a counter for updating figures number.  
%
% Output:  - figureCounter: a counter for updating figures number
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
time                = timeTot(positiveIndex);
% center of pressure at feet. In case of one foot balancing, only the
% first two elements are different from zero
CoP                 =  zeros(4,length(time));
CoP(1,:)            =  fc(4,positiveIndex)/fc(3,positiveIndex);
CoP(2,:)            = -fc(5,positiveIndex)/fc(3,positiveIndex); 
for k = 1:length(time)
    if fc(9,k) ~= 0
        CoP(3,k)    =  fc(10,k)/fc(9,k);
        CoP(4,k)    = -fc(11,k)/fc(9,k);
    else
        CoP(3,k)    =  0;
        CoP(4,k)    =  0;
    end
end

%% %%%%%%%%%%%%%%%%%%%%%%%%% CoM trajectory %%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
% if selected in the list, plot the CoM trajectory
if sum(listPlot == 3)
     figure(figureCounter)
     set(gcf,'numbertitle','off','name','CoM trajectory')
     plot3(xCoM(1,positiveIndex),xCoM(2,positiveIndex),xCoM(3,positiveIndex));
     hold on
     plot3(xCoMDes(1,positiveIndex),xCoMDes(2,positiveIndex),xCoMDes(3,positiveIndex),'r');     
     plot3(xCoM(1,1),xCoM(2,1),xCoM(3,1),'go');
     grid on;
     title('CoM Trajectory')
     axis equal
     xlabel('X [m]');
     ylabel('Y [m]');
     zlabel('Z [m]');
     figureCounter = figureCounter +1;
     legend('CoM trajectory','CoM reference')
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%% Feet pose %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%
if CONFIG.feet_on_ground(1) == 1
    
    figure(figureCounter)
    set(gcf,'numbertitle','off','name','Lfoot errors')
    subplot(1,2,1)
    hold all
    grid on
    plot(t,poseFeet(1:3,:))
    title('Left Foot Position')
    xlabel(' Time [s]')
    ylabel('Position [m]')
    
    figure(figureCounter)
    subplot(1,2,2)
    hold all
    grid on
    plot(t,poseFeet(4:6,:))
    title('Left Foot Orientation')
    xlabel('Time [s]')
    ylabel('Angle [rad]')
else
    
    figure(figureCounter)
    set(gcf,'numbertitle','off','name','Rfoot errors')
    subplot(1,2,1)
    hold all
    grid on
    plot(t,poseFeet(7:9,:))
    title('Right Foot Position')
    xlabel('Time [s]')
    ylabel('Position [m]')
    
    figure(figureCounter)
    subplot(1,2,2)
    hold all
    grid on
    plot(t,poseFeet(10:12,:))
    title('Right Foot Orientation')
    xlabel('Time [s]')
    ylabel('Angle [rad]')
end

figureCounter = figureCounter +1;

%% Contact forces
figure(figureCounter)
set(gcf,'numbertitle','off','name','Contact forces')
hold all
grid on
plot(t,fc)
title('Contact Wrenches')
xlabel('Time [s]')
ylabel('f_{c} [wrench]')

figureCounter = figureCounter +1;

if sum(CONFIG.feet_on_ground) == 2
    
    figure(figureCounter)
    set(gcf,'numbertitle','off','name','f null')
    hold all
    grid on
    plot(t,f0)
    title('Contact Wrenches Nullspace')
    xlabel('Time [s]')
    ylabel('f_{0} [wrench]')
    
    figureCounter = figureCounter +1;
end

%% Control torques
figure(figureCounter)
set(gcf,'numbertitle','off','name','Norm of torques')
hold on
grid on
plot(t,tau_norm)
title('Square norm of control torques')
xlabel('Time [s]')
ylabel('Torque [Nm]')

figureCounter = figureCounter +1;

%% Centers of Pressure at feet
if sum(CONFIG.feet_on_ground) == 2
    
    figure(figureCounter)
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
    
    figure(figureCounter)
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
    figure(figureCounter)
    set(gcf,'numbertitle','off','name','Foot CoP')
    plot(CoP(1,:),CoP(2,:))
    hold on
    grid on
    plot(CoP(1,1),CoP(2,1),'or')
    title('Foot CoP')
    xlabel('Y direction [m]')
    ylabel('X direction [m]')
    axis([poseFeet(1)+CONFIG.footSize(2,1) poseFeet(1)+CONFIG.footSize(2,2) poseFeet(2)+CONFIG.footSize(1,1) poseFeet(2)+CONFIG.footSize(1,2)])
    
end

figureCounter = figureCounter +1;

%% Centroidal momentum error
figure(figureCounter)
set(gcf,'numbertitle','off','name','H error')
subplot(2,1,1)
hold all
grid on
plot(t,HErr(1:3,:))
xlabel('Time [s]')
ylabel('H_{Lin}-H_{Lin}^{d}')
title('Linear Momentum Error')

figure(figureCounter)
subplot(2,1,2)
hold all
grid on
plot(t,HErr(4:6,:))
xlabel('Time [s]')
ylabel('H_{Ang}-H_{Ang}^{d}')
title('Angular Momentum Error')

figureCounter = figureCounter +1;

end
