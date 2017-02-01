function figureCont = visualizeMotorsDynamics(t,CONFIG,theta,dtheta_ref,dtheta)
%VISUALIZEMOTORSDYNAMICS visualizes the joint space dynamics of the iCub robot
%                        from forward dynamics integration.
%
% figureCont = VISUALIZEMOTORSDYNAMICS(varargin) takes as inputs the
% integration time t, a structure CONFIG which contains all the utility
% parameters, the motor position theta, motro velocities dtheta and
% their references dtheta_ref.
% The output is a counter for the automatic correction of figures numbers
% in case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
import WBM.utilities.getJointAnnotationICub;
figureCont     = CONFIG.figureCont;

% setup parameters
theta          = 180/pi*theta;
dtheta         = 180/pi*dtheta;
dtheta_ref     = 180/pi*dtheta_ref;
       
%% Motors dynamics: theta
for k=1:5
    
    % LEFT ARM
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Larm motor pos')
    subplot(3,2,k)
    plot(t,theta(k+3,:))
    legend('theta')  
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    name = getJointAnnotationICub('left_arm',k);
    title(name)
    
    % RIGHT ARM
    figure(figureCont+1)
    set(gcf,'numbertitle','off','name','Rarm motor pos')
    subplot(3,2,k)
    plot(t,theta(k+3+5,:))
    legend('theta')
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    name = getJointAnnotationICub('right_arm',k);
    title(name)
end

figureCont = figureCont +2;

for k=1:6
    
    % LEFT LEG
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Lleg motor pos')
    subplot(3,2,k)
    plot(t,theta(k+13,:))
    legend('theta')
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    name = getJointAnnotationICub('left_leg',k);
    title(name)
    
    % RIGHT LEG
    figure(figureCont+1)
    set(gcf,'numbertitle','off','name','Rleg motor pos')
    subplot(3,2,k)
    plot(t,theta(k+13+6,:))
    legend('theta')
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    name = getJointAnnotationICub('right_leg',k);
    title(name)
end

figureCont = figureCont +2;

for k=1:3
    
    % TORSO
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Torso motor pos')
    subplot(3,1,k)
    plot(t,theta(k,:))
    legend('theta')
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    name = getJointAnnotationICub('torso',k);
    title(name)
end

%% Motors dynamics: dtheta
for k=1:5
    
    % LEFT ARM
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Larm motor vel')
    subplot(3,2,k)
    plot(t,dtheta(k+3,:))
    hold on
    plot(t,dtheta_ref(k+3,:),'k')
    legend('dtheta','dthetaRef')  
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [deg/s]')
    name = getJointAnnotationICub('left_arm',k);
    title(name)
    
    % RIGHT ARM
    figure(figureCont+1)
    set(gcf,'numbertitle','off','name','Rarm motor vel')
    subplot(3,2,k)
    plot(t,dtheta(k+3+5,:))
    hold on
    plot(t,dtheta_ref(k+3+5,:),'k')
    legend('dtheta','dthetaRef')
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [deg/s]')
    name = getJointAnnotationICub('right_arm',k);
    title(name)
end

figureCont = figureCont +2;

for k=1:6
    
    % LEFT LEG
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Lleg motor vel')
    subplot(3,2,k)
    plot(t,dtheta(k+13,:))
    hold on
    plot(t,dtheta_ref(k+13,:),'k')
    legend('dtheta','dthetaRef')
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [deg/s]')
    name = getJointAnnotationICub('left_leg',k);
    title(name)
    
    % RIGHT LEG
    figure(figureCont+1)
    set(gcf,'numbertitle','off','name','Rleg motor vel')
    subplot(3,2,k)
    plot(t,dtheta(k+13+6,:))
    hold on
    plot(t,dtheta_ref(k+13+6,:),'k')
    legend('dtheta','dthetaRef')
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [deg/s]')
    name = getJointAnnotationICub('right_leg',k);
    title(name)
end

figureCont = figureCont +2;

for k=1:3
    
    % TORSO
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Torso motor vel')
    subplot(3,1,k)
    plot(t,dtheta(k,:))
    hold on
    plot(t,dtheta_ref(k,:),'k')
    legend('dtheta','dthetaRef')
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [deg/s]')
    name = getJointAnnotationICub('torso',k);
    title(name)
end

figureCont = figureCont +1;

end

