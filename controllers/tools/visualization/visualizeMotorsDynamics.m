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
counter = 0;

if strcmp(CONFIG.robot_name,'bigman')==1 || strcmp(CONFIG.robot_name,'icubGazeboSim')==1
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
    counter    = 13;
end

for k=1:6
    
    % LEFT LEG
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Lleg motor pos')
    subplot(3,2,k)
    plot(t,theta(k+counter,:))
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
    plot(t,theta(k+counter+6,:))
    legend('theta')
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    name = getJointAnnotationICub('right_leg',k);
    title(name)
end

figureCont = figureCont +2;

if strcmp(CONFIG.robot_name,'bigman')==1 || strcmp(CONFIG.robot_name,'icubGazeboSim')==1
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
    
    figureCont = figureCont +1;
end

%% Motors dynamics: dtheta
if strcmp(CONFIG.robot_name,'bigman')==1 || strcmp(CONFIG.robot_name,'icubGazeboSim')==1
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
end

for k=1:6
    
    % LEFT LEG
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Lleg motor vel')
    subplot(3,2,k)
    plot(t,dtheta(k+counter,:))
    hold on
    plot(t,dtheta_ref(k+counter,:),'k')
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
    plot(t,dtheta(k+counter+6,:))
    hold on
    plot(t,dtheta_ref(k+counter+6,:),'k')
    legend('dtheta','dthetaRef')
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [deg/s]')
    name = getJointAnnotationICub('right_leg',k);
    title(name)
end

figureCont = figureCont +2;

if strcmp(CONFIG.robot_name,'bigman')==1 || strcmp(CONFIG.robot_name,'icubGazeboSim')==1
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

end

