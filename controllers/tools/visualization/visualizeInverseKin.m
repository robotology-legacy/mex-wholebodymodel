function figureCont = visualizeInverseKin(CONFIG,IKIN)
%VISUALIZEINVERSEKIN visualizes the results on the inverse kinematics of the
%                    robot iCub.
%
% VISUALIZEINVERSEKIN verifies if the joint reference trajectory calculated
% in the inverse kinematics solver fits with the contact constraints and the
% desired CoM and momentum trajectories.
%
% figureCont = VISUALIZEINVERSEKIN(CONFIG,IKIN) takes as input the structure
% CONFIG containing all the utility parameters, and the structure IKIN
% which contains the visualization parameters. The output figureCont
% is a counter which automatically modifies the figure number according
% to the visualization setup.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% initial parameters
set(0,'DefaultFigureWindowStyle','Docked');

figureCont              = CONFIG.figureCont;
t                       = IKIN.t;
CoMTrajectoryError      = IKIN.CoMTrajectoryError;
feetError               = IKIN.feetError;
momentumError           = IKIN.momentumError;

%% CoM trajectory
titCoM  = {'X axis' 'Y axis' 'Z axis'};

for k = 1:3
    
    % POSITION
    figure(figureCont)
    set(gcf,'numbertitle','off','name','CoM error pos')
    subplot(3,1,k)
    plot(t,CoMTrajectoryError(k,:),'b')
    hold on
    grid on
    plot(t,CoMTrajectoryError(k+9,:),'r')
    xlabel('Time [s]')
    ylabel('Position [m]')
    title(titCoM(k))
    legend('desired CoM pos','ikin CoM pos')
    
    % VELOCITY
    figure(figureCont+1)
    set(gcf,'numbertitle','off','name','CoM error vel')
    subplot(3,1,k)
    plot(t,CoMTrajectoryError(k+3,:),'b')
    hold on
    grid on
    plot(t,CoMTrajectoryError(k+9+3,:),'r')
    xlabel('Time [s]')
    ylabel('Velocity [m/s]')
    title(titCoM(k))
    legend('desired CoM vel','ikin CoM vel')
    
    % ACCELERATION
    figure(figureCont+2)
    set(gcf,'numbertitle','off','name','CoM error acc')
    subplot(3,1,k)
    plot(t,CoMTrajectoryError(k+6,:),'b')
    hold on
    grid on
    plot(t,CoMTrajectoryError(k+9+6,:),'r')
    xlabel('Time [s]')
    ylabel('Acceleration [m/s^2]')
    title(titCoM(k))
    legend('desired CoM acc','ikin CoM acc')
end

figureCont = figureCont +3;

%% Feet errors
titFeet  = {'Error on left foot pos along x axis','Error on left foot pos along y axis','Error on left foot pos along z axis',...
            'Error on left foot ori along x axis','Error on left foot ori along y axis','Error on left foot ori along z axis',...
            'Error on right foot pos along x axis','Error on right foot pos along y axis','Error on right foot pos along z axis',...
            'Error on right foot ori along x axis','Error on right foot ori along y axis','Error on right foot ori along z axis'};

titYFeet = {'[m]' '[m]' '[m]' '[rad]' '[rad]' '[rad]'};

if     CONFIG.numConstraints == 1
    
    for k=1:6
        
        figure(figureCont)
        set(gcf,'numbertitle','off','name','Foot error')
        subplot(3,2,k)
        plot(t,feetError(k,:))
        grid on
        xlabel('Time [s]')
        ylabel(titYFeet(k))
        
        if CONFIG.feet_on_ground(1) == 1
            
            title(titFeet(k))
        else
            title(titFeet(k+6))
        end
    end
    
    figureCont = figureCont +1;
    
elseif CONFIG.numConstraints  == 2
    
    for k=1:6
        
        figure(figureCont)
        set(gcf,'numbertitle','off','name','Lfoot error')
        subplot(3,2,k)
        plot(t,feetError(k,:))
        grid on
        title(titFeet(k))
        xlabel('Time [s]')
        ylabel(titYFeet(k))
        
        figure(figureCont+1)
        set(gcf,'numbertitle','off','name','Rfoot error')
        subplot(3,2,k)
        plot(t,feetError(k+6,:))
        grid on
        title(titFeet(k+6))
        xlabel('Time [s]')
        ylabel(titYFeet(k))
    end
    
    figureCont = figureCont +2;
end

%% Momentum error
figure(figureCont)
set(gcf,'numbertitle','off','name','H error')
subplot(2,1,1)
hold all
grid on
plot(t,momentumError(1:3,:))
xlabel('Time [s]')
ylabel('H_{Lin}-H_{Lin}^{d}')
title('Linear Momentum Error (Inverse Kinematics)')

subplot(2,1,2)
hold all
grid on
plot(t,momentumError(4:6,:))
xlabel('Time [s]')
ylabel('H_{Ang}-H_{Ang}^{d}')
title('Angular Momentum Error (Inverse Kinematics)')

figureCont = figureCont +1;
set(0,'DefaultFigureWindowStyle','Normal');

end
