function figureCont = visualizeJointDynamics(varargin)
%VISUALIZEJOINTDYNAMICS visualizes the joint space dynamics of the iCub robot
%                       from forward dynamics integration.
%
% figureCont = VISUALIZEJOINTDYNAMICS(varargin) takes as inputs the
% integration time t, a structure CONFIG which contains all the utility
% parameters, the joint position qj and the desired joint position qjRef.
% Eventually, the parameter tAss (settling time of a second order system
% equations) can be added to the inputs list.
% The output is a counter for the automatic correction of figures numbers
% in case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
import WBM.utilities.getJointAnnotationICub;

% setup parameters
t           = varargin{1};
CONFIG      = varargin{2};
qj          = varargin{3};
qjRef       = varargin{4};
figureCont  = CONFIG.figureCont;
qj          = 180/pi*qj;
qjRef       = 180/pi*qjRef;

if nargin == 5
    
    tAss    = varargin{5};
    index   = zeros(length(tAss));
    
    for k   = 1:length(tAss)
        index(k) = sum(t>tAss(k))+1;
    end
    
    step    = qjRef(:,end)-qj(:,1);
end
       
%% Joints dynamics
counter = 0;

if strcmp(CONFIG.robot_name,'bigman')==1 || strcmp(CONFIG.robot_name,'icubGazeboSim')==1
    for k=1:5
    
        % LEFT ARM
        figure(figureCont)
        set(gcf,'numbertitle','off','name','Larm pos')
        subplot(3,2,k)
        plot(t,qj(k+3,:))
        hold on
        plot(t,qjRef(k+3,:),'k')
        legend('qj','qjRef')
    
        if nargin == 5
            plot(tAss(k+3),qj(k+3,end-index(k+3)),'ok')
            plot(t,qjRef(k+3,:)+0.05*step(k+3),'--g')
            plot(t,qjRef(k+3,:)-0.05*step(k+3),'--g') 
            legend('qj','qjRef','tAss')
        end
    
        grid on
        xlabel('Time [s]')
        ylabel('Angle [deg]')
        name = getJointAnnotationICub('left_arm',k);
        title(name)
    
        % RIGHT ARM
        figure(figureCont+1)
        set(gcf,'numbertitle','off','name','Rarm pos')
        subplot(3,2,k)
        plot(t,qj(k+3+5,:))
        hold on
        plot(t,qjRef(k+3+5,:),'k')
        legend('qj','qjRef')
    
        if nargin == 5
            plot(tAss(k+3+5),qj(k+3+5,end-index(k+3+5)),'ok')
            plot(t,qjRef(k+3+5,:)+0.05*step(k+3+5),'--g')
            plot(t,qjRef(k+3+5,:)-0.05*step(k+3+5),'--g') 
            legend('qj','qjRef','tAss')
        end
    
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
    set(gcf,'numbertitle','off','name','Lleg pos')
    subplot(3,2,k)
    plot(t,qj(k+counter,:))
    hold on
    plot(t,qjRef(k+counter,:),'k')
    legend('qj','qjRef')
    
    if nargin == 5
        plot(tAss(k+counter),qj(k+counter,end-index(k+counter)),'ok')
        legend('qj','qjRef','tAss')        
    end
    
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    name = getJointAnnotationICub('left_leg',k);
    title(name)
    
    % RIGHT LEG
    figure(figureCont+1)
    set(gcf,'numbertitle','off','name','Rleg pos')
    subplot(3,2,k)
    plot(t,qj(k+counter+6,:))
    hold on
    plot(t,qjRef(k+counter+6,:),'k')
    legend('qj','qjRef')
    
    if nargin == 5
        plot(tAss(k+counter+6),qj(k+counter+6,end-index(k+counter+6)),'ok')
        legend('qj','qjRef','tAss')        
    end
    
    plot(t,qjRef(k+counter+6,:),'k')
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
        set(gcf,'numbertitle','off','name','Torso pos')
        subplot(3,1,k)
        plot(t,qj(k,:))
        hold on
        plot(t,qjRef(k,:),'k')
        legend('qj','qjRef')
    
        if nargin == 5
            plot(tAss(k),qj(k,end-index(k)),'ok')
            plot(t,(qjRef(k,:)+0.05*step(k)),'--g')
            plot(t,(qjRef(k,:)-0.05.*step(k)),'--g')
            legend('qj','qjRef','tAss')        
        end
    
        grid on
        xlabel('Time [s]')
        ylabel('Angle [deg]')
        name = getJointAnnotationICub('torso',k);
        title(name)
    end

    figureCont = figureCont +1;
end
end
