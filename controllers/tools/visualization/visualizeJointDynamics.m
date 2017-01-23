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
for k=1:5
    
    % LEFT ARM
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Larm pos')
    subplot(3,2,k)
    plot(t,qj(k+3,:))
    hold on
    
    if nargin == 5
        plot(tAss(k+3),qj(k+3,end-index(k+3)),'ok')
        plot(t,qjRef(k+3,:)+0.05*step(k+3),'--g')
        plot(t,qjRef(k+3,:)-0.05*step(k+3),'--g')   
    end
    
    plot(t,qjRef(k+3,:),'k')
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    name = whatname('left_arm',k);
    title(name)
    legend('qj','qjRef')
    
    % RIGHT ARM
    figure(figureCont+1)
    set(gcf,'numbertitle','off','name','Rarm pos')
    subplot(3,2,k)
    plot(t,qj(k+3+5,:))
    hold on
    
    if nargin == 5
        plot(tAss(k+3+5),qj(k+3+5,end-index(k+3+5)),'ok')
        plot(t,qjRef(k+3+5,:)+0.05*step(k+3+5),'--g')
        plot(t,qjRef(k+3+5,:)-0.05*step(k+3+5),'--g')  
    end
    
    plot(t,qjRef(k+3+5,:),'k')
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    name = whatname('right_arm',k);
    title(name)
    legend('qj','qjRef')
end

figureCont = figureCont +2;

for k=1:6
    
    % LEFT LEG
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Lleg pos')
    subplot(3,2,k)
    plot(t,qj(k+13,:))
    hold on
    
    if nargin == 5
        plot(tAss(k+13),qj(k+13,end-index(k+13)),'ok')
    end
    
    plot(t,qjRef(k+13,:),'k')
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    name = whatname('left_leg',k);
    title(name)
    legend('qj','qjRef')
    
    % RIGHT LEG
    figure(figureCont+1)
    set(gcf,'numbertitle','off','name','Rleg pos')
    subplot(3,2,k)
    plot(t,qj(k+13+6,:))
    hold on
    
    if nargin == 5
        plot(tAss(k+13+6),qj(k+13+6,end-index(k+13+6)),'ok')
    end
    
    plot(t,qjRef(k+13+6,:),'k')
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    name = whatname('right_leg',k);
    title(name)
    legend('qj','qjRef')
end

figureCont = figureCont +2;

for k=1:3
    
    % TORSO
    figure(figureCont)
    set(gcf,'numbertitle','off','name','Torso pos')
    subplot(3,1,k)
    plot(t,qj(k,:))
    hold on
    
    if nargin == 5
        plot(tAss(k),qj(k,end-index(k)),'ok')
        plot(t,(qjRef(k,:)+0.05*step(k)),'--g')
        plot(t,(qjRef(k,:)-0.05.*step(k)),'--g')      
    end
    
    plot(t,qjRef(k,:),'k')
    grid on
    xlabel('Time [s]')
    ylabel('Angle [deg]')
    name = whatname('torso',k);
    title(name)
    legend('qj','qjRef')
end

figureCont = figureCont +1;

end
