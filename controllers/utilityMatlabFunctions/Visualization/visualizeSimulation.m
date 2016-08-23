function figureCont = visualizeSimulation(t,chi,CONFIG)
%VISUALIZESIMULATION is the iCub MATLAB simulator.
%
%   VISUALIZESIMULATION generates a simulation from the forward dynamics
%   integration of the iCub robot. There are two simulations. The one on the
%   left is performed considering the robot not attached to the ground, i.e.
%   free floating. The one on the right is performed considering also the
%   contacts the robot exerts with the environment, i.e. it is a constrained
%   floating base system.
%
%   figureCont = VISUALIZESIMULATION(t,chi,config) takes as input the integration
%   time T, the robot state CHI and a structure CONFIG containing all the
%   utility parameters.
%   The output is a counter for the automatic correction of figures numbers
%   in case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% initial parameters
figureCont = CONFIG.figureCont;

%% Simulator setup
BackGroundColor = [0 0 0];
GridColor       = [1 1 1];
figure_main     = figure('Name', 'iCub Simulator', 'NumberTitle', 'off',...
                         'Position', [500,800,1200,650],'Color',BackGroundColor);

sizeFig         = [10 26 800 600];

set(gcf, 'position', sizeFig);

CONFIG.figure_main = figure_main;

set(figure_main, 'MenuBar', 'none', 'BackingStore', 'off');
set(figure_main, 'BackingStore', 'off');

CONFIG.plot_main   = zeros(1,4);

plot_pos          = [0.51,0.05,0.45,1;
                     0.01,0.05,0.45,1];

for ii=1:2
    
    CONFIG.plot_main(ii) = subplot('Position', plot_pos(ii,:));
    CONFIG.plot_objs{ii} = plot3(0,0,0,'.');
    hold on;
    set(gca,'Color',BackGroundColor,'Xcolor',GridColor,'Ycolor',GridColor,'Zcolor',GridColor);
    view([45 25 25])
end

axes(CONFIG.plot_main(1));

CONFIG.demux.baseOrientationType  = 1;
robotConfiguration_t              = zeros(size(chi(:,1:8+CONFIG.ndof-1)));

for i = 1:length(t)
    
    [basePosei,jointAnglesi,~,~] = stateDemux(chi(i,:)',CONFIG);
    robotConfiguration_t(i,:)    = [basePosei(1:3)',basePosei(4:7)',jointAnglesi'];
end

visualizeForwardDynamics(robotConfiguration_t,CONFIG);

figureCont = figureCont +1;
end
