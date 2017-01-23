function figureCont = visualizeSimulation(varargin)
%VISUALIZESIMULATION is the iCub MATLAB simulator.
%
% VISUALIZESIMULATION generates a simulation from the forward dynamics
% integration of the iCub robot. There are two simulations. The one on the
% left is performed considering the robot not attached to the ground, i.e.
% free floating. The one on the right is performed considering also the
% contacts the robot exerts with the environment, i.e. it is a constrained
% floating base system.
%
% * CURRENT VISUALIZER: FROM visualizeForwardDynamics.m *

% ------------Initialization----------------
%% Initial parameters
t          = varargin{1};
chi        = varargin{2};
CONFIG     = varargin{3};
references = varargin{4};

if nargin == 5
    jetsIntensitiesByWeight = varargin{5};
end

% Configure the visualization
figureCont                          = CONFIG.figureCont;
CONFIG.visualiser.computeKinematics = true;
CONFIG.visualiser.saveKinematics    = false;
CONFIG.visualiser.timeStep          = 0.05;
CONFIG.plotComTrajectories          = true;
CONFIG.visualiser.useSavedData      = false;

%% Use recorded data
if CONFIG.visualiser.useSavedData

    fileName                = 'helicoidal.mat';
    STORED_DATA             = load(fileName);
    CONFIG.fileName         = fileName;
    t                       = STORED_DATA.time.Data;
    chi                     = STORED_DATA.state.Data;
    references              = STORED_DATA.desired_x_dx_ddx_dddx_CoM.Data;
    CONFIG.state0           = STORED_DATA.state0;
end

%% Make a video of the simulation
CONFIG.visualiser.makeVideo = false;

if CONFIG.visualiser.makeVideo
    CONFIG.visualiser.video.filename = 'robotSim';
end

%% Configure the plots
BackGroundColor = [0 0 0];
GridColor       = [1 1 1];
figure_main     = figure('Name', 'iCub Simulator', 'NumberTitle', 'off',...
                         'Position',[500,800,1200,650],'Color',BackGroundColor);

%% ADAPT FIGURE DIMENSION DEPENDING ON SCREEN SIZE
sizeFig      = get(0, 'MonitorPositions');
sizeFig      = sizeFig(1,:);
sizeFig      = 2*sizeFig/3;
sizeFig(1:2) = sizeFig(3:4)/10;
set(gcf, 'position', sizeFig);

CONFIG.figure_main = figure_main;
CONFIG.plot_main   = zeros(1,4);

plot_pos = [0.51,0.05,0.45,1;
            0.01,0.05,0.45,1];
for ii=1:2
    CONFIG.plot_main(ii) = subplot('Position', plot_pos(ii,:));
    CONFIG.plot_objs{ii} = plot3(0,0,0,'.');
    hold on;
    set(gca,'Color',BackGroundColor,'Xcolor',GridColor,'Ycolor',GridColor,'Zcolor',GridColor);
    set(gcf, 'MenuBar', 'None')
    view([45 25 25])
end

axes(CONFIG.plot_main(1));

% Root link trajectory
CONFIG.demux.baseOrientationType = 1;
robotConfiguration_t             = zeros(size(chi(:,1:8+CONFIG.ndof-1)));

for i = 1:length(t)
    
    [basePosei,jointAnglesi,~,~] = stateDemux(chi(i,:)',CONFIG);
    robotConfiguration_t(i,:)    = [basePosei(1:3)',basePosei(4:7)',jointAnglesi'];
end

%% Visualize the robot simulation
visualizeForwardDynamics(robotConfiguration_t,t,CONFIG,references);

% figures counter
figureCont = figureCont +1;

end
