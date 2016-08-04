function [] = visualizer_demo(t,state,config,references,jetsIntensitiesByWeight)

useSavedData      = true;
if useSavedData
    close all;
    clear all;
    clc;
    % load('helicoidalShort.mat');
    %     load('helicoidal.mat');
    fileName = 'helicoidal.mat';
    load(fileName);
    config.fileName = fileName;
    t = time.Data;
    state = state.Data;
    references = desired_x_dx_ddx_dddx_CoM;
end
config.visualiser.makeVideo         = true;
if config.visualiser.makeVideo
    config.visualiser.video.filename = 'helicoidal';
end

config.visualiser.computeKinematics = true;
config.visualiser.saveKinematics    = false;

config.visualiser.timeStep = 0.05;
% config.visualiser.wait     = waitbar(0,'Wait:');
config.plotComTrajectories = true;

addpath([getenv('CODYCO_SUPERBUILD_ROOT') '/main/mexWholeBodyModel/mex-wholebodymodel/matlab/wrappers'])

wbm_modelInitialise('icubGazeboSim');

BackGroundColor = [0 0 0];
GridColor       = [1 1 1];
figure_main = figure('Name', 'iCub Simulator', 'NumberTitle', 'off',...
    'Position', [500,800,1200,650],'Color',BackGroundColor);

%% ADAPT FIGURE DIMENSION DEPENDING ON SCREEN SIZE
sizeFig = get(0, 'MonitorPositions');
sizeFig = 2*sizeFig/3;
sizeFig(1:2) = sizeFig(3:4)/10;
set(gcf, 'position', sizeFig);
config.figure_main = figure_main;


config.plot_main = zeros(1,4);

plot_pos = [0.51,0.05,0.45,1;
    0.01,0.05,0.45,1];
for ii=1:2
    config.plot_main(ii) = subplot('Position', plot_pos(ii,:));
    config.plot_objs{ii} = plot3(0,0,0,'.');
    hold on;
    set(gca,'Color',BackGroundColor,'Xcolor',GridColor,'Ycolor',GridColor,'Zcolor',GridColor);
    set(gcf, 'MenuBar', 'None')
end

axes(config.plot_main(1));

% root link trajectory
config.demux.baseOrientationType = 1;
robotConfiguration_t = zeros(length(t),7+size(config.ndofM,1));
for i = 1:length(t)
    [basePosei,jointAnglesi,~,~] = stateDemux(state(i,:),config);
    robotConfiguration_t(i,:) = [basePosei(1:3,4)',basePosei(:,1)',jointAnglesi];
end

visualizeForwardDynamics(robotConfiguration_t,t,config,references.Data,jetsIntensitiesByWeight.Data);


end
