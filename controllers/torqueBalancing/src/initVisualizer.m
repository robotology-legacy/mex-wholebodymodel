function figureCont = initVisualizer(t_total,chi_total,MODEL)
%INITVISUALIZER initializes the visualization of forward dynamics
%               integration results.
%
% Format: figureCont = initVisualizer(t,chi,MODEL,INIT_CONDITIONS)
%
% Inputs:  - t_total vector of time instants;
%          - chi_total matrix of state vectors (at each instant) [13+4*ndof x lenght(t_tota)];
%          - MODEL is a structure defining the robot model;.
%
% Output:  - figureCont a counter for correctly set figure numbers.
%
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
global state;
% reset state to the initial value
state = 1;

%% Configuration parameters
ndof             = MODEL.ndof;
figureCont       = MODEL.figureCont;

%% Robot simulator (offline)
if MODEL.CONFIG.visualize_robot_simulator == 1
    MODEL.VISUALIZER = configureSimulator();
    iDyntreeSimulator(t_total,chi_total,MODEL);
end

%% Forward dynamics integration results
if MODEL.CONFIG.visualize_integration_results == 1 
    % configure figures
    set(0,'DefaultFigureWindowStyle','Docked');
    set(0,'DefaultAxesFontSize',16);
    set(0,'DefaultTextFontSize',16);
    set(0,'DefaultLineLineWidth',2);
    set(0,'DefaultLineMarkerSize',8);
    % elaborate data obtained with integration. First, creat a time vector
    % and select only the instants for which there are data available
    timeTot       = MODEL.timeTot;
    load('./media/storedValues.mat');
    positiveIndex = timeIndex ~= 0;
    % interactive display for visualizing the results
    str   = {'torques','torques norm','joint position','motor velocity'};
    [s,v] = listdlg('PromptString','Select outputs:',...
                    'ListString',str);    
end
% update figures counter
figureCont = MODEL.figureCont;
set(0,'DefaultFigureWindowStyle','Normal');

end
