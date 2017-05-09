%% INITVISUALIZER 
%  initializes the visualization tool. It loads the .mat file
%  storing integration results and plots the relative figures.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, March 2017

%% ------------Initialization----------------
% load main file (forward dynamics integration results)
load('./media/storedValuesFwdDyn.mat')
% load other files if present
if CONFIG.use_gainTuning
    load('./media/storedValuesGainTuning.mat')
end
if CONFIG.use_ikinSolver
    load('./media/storedValuesIkin.mat')
end

%% Configure figures setup and pre-processing data
preProcessing = true;
timeTot       = CONFIG.tStart:CONFIG.sim_step:CONFIG.tEnd;
if preProcessing
    set(0,'DefaultFigureWindowStyle','Docked');
    set(0,'DefaultAxesFontSize',16);
    set(0,'DefaultTextFontSize',16);
    set(0,'DefaultLineLineWidth',2);
    set(0,'DefaultLineMarkerSize',8);
    % elaborate data obtained with integration. First, adapt data dimensions
    % to the size of a fixed-step time vector (forward dynamics uses a variable 
    % step integrator)
    positiveIndex = timeIndex ~= 0;
else
    positiveIndex = 1:length(timeTot); %#ok<UNRCH>
end

%% Interactive display for visualizing the results
% create a first list of (always) available results  
str   = {'torques','joint position','CoM position','feet pose','centroidal momentum','contact forces'};
% add other results if present
if CONFIG.use_gainTuning 
    str = [str, {'gain tuning'}];
end
if CONFIG.use_ikinSolver 
    str = [str, {'inverse kinematics'}];
end
if CONFIG.use_SEA 
    str = [str, {'motor dynamics'}];
end
% create a menu' for showing the results
[listPlot,~] = listdlg('PromptString','Select outputs to plot:',...
                       'ListString',str);    

%% Show the corresponding figures
figureCounter = 1;
% forward dynamics
figureCounter = visualizeForwardDyn(timeTot,fc,H,HRef,poseFeet,qj,qjRef,tau_xi,xCoM,xCoMDes,...
                                    positiveIndex,listPlot,figureCounter);




set(0,'DefaultFigureWindowStyle','Normal');


