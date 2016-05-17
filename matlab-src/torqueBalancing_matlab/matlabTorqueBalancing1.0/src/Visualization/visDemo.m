function ContFig = visDemo(t,chi,params)
%VISDEMO is the iCub MATLAB simulator. 
%   VISDEMO generates a simulation from the forward dynamics integration 
%   of the iCub robot. There are two simulations. The one on the left is 
%   performed considering the robot not attached to the ground, i.e. free 
%   floating. The one on the right is performed considering also the contacts 
%   the robot exerts with the environment, i.e. it is a constrained floating
%   base system.
%    
%   ContFig = VISDEMO(t,chi,params) takes as input the integration time t,
%   the robot state Chi and a structure params containing all the utility
%   parameters.  The output is a counter for the automatic correction of
%   figures numbers in case a new figure is added.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%

% ------------Initialization----------------
% initial parameters
ContFig = params.ContFig;

%% Simulator setup
BackGroundColor = [0 0 0];
GridColor       = [1 1 1];
figure_main     = figure('Name', 'iCub Simulator', 'NumberTitle', 'off',...
                         'Position', [500,800,1200,650],'Color',BackGroundColor);
                     
sizeFig         = [10 26 800 600];

set(gcf, 'position', sizeFig);
 
params.figure_main = figure_main;
 
set(figure_main, 'MenuBar', 'none', 'BackingStore', 'off');
set(figure_main, 'BackingStore', 'off');

params.plot_main   = zeros(1,4);
    
 plot_pos          = [0.51,0.05,0.45,1;
                      0.01,0.05,0.45,1];

  for ii=1:2
      
        params.plot_main(ii) = subplot('Position', plot_pos(ii,:));
        params.plot_objs{ii} = plot3(0,0,0,'.');
        hold on;
        set(gca,'Color',BackGroundColor,'Xcolor',GridColor,'Ycolor',GridColor,'Zcolor',GridColor);
        view([45 25 25])
  end
    
 axes(params.plot_main(1));

 params.demux.baseOrientationType  = 1;
 robotConfiguration_t              = zeros(size(chi(:,1:8+params.ndof-1))); 

 for i = 1:length(t)
    
     [basePosei,jointAnglesi,~,~] = stateDemux(chi(i,:)',params);
     robotConfiguration_t(i,:)    = [basePosei(1:3)',basePosei(4:7)',jointAnglesi'];
 end 

 visualizeForwardDynamics(robotConfiguration_t,params);
 
 ContFig = ContFig +1;
end
