%% visDemo
% visualize a demo of the robot's movements.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016
%
function [] = visDemo(t,chi,params)
%% Demo generation
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

end
