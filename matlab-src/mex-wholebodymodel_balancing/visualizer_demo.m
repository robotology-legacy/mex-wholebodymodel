function [] = visualizer_demo(t,chi,params)

% This is a program for visualizing the demo of the balancing and the floating base position in space
    
   %initialize GUI
    figure_main = figure('Name', 'iCub Simulator', 'NumberTitle', 'off',...
                         'Position', [50,400,600,650]);
    
    params.figure_main = figure_main;
    set(figure_main, 'MenuBar', 'none', 'BackingStore', 'off');
    set(figure_main, 'BackingStore', 'off');
 
    params.plot_main =zeros(1,4);
    
    plot_pos = [0.51,0.20,0.45,0.40;
                0.01,0.20,0.45,0.40;
                0.51,0.62,0.45,0.40;
                0.01,0.62,0.45,0.40];

    for ii=1:4
    
        params.plot_main(ii) = subplot('Position', plot_pos(ii,:));
        params.plot_objs{ii} = plot3(0,0,0,'.');
        axis([-0.5 0.5 -0.42 0.58 0 1]);
        hold on;
        patch([-0.45 -0.45 0.45 0.45],[-0.37 0.53 0.53 -0.37],[0 0 0 0],[0.6 0.6 0.8]);
        set(gca,'Color',[0.8 0.8 0.8]);
        set(gca,'XColor',[0.8 0.8 0.8]);
        set(gca,'YColor',[0.8 0.8 0.8]);
        set(gca,'ZColor',[0.8 0.8 0.8]);
        set(gca,'xdir','reverse')
        set(gca, 'drawmode', 'fast');
        params.draw_init = 1;
        rotate3d(gca,'on');

        figure(figure_main);
        
    end
    
    axes(params.plot_main(1))

   %CoM trajectory
    ndof  = params.ndof;
    x_b   = chi(:,1:3);
    qt_b  = chi(:,4:7);
    qj    = chi(:,8:ndof+7);

    visualizeForwardDynamics([x_b,qt_b,qj],t,params);    
    pause;

%% plot base link positions
    figure(2);
    plot3(x_b(:,1),x_b(:,2),x_b(:,3));
    hold on
    plot3(x_b(1,1),x_b(1,2),x_b(1,3),'ro');
    grid on;
    title('positions of the root link')
 
    axis square;
   %axis equal
    
    xlabel('X(m)');
    ylabel('Y(m)');
    zlabel('Z(m)');
    
    figure(3);
    plot(t,x_b(:,1),t,x_b(:,2),'r',t,x_b(:,3),'k');
    hold on
    grid on;
    title('positions of the root link')

end
