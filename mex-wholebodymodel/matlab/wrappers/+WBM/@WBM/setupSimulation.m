function setupSimulation(~, sim_config)
    % check if sim_config is an instance from a derived class of "wbmSimConfig" ...
    if ~isa(sim_config, 'WBM.absSimConfig')
        error('WBM::setupSimulation: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end

    figure; clf;

    % init the main figure window for the simulation:
    sim_config.hFigure_main = figure('Name', sim_config.main_title, 'Position', sim_config.MAIN_POS);
    set(sim_config.hFigure_main, 'NumberTitle', 'off', 'MenuBar', 'none', 'BackingStore', 'off');
    % setup the rendering method of the current figure handle:
    d = opengl('data');
    if ~d.Software
        % use the hardware-accelerated OpenGL renderer and not the slow software variant.
        % Note: the axes DrawMode property is ignored in OpenGL.
        set(gcf, 'Renderer', 'opengl');
    else
        % set the renderer property to "painters" to benefit from the DrawMode
        % or the new property SortMethod ...
        set(gcf, 'Renderer', 'painters');
    end

    sort_meth = true;
    if verLessThan('matlab', '8.4.0') % check Matlab version ...
        sort_meth = false;
    end
    % split up the main figure into a 2x2 grid, create and setup the axes for each
    % 3D-subplot and draw a solid patch (rectangle) on the bottom of the axes:
    figure(sim_config.hFigure_main);
    for i = 1:4
        sim_config.hAxes(i) = subplot('Position', sim_config.AXES_POS(i,1:4));
        axis(sim_config.AXIS_LIMITS);

        % set the axes-properties:
        set(gca, 'XDir',   'reverse', ...
                 'Color',  sim_config.AXES_COLORS(1,1:3), ...
                 'XColor', sim_config.AXES_COLORS(2,1:3), ...
                 'YColor', sim_config.AXES_COLORS(3,1:3), ...
                 'ZColor', sim_config.AXES_COLORS(4,1:3));
        if sort_meth
            % for Matlab R2014b and later ...
            set(gca, 'SortMethod', 'childorder');
        else
            % for older Matlab versions (<= R2014a) ...
            set(gca, 'DrawMode', 'fast');
        end
        % enable mouse-base rotation on all axes ...
        rotate3d(gca, 'on'); 
        %params.draw_init = 1;
        
        hold on; % retain plots in the current axes ...
        % draw the colored rectangle ...
        patch(sim_config.PATCH_SHAPE(1,1:4), sim_config.PATCH_SHAPE(2,1:4), ...
              sim_config.PATCH_SHAPE(3,1:4), sim_config.PATCH_COLOR);
        % draw the origin point of the axis onto the rectangle ...
        sim_config.plot_objs{i} = plot3(0, 0, 0, 'Marker', '.', 'MarkerEdgeColor', 'k');
    end
    % set the current axes handle of the main figure to the first subplot ...
    set(sim_config.hFigure_main, 'CurrentAxes', sim_config.hAxes(1));
end
