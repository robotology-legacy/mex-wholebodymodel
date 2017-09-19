function sim_config = setupSimulation(~, sim_config, rot3d)
    switch nargin
        case 3
            if ~islogical(rot3d)
                error('WBM::setupSimulation: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
        case 2
            % no mouse-base rotation on all axes (default)
            rot3d = false;
        otherwise
            error('WBM::setupSimulation: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % check if sim_config is an instance from a derived class of "wbmSimConfig" ...
    if ~isa(sim_config, 'WBM.absSimConfig')
        error('WBM::setupSimulation: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end

    if ~isempty(sim_config.hWndFigure)
        clf(sim_config.hWndFigure); % clear the figure window ...
    end

    % init the figure window for the simulation:
    fig_pos = horzcat(sim_config.wnd_pos, sim_config.wnd_size);
    sim_config.hWndFigure = figure('Name', sim_config.wnd_title, 'Position', fig_pos);
    set(sim_config.hWndFigure, 'NumberTitle', 'off', 'MenuBar', 'none', 'BackingStore', 'off');

    if ~strcmp(sim_config.environment.bkgrd_color_opt, 'white')
        % change the color option of the axis background, axis lines and labels, and the figure background ...
        colordef(sim_config.hWndFigure, sim_config.environment.background_color_opt);
    end % else, use the default system colors ...

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
    figure(sim_config.hWndFigure);
    if rot3d
        % enable mouse-base rotation on all axes within the figure ...
        rotate3d on;
    end

    nAxes = sim_config.nAxes;
    if (size(sim_config.hAxes,2) ~= nAxes)
        error('WBM::setupSimulation: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
    end

    for i = 1:nAxes
        sim_config.hAxes(1,i) = subplot('Position', sim_config.axes_pos(i,1:4));
        axis(sim_config.axis_limits);

        % set the axes-properties:
        set(gca, 'XDir',   'reverse', ...
                 'Color',  sim_config.axes_colors(1,1:3), ...
                 'XColor', sim_config.axes_colors(2,1:3), ...
                 'YColor', sim_config.axes_colors(3,1:3), ...
                 'ZColor', sim_config.axes_colors(4,1:3));
        if sort_meth
            % for Matlab R2014b and later ...
            set(gca, 'SortMethod', 'childorder');
        else
            % for older Matlab versions (<= R2014a) ...
            set(gca, 'DrawMode', 'fast');
        end

        hold on; % retain plots in the current axes ...
        % draw the colored ground floor (rectangle) ...
        fill3(sim_config.environment.grnd_shape(1,1:4), sim_config.environment.grnd_shape(2,1:4), ...
              sim_config.environment.grnd_shape(3,1:4), sim_config.environment.grnd_color);
        % draw the origin point of the axis onto the floor (rectangle) ...
        sim_config.gfx_objects{1,i} = plot3(0, 0, 0, 'Marker', '.', 'MarkerSize', 4, 'MarkerEdgeColor', 'k');
    end
    % set the current axes handle of the main figure to the first subplot ...
    set(sim_config.hWndFigure, 'CurrentAxes', sim_config.hAxes(1,1));
end
