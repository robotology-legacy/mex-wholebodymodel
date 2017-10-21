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
    if ~isa(sim_config, 'WBM.wbmSimConfig')
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
        % change the color options of the axis background, the axis lines and
        % labels, and the figure background:
        colordef(sim_config.hWndFigure, sim_config.environment.bkgrd_color_opt);
    end % else, use the default system colors ...

    % setup the rendering method of the current figure handle:
    gldta = opengl('data');
    if ~gldta.Software
        % use the hardware-accelerated OpenGL renderer and not the slow software variant.
        % Note: the axes DrawMode property is ignored in OpenGL.
        set(gcf, 'Renderer', 'opengl');
    else
        % set the renderer property to "painters" to benefit from the DrawMode
        % or the new property SortMethod ...
        set(gcf, 'Renderer', 'painters');
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
    % initialize all axes for the simulation and add them to the config ...
    sim_config = initAxes(sim_config, nAxes);
    % set the current axes handle of the main figure to the first subplot ...
    set(sim_config.hWndFigure, 'CurrentAxes', sim_config.hAxes(1,1));
end
%% END of setupSimulation.


%% AXES INITIALIZATION:

function sim_config = initAxes(sim_config, nAxes)
    if verLessThan('matlab', '8.4.0') % check Matlab version ...
        % for older Matlab versions (<= R2014a):
        for i = 1:nAxes
            sim_config.hAxes(1,i) = createAxes(sim_config, i);
            set(gca, 'DrawMode', 'fast');
        end
        return
    end
    % for Matlab R2014b and later:
    for i = 1:nAxes
        sim_config.hAxes(1,i) = createAxes(sim_config, i);
        set(gca, 'SortMethod', 'childorder');
    end
end

function hax = createAxes(sim_config, idx)
    hax = subplot('Position', sim_config.axes_pos(idx,1:4));
    axis(sim_config.axis_limits);

    % set the axes-properties:
    set(gca, 'XDir',   'reverse', ...
             'Color',  sim_config.axes_colors(1,1:3), ...
             'XColor', sim_config.axes_colors(2,1:3), ...
             'YColor', sim_config.axes_colors(3,1:3), ...
             'ZColor', sim_config.axes_colors(4,1:3));
end
