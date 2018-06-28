% Copyright (C) 2015-2018, by Martin Neururer
% Author: Martin Neururer
% E-mail: martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
% Date:   January-May, 2018
%
% Departments:
%   Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia and
%   Automation and Control Institute - TU Wien.
%
% This file is part of the Whole-Body Model Library for Matlab (WBML).
%
% The development of the WBM-Library was made in the context of the master
% thesis "Learning Task Behaviors for Humanoid Robots" and is an extension
% for the Matlab MEX whole-body model interface, which was supported by the
% FP7 EU-project CoDyCo (No. 600716, ICT-2011.2.1 Cognitive Systems and
% Robotics (b)), <http://www.codyco.eu>.
%
% Permission is granted to copy, distribute, and/or modify the WBM-Library
% under the terms of the GNU Lesser General Public License, Version 2.1
% or any later version published by the Free Software Foundation.
%
% The WBM-Library is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU Lesser General Public License for more details.
%
% A copy of the GNU Lesser General Public License can be found along
% with the WBML. If not, see <http://www.gnu.org/licenses/>.

function sim_config = setupSimulation(~, sim_config, rot3d)
    % Setups the figure window and the draw properties of the preinitialized
    % configuration object for the visualizer of the whole body model simulation.
    %
    % Arguments:
    %   sim_config (:class:`~WBM.wbmSimConfig`): Preconfigured configuration object
    %                                            with the geometric data of the robot
    %                                            model (joints and body shapes) to be
    %                                            simulated for the whole body model
    %                                            visualizer.
    %
    %                                            **Note:** The setup method assumes,
    %                                            that in the given configuration are
    %                                            following parameters defined:
    %
    %                                            axes:
    %                                               *number*, *positions* and *colors*.
    %
    %                                            figure window:
    %                                               *title*, *size*, *position*
    %                                               and *background colors*.
    %
    %                                            If additionally the flag for not showing
    %                                            the window or making a video is enabled,
    %                                            the figure window will be disabled and is
    %                                            invisible on the screen.
    %   rot3d (logical, scalar): Boolean flag to indicate if the mouse-base rotation
    %                            should be enabled on all axes within the figure
    %                            (default: *false*) -- *optional*.
    %
    %                            If the flag is set to *false* (default), then the
    %                            mouse-base rotation is *disabled* on all axes.
    % Returns:
    %   sim_config (struct): Configuration object with the initialized figure window,
    %   axes and draw properties for the whole body model visualizer.
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
        % change the color options of the axis background, axis lines and
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

    if (~sim_config.show_wnd || sim_config.mkvideo)
        % don't show the window on the screen (i.e. for making videos) ...
        set(sim_config.hWndFigure, 'Visible', 'off');
    end
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
