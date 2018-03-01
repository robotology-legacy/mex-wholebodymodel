% Copyright (C) 2015-2018, by Martin Neururer
% Author: Martin Neururer
% E-mail: martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
% Date:   January, 2018
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
% FP7 EU project CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and
% Robotics (b)), <http://www.codyco.eu>.
%
% Permission is granted to copy, distribute, and/or modify the WBM-Library
% under the terms of the GNU Lesser General Public License, Version 2.1
% or any later version published by the Free Software Foundation.
%
% The WBM-Library is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
%
% A copy of the GNU Lesser General Public License can be found along
% with the WBML. If not, see <http://www.gnu.org/licenses/>.

classdef genericSimConfig < WBM.wbmSimConfig
    % :class:`!genericSimConfig` is an *data type* (class) and represents the
    % *generic configuration interface* for the visualizer of the robot
    % simulation.
    %
    % Attributes:
    %   custom_view  (double, vector): Custom viewpoint specification in terms of
    %                                  *azimuth* (az) and *elevation* (el) as a
    %                                  row-vector :math:`[az, el]`.
    %   axes_vwpts     (cell, matrix): List of predefined viewpoint positions for
    %                                  the orientation of the axes (*readonly*).
    %                                  The list is a 2-dimensional array where in
    %                                  the first column are the names of the axes
    %                                  orientations and in the second column the
    %                                  corresponding viewpoint vectors.
    %   wnd_visible (logical, scalar): Figure window visibility, specified as
    %                                  *true* or *false*. If the property is set
    %                                  to *false*, then the entire figure is
    %                                  disabled and invisible on the screen. This
    %                                  is important for cases where the visibility
    %                                  of the figure is temporarily undesired (e.g.
    %                                  for performance reasons).
    %
    %   DF_WND_POS       (double, vector): Default window position vector, specified
    %                                      in *pixels* with the values
    %                                      :math:`[50\:\textrm{(left)}, 400\:\textrm{(bottom)}]`,
    %                                      (*constant*).
    %   DF_WND_SIZE      (double, vector): Default window size of the figure,
    %                                      specified as a vector with the values
    %                                      :math:`[600\:\textrm{(width)}, 650\:\textrm{(height)}]`,
    %                                      in *pixels* (*constant*).
    %   DF_AXES_NBR       (uint8, scalar): Default number of 4 axes graphics objects
    %                                      in the figure window, to divide the
    %                                      figure into a (2 x 2) grid (*constant*).
    %   DF_AXES_POS      (double, matrix): Default positions of each axes graphics
    %                                      object in the figure window, specified
    %                                      as a (4 x 4) matrix of the form
    %                                      :math:`[x\:\textrm{(left)}, y\:\textrm{(bottom)}, wid\:\textrm{(width)}, hgt\:\textrm{(height)}]`,
    %                                      where each row represents an axes
    %                                      position vector (*constant*).
    %   DF_AXES_COLORS   (double, matrix): Default (4 x 3) color matrix for the axes
    %                                      back planes and for the axis lines, tick
    %                                      values and labels (*constant*).
    %
    %                                      **Note:** The default color for all axes
    %                                      components is defined as :attr:`!gray80`
    %                                      (hex: #CCCCCC) of the color class
    %                                      :class:`~WBM.wbmColor`.
    %   DF_AXES_VWPTS      (cell, matrix): List of default viewpoint positions to
    %                                      determine the orientation of each axes
    %                                      (*constant*).
    %
    %                                      Following default axes viewpoints are
    %                                      defined:
    %
    %                                      .. tabularcolumns:: |c|c|
    %
    %                                      +-----------------+--------------------------------+
    %                                      | Viewpoint Name: | View Angles :math:`[az, el]`:  |
    %                                      +=================+================================+
    %                                      | ``'front'``     | :math:`[\ang{-90}, \ang{1}]`   |
    %                                      +-----------------+--------------------------------+
    %                                      | ``'side_l'``    | :math:`[\ang{0}, \ang{1}]`     |
    %                                      +-----------------+--------------------------------+
    %                                      | ``'side_r'``    | :math:`[\ang{180}, \ang{1}]`   |
    %                                      +-----------------+--------------------------------+
    %                                      | ``'top'``       | :math:`[\ang{-90}, \ang{90}]`  |
    %                                      +-----------------+--------------------------------+
    %                                      | ``'custom'``    | :math:`[\ang{-37.5},\ang{30}]` |
    %                                      +-----------------+--------------------------------+
    %
    %   DF_VWPTS_ANNOT  (cellstr, vector): Default list with following annotation strings (titles)
    %                                      for the default axes viewpoints (*constant*):
    %
    %                                         ``{'front', 'l. side', 'r. side', 'top', 'perspective'}``.
    %   DF_AXES_VIEWS      (cell, vector): Default viewpoint name list that specifies in the
    %                                      visualizer the view of each axes (*constant*):
    %
    %                                         ``{'custom', 'top', 'side_l', 'front'}``.
    %   DF_AXIS_LIMITS   (double, vector): Default (6 x 1) vector of the form
    %                                      :math:`[x_{min}, x_{max}, y_{min}, y_{max}, z_{min}, z_{max}]`,
    %                                      to specify the axis limits of every Cartesian
    %                                      axes in the figure window (*constant*).
    %   DF_GROUND_SHAPE  (double, matrix): Default (3 x 4) polygon matrix, consisting of
    %                                      four rectangular vertex coordinates, which form
    %                                      the shape of the ground floor plane (*constant*).
    %   DF_GROUND_COLOR  (double, vector): Default color of the ground, specified by
    %                                      the RGB-triplet :math:`[153  153  204]`
    %                                      (hex: #9999CC) (*constant*).
    %   DF_GROUND_COLOR2 (double, vector): Optional default color for the ground, specified
    %                                      by the RGB-triplet :math:`[201  220  222]`
    %                                      (hex: #C9DCDE) (*constant*).
    %   DF_LIGHT_POS     (double, vector): Default location of the point light, specified
    %                                      by the Cartesian vector :math:`[0.25, 0, 1.25]`
    %                                      (*constant*).
    %   DF_VID_FILENAME    (char, vector): Default filename string for creating a video
    %                                      file of the the current robot simulation:
    %                                      ``'robot-simulation.avi'`` (*constant*).
    %   DF_VID_FPS       (double, scalar): Default rate of video playback in *frames
    %                                      per second* :math:`[\mathrm{fps}]`:
    %                                      30 (*constant*).
    %
    %   robot_body         (:class:`~WBM.wbmSimBody`): Data object to define the geometric shape
    %                                                  for the body of the simulated robot.
    %   environment (:class:`~WBM.wbmSimEnvironment`): Data object to define the environment
    %                                                  settings for the robot simulation.
    %   trajectories (:class:`~WBM.wbmLinkTrajectory`, vector): Array of trajectory objects to show the
    %                                                           trajectory curves of specific links of
    %                                                           the robot.
    %   target_pts (:class:`~WBM.wbmTargetPoint`, vector): Array of target points that should reached
    %                                                      in the simulation by specific links of the
    %                                                      robot.
    %   hWndFigure (:class:`!matlab.ui.Figure`): Figure window for the visualization
    %                                            of the robot simulation.
    %   wnd_title                (char, vector): Title (name) of the figure window.
    %   wnd_pos                (double, vector): Location of the left inner corner of
    %                                            the figure window with the origin at
    %                                            the left bottom corner of the primary
    %                                            display, specified as a row-vector of
    %                                            the form :math:`[pos_{left}, pos_{bottom}]`
    %                                            (default location: :attr:`~WBM.genericSimConfig.DF_WND_POS`).
    %   wnd_size               (double, vector): Size of the figure window, specified as
    %                                            a row-vector of the form :math:`[width, height]`
    %                                            (default size: :attr:`~WBM.genericSimConfig.DF_WND_SIZE`).
    %   show_wnd              (logical, scalar): Boolean flag to determine if the figure
    %                                            should be displayed on the screen. If the
    %                                            value is set to *false*, then the figure
    %                                            is after the simulation setup disabled
    %                                            and invisible on the screen. This can be
    %                                            helpful for performance reasons, e.g., to
    %                                            increase the speed of creating a video of
    %                                            the current simulation (default: *true*).
    %   nAxes                   (uint8, scalar): Number of axes graphics objects in
    %                                            the figure window (default number:
    %                                            :attr:`~WBM.genericSimConfig.DF_AXES_NBR`).
    %   hAxes                  (double, vector): (1 x *nAxes*) array with handles
    %                                            of the created axes objects.
    %   axes_pos               (double, matrix): Matrix with size and position of
    %                                            each axes within the figure window,
    %                                            specified as row-vectors of the form
    %                                            :math:`[pos_{left}, pos_{bottom}, width, height]`
    %                                            (default positions: :attr:`~WBM.genericSimConfig.DF_AXES_POS`).
    %   axes_colors            (double, matrix): Colors for the axes back planes and
    %                                            for the axis lines, tick values and
    %                                            labels, specified by RGB-triplets as
    %                                            row-vectors (default colors:
    %                                            :attr:`~WBM.genericSimConfig.DF_AXES_COLORS`).
    %   axes_views               (cell, vector): Row-array of different string matching viewpoint
    %                                            names to define the robot simulation at each
    %                                            axes in a distinctive view (default views:
    %                                            :attr:`~WBM.genericSimConfig.DF_AXES_VIEWS`).
    %   axis_limits            (double, vector): (6 x 1) vector of the form
    %                                            :math:`[x_{min}, x_{max}, y_{min}, y_{max}, z_{min}, z_{max}]`,
    %                                            to specify the axis limits of every
    %                                            Cartesian axes in the figure window
    %                                            (default limits: :attr:`~WBM.genericSimConfig.DF_AXIS_LIMITS`).
    %   vwpts_annot              (cell, vector): Row-list with short annotation strings (or titles)
    %                                            for the given viewpoints in the figure window.
    %   gfx_objects              (cell, vector): (1 x *nAxes*) array of *graphics object handles*
    %                                            as data container for each axes in the figure
    %                                            window to store, manipulate and visualize all
    %                                            graphics objects (environment, robot body,
    %                                            trajectories and target points) of the simulation.
    %   show_light            (logical, scalar): Boolean flag to enable a point light in the
    %                                            simulation environment that radiates from a
    %                                            specified location in all directions
    %                                            (default: *false*).
    %   light_pos              (double, vector): Location of the point light, specified
    %                                            by a Cartesian vector :math:`[x, y, z]`.
    %   show_titles           (logical, scalar): Boolean flag to show the viewpoint titles
    %                                            in the left bottom corner of each axes
    %                                            (default: *true*).
    %   tit_font_sz            (double, scalar): Font size of the viewpoint titles, specified
    %                                            as a scalar numeric value.
    %   tit_font_color    (double/char, vector): Font color of the viewpoint titles, specified
    %                                            by a RGB-triplet or a color name.
    %   show_legend           (logical, scalar): Boolean flag to show in the figure window
    %                                            the legend for the link trajectories
    %                                            of the robot (default: *true*).
    %   lgd_font_sz            (double, scalar): Font size of the legend, specified
    %                                            as a scalar numeric value.
    %   lgd_font_color    (double/char, vector): Font color of the legend, specified
    %                                            by a RGB-triplet or a color name.
    %   lgd_bkgrd_color   (double/char, vector): Background color of the legend, specified
    %                                            by a RGB-triplet or a color name.
    %   lgd_edge_color (double or char, vector): Edge color of the box outline of the legend,
    %                                            specified by a RGB-triplet or a color name.
    %   lgd_location             (char, vector): Location of the legend with respect to the
    %                                            axes. The location values are the same as
    %                                            listed in the table of the *legend properties*
    %                                            of Matlab.
    %   lgd_orient               (char, vector): Orientation of the legend, specified by one of
    %                                            these values: *'vertical'*, *'horizontal'*.
    %   mkvideo               (logical, scalar): Boolean flag to create a video of the current
    %                                            robot simulation (default: *false*).
    %
    %                                            **Note:** If the flag is enabled by setting
    %                                            the value to *true*, then the figure window
    %                                            will not shown on the screen, to speed up
    %                                            the creation of the video.
    %   vid_axes_idx            (uint8, scalar): Index of an axes graphics object in the
    %                                            figure window to capture an image or video
    %                                            only from a particular subplot frame. If
    %                                            the value is set to 0, then the entire
    %                                            figure window (all axes) will be captured
    %                                            for the image or video (default value: 0).
    %   vid_filename             (char, vector): The filename of the video with the ``.avi``
    %                                            extension and the full path where the file
    %                                            will be written.
    %
    %                                            **Note:** Currently only the *uncompressed
    %                                            AVI* file format can be used for creating
    %                                            video files, because only this file format
    %                                            will be supported by Matlab on all platforms.
    %   vid_fps                (double, scalar): Frame rate of the video playback in frames per
    %                                            second :math:`[\mathrm{fps}]`, specified by a
    %                                            positive number.
    %   pl_stack                 (cell, matrix): Index stack (list) for the volume body objects that
    %                                            are defined as payloads for particular manipulators
    %                                            (hands) of the robot. Each payload object of the
    %                                            stack will be processed successively and is assigned
    %                                            to a manipulator of the robot, specified by one of
    %                                            these values:
    %
    %                                               - ``'lh'``: Use the *left hand* as manipulator.
    %                                               - ``'rh'``: Use the *right hand* as manipulator.
    %                                               - ``'bh'``: Use *both hands* to manipulate the object.
    %
    %                                            **Note:** The payload stack is a 2-dimensional
    %                                            array. In the first column are listed the
    %                                            reference indices to the volume body objects
    %                                            of the environment and in the second column is
    %                                            each object linked to a particular manipulator
    %                                            (hand) for processing.
    %   pl_time_idx            (uint32, matrix): Utilization time matrix of the given payload objects
    %                                            in the stack to determine the points in time for the
    %                                            simulation when each object is grabbed and released
    %                                            by the specified manipulator.
    %
    %                                            **Note:** The rows of the index matrix representing
    %                                            time index vectors of the form, :math:`[idx_{start}, idx_{end}]`,
    %                                            where :math:`idx_{start}` denotes the time index
    %                                            when the object is grabbed and :math:`idx_{end}`
    %                                            is the time index when the object is released by
    %                                            the manipulator.
    %   zoom_axes              (double, matrix): Zoom index matrix to specify which axes of the
    %                                            figure window should be zoomed by a particular
    %                                            zoom factor.
    %
    %                                            **Note:** Each row of the index matrix is
    %                                            of the form :math:`[idx_{ax}, zfac]`, where
    %                                            :math:`idx_{ax}` denotes the reference index
    %                                            of an axes graphics object in the figure and
    %                                            :math:`zfac` is the corresponding zoom factor.
    %                                            If :math:`zfac > 1`, the scene appears larger
    %                                            (*zoom in*); if :math:`zfac \geq 0` and
    %                                            :math:`zfac < 1`, the scene appears smaller
    %                                            (*zoom out*). If :math:`zfac = 1`, then the
    %                                            scene keeps its original size. The row number
    %                                            of the matrix can grow at most *nAxes* long.
    %   shift_axes               (cell, matrix): Shift index array to specify which axes of
    %                                            the figure window should be shifted in a
    %                                            particular direction within the axis limits
    %                                            that are scaled to a given zoom factor.
    %                                            This is very useful if only a special area
    %                                            of the scene is of interest.
    %
    %                                            **Note:** The index array is a 2-element
    %                                            row-array of the form :math:`\{idx_{ax}, v_{sh}\}`.
    %                                            The first element :math:`idx_{ax}` of the
    %                                            array represents a column-vector with
    %                                            reference indices to the axes graphics
    %                                            objects in the figure window. The second
    %                                            element :math:`v_{sh}` describes a (n x 3)
    %                                            matrix of Cartesian vectors of the form
    %                                            :math:`[x, y, z]`. Each row-vector of the
    %                                            matrix specifies the *shift direction* of
    %                                            the scene of the current axes. The row
    %                                            numbers of both array elements must be
    %                                            equal and can grow at most *nAxes* long.
    properties(Dependent)
        % public properties for fast get/set methods:
        custom_view@double  vector
        axes_vwpts@cell     matrix
        wnd_visible@logical scalar
    end

    properties(Constant)
        DF_WND_POS@double       vector = [50  400]; % [x (left), y (bottom)]
        DF_WND_SIZE@double      vector = [600 650]; % [width, height]
        DF_AXES_NBR@uint8       scalar = 4; % number of axes to divide the figure into a m-by-n grid.
        % position of the new axes in the figure, specified as a four-element
        % vector [x (left), y (bottom), wid (width), hgt (height)]:
        %                                  x     y     wid   hgt
        DF_AXES_POS@double      matrix = [0.54  0.05  0.45  0.42;
                                          0.01  0.05  0.45  0.42;
                                          0.54  0.60  0.45  0.42;
                                          0.01  0.60  0.45  0.42];
        DF_AXES_COLORS@double   matrix = repmat(WBM.wbmColor.gray80, 4, 1);
        % default viewpoint positions to determine the orientation of the axes
        % [az (azimuth), el (elevation)]:
        %                                             az    el
        DF_AXES_VWPTS@cell      matrix = {'front',  [-90     1];
                                          'side_l', [ 0      1];
                                          'side_r', [ 180    1];
                                          'top',    [-90    90];
                                          'custom', [-37.5  30]};
        DF_VWPTS_ANNOT@cell     vector = {'front', 'l. side', 'r. side', ...
                                          'top', 'perspective'};
        % default viewpoints of the simulation to show in the visualizer:
        DF_AXES_VIEWS@cell      vector = {'custom', 'top', 'side_l', 'front'};

        DF_AXIS_LIMITS@double   vector = [-0.5  0.5  -0.42  0.58  0  1];

        DF_GROUND_SHAPE@double  matrix = [-0.45  -0.45  0.45   0.45;  % x
                                          -0.37   0.53  0.53  -0.37;  % y
                                              0      0     0      0]; % z
        DF_GROUND_COLOR@double  vector = [0.6  0.6  0.8];
        DF_GROUND_COLOR2@double vector = [201  220  222] ./ 255; % hex: #c9dcde

        DF_LIGHT_POS@double     vector = [0.25  0  1.25]; % [x,y,z]

        DF_VID_FILENAME@char           = 'robot-simulation.avi';
        DF_VID_FPS@double              = 30; % frames per second (frame rate)
    end

    properties
        robot_body@WBM.wbmSimBody
        environment@WBM.wbmSimEnvironment
        trajectories@WBM.wbmLinkTrajectory vector = WBM.wbmLinkTrajectory.empty;
        target_pts@WBM.wbmTargetPoint      vector = WBM.wbmTargetPoint.empty;

        hWndFigure@matlab.ui.Figure
        wnd_title@char             = '';
        wnd_pos@double      vector = WBM.genericSimConfig.DF_WND_POS;
        wnd_size@double     vector = WBM.genericSimConfig.DF_WND_SIZE;
        show_wnd@logical    scalar = true;

        nAxes@uint8         scalar = WBM.genericSimConfig.DF_AXES_NBR;
        hAxes@double        vector = [];
        axes_pos@double     matrix = [];
        axes_colors@double  matrix = [];
        axes_views@cell     vector = {};
        axis_limits@double  vector = WBM.genericSimConfig.DF_AXIS_LIMITS;

        vwpts_annot@cell    vector = {};
        gfx_objects@cell    vector = {};

        show_light@logical  scalar = false;
        light_pos@double    vector = WBM.genericSimConfig.DF_LIGHT_POS;

        show_titles@logical scalar = true;
        tit_font_sz@double  scalar = 9;
        tit_font_color             = WBM.wbmColor.dimgray;

        show_legend@logical scalar = false;
        lgd_font_sz@double  scalar = 9;
        lgd_font_color             = 'black';
        lgd_bkgrd_color            = WBM.wbmColor.whitesmoke;
        lgd_edge_color             = WBM.wbmColor.dimgray;
        lgd_location@char          = 'northeast';
        lgd_orient@char            = 'vertical';

        mkvideo@logical     scalar = false;
        vid_axes_idx@uint8  scalar = 0; % 0 ... capture all axes (entire figure window) for the video
        vid_filename@char          = WBM.genericSimConfig.DF_VID_FILENAME;
        vid_fps@double      scalar = WBM.genericSimConfig.DF_VID_FPS;
    end

    properties(SetAccess = protected, GetAccess = public)
        pl_stack@cell      matrix = {};         % index list of volume body objects that are defined as payload.
        pl_time_idx@uint32 matrix = uint32([]); % utilization time index of the payload objects [start_idx (obj. grabbed), end_idx (obj. released)].

        % index lists of axes to be zoomed or shifted:
        zoom_axes@double   matrix = []; % [ax_idx, zfac], zfac = 1 ... keeps the original size (zoom in: zfac > 1, zoom out: zfac > 0 and zfac < 1)
        shift_axes@cell    matrix = {}; % {ax_idx, sh_vec}, sh_vec = [x,y,z]
    end

    properties(Access = protected)
        maxes_vwpts@cell matrix = WBM.genericSimConfig.DF_AXES_VWPTS; % list of axes viewpoint positions
    end

    methods
        function obj = genericSimConfig(wnd_title, rob_sim_body, varargin)
            % Constructor.
            %
            % The constructor of the *generic simulation configuration* can be
            % called in two different ways, where the *keyword arguments* in
            % the square brackets are optional:
            %
            %   - .. function:: genericSimConfig(wnd_title, rob_sim_body[, nax[, env_settings]])
            %   - .. function:: genericSimConfig(wnd_title, rob_sim_body[, [nax|env_settings]])
            %
            % Arguments:
            %   wnd_title                (char, vector): String to specify the title
            %                                            (name) of the figure window.
            %   rob_sim_body (:class:`~WBM.wbmSimBody`): Data object to define the geometric
            %                                            shape for the body of the simulated
            %                                            robot.
            %   varargin: Variable-length input argument list.
            %
            % Keyword Arguments:
            %   nax                              (int, scalar): The number of axes graphics objects
            %                                                   to be created in the figure window.
            %   env_settings (:class:`~WBM.wbmSimEnvironment`): Data object to define the environment
            %                                                   settings for the robot simulation.
            % Returns:
            %   obj: An instance of the :class:`!genericSimConfig` data type.
            switch nargin
                case 4
                    % nax          = varargin{1}
                    % env_settings = varargin{2}
                    obj.nAxes       = varargin{1,1};
                    obj.environment = varargin{1,2};
                case 3
                    if isa(varargin{1,1}, 'WBM.wbmSimEnvironment')
                        % env_settings = varargin{1}
                        obj.environment = varargin{1,1};
                    elseif isscalar(varargin{1,1})
                        % nax = varargin{1}
                        obj.nAxes = varargin{1,1};
                        defaultEnvSettings(obj);
                    else
                        error('genericSimConfig::genericSimConfig: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                    end
                case 2
                    % use the default environment settings ...
                    defaultEnvSettings(obj);
                otherwise
                    error('genericSimConfig::genericSimConfig: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            obj.wnd_title  = wnd_title;
            obj.robot_body = rob_sim_body;

            obj.hAxes       = zeros(1,obj.nAxes);
            obj.gfx_objects = cell(1,obj.nAxes);

            if (obj.nAxes <= 4)
                % use the default axes settings ...
                obj.axes_pos    = WBM.genericSimConfig.DF_AXES_POS;
                obj.axes_colors = WBM.genericSimConfig.DF_AXES_COLORS;
                obj.axes_views  = WBM.genericSimConfig.DF_AXES_VIEWS;
                if obj.show_titles
                    obj.vwpts_annot = WBM.genericSimConfig.DF_VWPTS_ANNOT;
                end
            end
        end

        function zoomAxes(obj, axes_idx, zoom_fac)
            % Specifies the axes in the figure window that should be zoomed in
            % or out by a particular *zoom factor*.
            %
            % Arguments:
            %   axes_idx    (int, vector): Vector with the index numbers of the axes
            %                              that should be zoomed in the figure window.
            %   zoom_fac (double, vector): Vector with the corresponding *zoom factors*
            %                              of the axes that should be zoomed in or out.
            % Note:
            %   Both vectors must have the same length and can not exceed the size of *nAxes*.
            if ( ~isvector(axes_idx) || ~isvector(zoom_fac) )
                error('genericSimConfig::zoomAxes: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            axes_idx = axes_idx(:);
            zoom_fac = zoom_fac(:);

            if ~isreal(zoom_fac)
                error('genericSimConfig::zoomAxes: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            n = size(axes_idx,1);
            if (n ~= size(zoom_fac,1))
                error('genericSimConfig::zoomAxes: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            if ( (max(axes_idx) > obj.nAxes) || (min(axes_idx) < 1) )
                error('genericSimConfig::zoomAxes: %s', WBM.wbmErrorMsg.VAL_OUT_OF_BOUNDS);
            end
            if (n > 1)
                WBM.utilities.chkfun.checkNumListAscOrder(axes_idx, 'genericSimConfig::zoomAxes')
            end
            obj.zoom_axes = zeros(n,2);
            obj.zoom_axes(1:n,1) = axes_idx; % axes indices to be zoomed
            obj.zoom_axes(1:n,2) = zoom_fac; % zoom factors
        end

        function shiftAxes(obj, axes_idx, shift_pos)
            % Specifies the axes in the figure window that should be shifted to
            % a particular position of interest.
            %
            % Arguments:
            %   axes_idx     (int, vector): Vector with the index numbers of the axes
            %                               that should be zoomed in the figure window.
            %   shift_pos (double, matrix): (n x 3) matrix with the corresponding Cartesian
            %                               *shift coordinates* of the axes that should be
            %                               shifted.
            % Note:
            %   The index vector and the shift matrix must have the same length and can not
            %   exceed the size of *nAxes*.
            if ( ~isvector(axes_idx) || ~ismatrix(shift_pos) )
                error('genericSimConfig::shiftAxes: %s', WBM.wbmErrorMsg.WRONG_VEC_DIM);
            end
            axes_idx = axes_idx(:);

            if ( ~isreal(axes_idx) || ~isreal(shift_pos) )
                error('genericSimConfig::shiftAxes: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            [m, n] = size(shift_pos);
            if (m ~= size(axes_idx,1))
                error('genericSimConfig::shiftAxes: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end
            if (n ~= 3)
                error('genericSimConfig::shiftAxes: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
            end
            if ( (max(axes_idx) > obj.nAxes) || (min(axes_idx) < 1) )
                error('genericSimConfig::shiftAxes: %s', WBM.wbmErrorMsg.VAL_OUT_OF_BOUNDS);
            end
            if (m > 1)
                WBM.utilities.chkfun.checkNumListAscOrder(axes_idx, 'genericSimConfig::shiftAxes')
            end
            obj.shift_axes = cell(1,2);
            obj.shift_axes{1,1} = uint8(axes_idx); % axes indices to be shifted
            obj.shift_axes{1,2} = shift_pos;       % shift positions [x, y, z]
        end

        function addView(obj, vp_name, vp)
            % Adds a new view to the viewpoint positions list that determines
            % the axes orientations in the figure window.
            %
            % Arguments:
            %   vp_name (char, vector): Variable name of the new viewpoint (axes
            %                           orientation name).
            %   vp    (double, vector): (1 x 3) Cartesian viewpoint position vector.
            %
            % Note:
            %   For flexibility reasons, the viewpoint list may contain more views
            %   than axes are indicated in the figure window.
            if ~ischar(vp_name)
                error('genericSimConfig::addView: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            WBM.utilities.chkfun.checkRVecDim(vp, 2, 'genericSimConfig::addView');

            if strcmp(vp_name, obj.axes_views)
                error('genericSimConfig::addView: The name is already reserved!');
            end
            len = size(obj.maxes_vwpts,1);
            idx = len + 1;
            obj.maxes_vwpts{idx,1} = vp_name;
            obj.maxes_vwpts{idx,2} = vp;
        end

        function createVideo(obj, varargin)
            % Sets the settings for creating a video of the current robot
            % simulation.
            %
            % The method can be called in three different ways:
            %
            %   - .. function:: createVideo(filename[, [rmode, fps]])
            %   - .. function:: createVideo(filename[, [rmode|fps]])
            %   - .. function:: createVideo()
            %
            % If the parameters are not completely given, then the method uses
            % the default values for the video settings.
            %
            % Arguments:
            %   varargin: Variable-length input argument list.
            %
            % Keyword Arguments:
            %   filename (char, vector): The filename of the video with the full path
            %                            where the file will be written.
            %
            %                            **Note:** Currently only the *uncompressed
            %                            AVI* file format can be used, since Matlab
            %                            supports only this format on all platforms.
            %   rmode    (char, vector): Render mode to specify the rendering method
            %                            for the graphics objects, specified by one
            %                            of these values:
            %
            %                               - ``'r_fast'``: Fast rendering mode (*childorder*),
            %                                 where the objects will be drawn in the order in
            %                                 which they are created by the graphics functions
            %                                 (*default*).
            %                               - ``'r_depth'``: Draw the graphics objects by a
            %                                 *depth-sort method* (back-to-front order) based
            %                                 on the current view.
            %
            %                                 **Note:** Without further optimizations this
            %                                 method is mostly slow and can cause "animation
            %                                 flickering" during the simulation.
            %   fps       (int, scalar): Frame rate of the video playback in frames
            %                            per second :math:`[\mathrm{fps}]` (default
            %                            rate: :attr:`~WBM.genericSimConfig.DF_VID_FPS`).
            % Note:
            %   The method enables in the configuration object the boolean property
            %   :attr:`~WBM.genericSimConfig.mkvideo` with the effect, that the figure
            %   will not displayed on the screen during the creation of the video. This
            %   will increase the performance of creating the video.
            filename = WBM.genericSimConfig.DF_VID_FILENAME;
            fps      = WBM.genericSimConfig.DF_VID_FPS;
            rmode    = 'r_fast';

            if (nargin > 1)
                switch nargin
                    case 2
                        filename = varargin{1,1};
                    case 3
                        filename = varargin{1,1};
                        if ischar(varargin{1,2})
                            rmode = varargin{1,2};
                        else
                            fps = varargin{1,2};
                        end
                    case 4
                        filename = varargin{1,1};
                        rmode    = varargin{1,2};
                        fps      = varargin{1,3};
                end
            end
            renderMode(obj, rmode, 'off');

            obj.mkvideo      = true;
            obj.vid_filename = filename;
            obj.vid_fps      = fps;
        end

        function renderMode(obj, rmode, vis)
            % Specifies the *rendering method* to be used for the graphics
            % objects during the animation of the robot.
            %
            % Arguments:
            %   rmode (char, vector): Render mode, specified by one of these
            %                         values:
            %                            - ``'r_fast'``: Fast rendering mode (*childorder*),
            %                              where the objects will be drawn in the order in
            %                              which they are created by the graphics functions
            %                              (*default*).
            %                            - ``'r_depth'``: Draw the graphics objects by a
            %                              *depth-sort method* (back-to-front order) based
            %                              on the current view.
            %
            %                              **Note:** Without further optimizations this
            %                              method is mostly slow and can cause "animation
            %                              flickering" during the simulation.
            %   vis   (char, vector): Determines if the figure window will be *visible*
            %                         on the screen, specified as *'on'* or *'off'*
            %                         (default value: *'on'*).
            % Note:
            %   This method requires a Matlab version of at least R2014b (8.4.0) or higher!
            if isempty(obj.hWndFigure)
                error('genericSimConfig::renderMode: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
            end
            if (nargin == 2)
                % show the figure on the screen (default)
                vis = 'on';
            end
            set(obj.hWndFigure, 'Visible', vis);

            if ~verLessThan('matlab', '8.4.0')
                switch rmode
                    case 'r_fast'
                        sort_meth = 'childorder';
                    case 'r_depth'
                        sort_meth = 'depth';
                    otherwise
                        error('genericSimConfig::renderMode: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
                end

                for i = 1:obj.nAxes
                    set(obj.hWndFigure, 'CurrentAxes', obj.hAxes(1,i));
                    set(gca, 'SortMethod', sort_meth);
                end
                return
            end
            % else ...
            warning('genericSimConfig::renderMode: Matlab R2014b or higher is required.');
        end

        function vp = get.custom_view(obj)
            idx = find(strcmp(obj.maxes_vwpts(1:end,1), 'custom'));
            if isempty(idx)
                error('genericSimConfig::get.custom_view: %s', WBM.wbmErrorMsg.NAME_NOT_EXIST);
            end
            vp = obj.maxes_vwpts{idx,2};
        end

        function set.custom_view(obj, vp)
            WBM.utilities.chkfun.checkRVecDim(vp, 2, 'genericSimConfig::set.custom_view');
            idx = find(strcmp(obj.maxes_vwpts(1:end,1), 'custom'));
            if isempty(idx)
                % define a custom viewpoint ...
                idx = size(obj.maxes_vwpts,1) + 1;
                obj.maxes_vwpts{idx,1} = 'custom';
            end
            obj.maxes_vwpts{idx,2} = vp;
        end

        function axes_vp = get.axes_vwpts(obj)
            axes_vp = obj.maxes_vwpts;
        end

        function set.wnd_visible(obj, vis)
            if ~vis
                set(obj.hWndFigure, 'Visible', 'off');
            else
                set(obj.hWndFigure, 'Visible', 'on');
            end
            obj.show_wnd = vis;
        end

    end

    methods(Access = protected)
        function defaultEnvSettings(obj)
            obj.environment = WBM.wbmSimEnvironment;
            obj.environment.bkgrd_color_opt = 'white';
            obj.environment.grnd_shape      = WBM.genericSimConfig.DF_GROUND_SHAPE;
            obj.environment.grnd_color      = WBM.genericSimConfig.DF_GROUND_COLOR;
            obj.environment.grnd_edge_color = 'black';
            obj.environment.orig_pt_color   = 'black';
            obj.environment.orig_pt_size    = 3.5;
            obj.environment.vb_objects      = WBM.vbObject.empty;
        end

    end
end
