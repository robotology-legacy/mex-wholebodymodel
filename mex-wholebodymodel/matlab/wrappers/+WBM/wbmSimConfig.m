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

classdef (Abstract) wbmSimConfig < handle
    % :class:`!wbmSimConfig` is an *abstract data type class* and represents the
    % *base configuration interface* for the visualizer of the robot simulation.
    %
    % The abstract class increases the flexibility and enables for the users to
    % derive user specific configuration classes for the visualization of the
    % robot.
    %
    % Attributes:
    %   custom_view (double, vector): Custom viewpoint specification in terms of
    %                                 *azimuth* (az) and *elevation* (el) as a
    %                                 row-vector :math:`[az, el]`.
    %   axes_vwpts    (cell, matrix): List of predefined viewpoint positions for
    %                                 the orientation of the axes (*readonly*).
    %                                 The list is a 2-dimensional array where in
    %                                 the first column are the names of the axes
    %                                 orientations and in the second column the
    %                                 corresponding viewpoint vectors.
    %
    %   DF_WND_POS      (double, vector): Default position of the window (*constant*).
    %   DF_WND_SIZE     (double, vector): Default window size of the figure (*constant*).
    %   DF_AXES_NBR      (uint8, scalar): Default number of axes graphics objects
    %                                     in the figure window (*constant*).
    %   DF_AXES_POS     (double, matrix): Default positions of each axes graphics
    %                                     object in the figure window (*constant*).
    %
    %                                     **Note:** The axes positions must be
    %                                     stored in a (n x 4) matrix. Each row of
    %                                     the matrix represents an axes position.
    %                                     The row number of the matrix must be
    %                                     the same as the number of axes.
    %   DF_AXES_COLORS  (double, matrix): Default colors for the axes back planes
    %                                     and for the axis lines, tick values and
    %                                     labels (*constant*).
    %
    %                                     **Note:** The colors must be specified
    %                                     in a (4 x 3) matrix with horizontal
    %                                     RGB-triplets. The first triplet defines
    %                                     the color for the axes back planes and
    %                                     all other colors are for the axis lines
    %                                     tick values and labels in the x, y and
    %                                     z direction.
    %   DF_AXES_VWPTS     (cell, matrix): List of default viewpoint positions to
    %                                     determine the orientation of each axes
    %                                     (*constant*).
    %
    %                                     **Note:** The list must be an 2-dimensional
    %                                     array with the viewpoint names in the first
    %                                     column and in the second column the specified
    %                                     viewpoints (row-vectors) with *azimuth* (az)
    %                                     and *elevation* (el) values as terms.
    %
    %   DF_AXES_VIEWS     (cell, vector): Default row-array of string matching viewpoint
    %                                     names that shows the robot simulation at each
    %                                     axes with a different view (*constant*).
    %   DF_AXIS_LIMITS  (double, vector): Default (6 x 1) vector of the form
    %                                     :math:`[x_{min}, x_{max}, y_{min}, y_{max}, z_{min}, z_{max}]`,
    %                                     to specify the axis limits of every Cartesian
    %                                     axes in the figure window (*constant*).
    %   DF_GROUND_SHAPE (double, matrix): Default polygon as a set of vertical
    %                                     XYZ-triplets that forms the shape of
    %                                     the ground floor plane (*constant*).
    %   DF_GROUND_COLOR (double, vector): Default ground color specified by a
    %                                     RGB-triplet (*constant*).
    %
    %   robot_body         (:class:`~WBM.wbmSimBody`): Data object to define the geometric shape
    %                                                  for the body of the simulated robot.
    %   environment (:class:`~WBM.wbmSimEnvironment`): Data object to define the environment
    %                                                  for the robot simulation.
    %   trajectories (:class:`~WBM.wbmLinkTrajectory`, vector): Array of trajectory objects to show the
    %                                                           trajectory curves of specific links of
    %                                                           the robot.
    %   target_pts (:class:`~WBM.wbmTargetPoint`, vector): Array of target points that should reached
    %                                                      in the simulation by specific links of the
    %                                                      robot.
    %   hWndFigure       (``matlab.ui.Figure``): Figure window for visualizing the
    %                                            robot simulation.
    %   wnd_title                (char, vector): Title of the figure window.
    %   wnd_pos                (double, vector): Location of the left inner corner of
    %                                            figure window with the origin at the
    %                                            left bottom corner of the primary
    %                                            display. The property is defined
    %                                            as a row-vector of the form
    %                                            :math:`[pos_{left}, pos_{bottom}]`
    %                                            (default location: :attr:`DF_WND_POS`).
    %   wnd_size               (double, vector): Size of the figure window. The
    %                                            Property is defined as a row-vector
    %                                            with :math:`[width, height]`
    %                                            (default size: :attr:`DF_WND_SIZE`).
    %   show_wnd              (logical, scalar): Boolean flag to determine if the figure
    %                                            should be shown on the screen. If the
    %                                            value is set to *false*, then the figure
    %                                            is invisible. This helpful, e.g., to
    %                                            increase the speed of creating a video
    %                                            of the current simulation (default: *true*).
    %   nAxes                   (uint8, scalar): Number of axes graphics objects in
    %                                            the figure window (default number:
    %                                            :attr:`DF_AXES_NBR`).
    %   hAxes                  (double, vector): (1 x *nAxes*) array of handles
    %                                            of the created axes objects.
    %   axes_pos               (double, matrix): Matrix with size and position of each
    %                                            axes within the figure window and is
    %                                            specified by row-vectors of the form
    %                                            :math:`[pos_{left}, pos_{bottom}, width, height]`
    %                                            (default positions: :attr:`DF_AXES_POS`).
    %   axes_colors            (double, matrix): Colors for the axes back planes and
    %                                            for the axis lines, tick values and
    %                                            labels, specified by RGB-triplets as
    %                                            row-vectors (default colors:
    %                                            :attr:`DF_AXES_COLORS`).
    %   axes_views               (cell, vector): Row-array of string matching viewpoint names
    %                                            that shows the robot simulation at each axes
    %                                            axes with a different view (default views:
    %                                            :attr:`DF_AXES_VIEWS`).
    %   axis_limits            (double, vector): (6 x 1) vector of the form
    %                                            :math:`[x_{min}, x_{max}, y_{min}, y_{max}, z_{min}, z_{max}]`,
    %                                            to specify the axis limits of every
    %                                            Cartesian axes in the figure window
    %                                            (default limits: :attr:`DF_AXIS_LIMITS`).
    %   vwpts_annot              (cell, vector): String list (row) of titles for each viewpoint
    %                                            in the figure window.
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
    %   tit_font_color (double or char, vector): Font color of the viewpoint titles, specified
    %                                            by a RGB-triplet or a color name.
    %   show_legend            (logical, scalar): Boolean flag to show in the figure window
    %                                             the legend for the link trajectories
    %                                             of the robot (default: *true*).
    %   lgd_font_sz             (double, scalar): Font size of the legend, specified
    %                                             as a scalar numeric value.
    %   lgd_font_color  (double or char, vector): Font color of the legend, specified
    %                                             by a RGB-triplet or a color name.
    %   lgd_bkgrd_color (double or char, vector): Background color of the legend, specified
    %                                             by a RGB-triplet or a color name.
    %   lgd_edge_color  (double or char, vector): Edge color of the box outline of the legend,
    %                                             specified by a RGB-triplet or a color name.
    %   lgd_location              (char, vector): Location of the legend with respect to
    %                                             axes. The location values are the same as
    %                                             listed in the table in the *legend properties*
    %                                             of Matlab.
    %   lgd_orient                (char, vector): Orientation of the legend, specified by one of
    %                                             these values: *'vertical'*, *'horizontal'*.
    %   mkvideo                (logical, scalar): Boolean flag to create a video of the current
    %                                             robot simulation (default: *false*).
    %
    %                                             **Note:** If the flag is enabled by setting
    %                                             the value to *true*, then the figure window
    %                                             will not shown on the screen to speed up the
    %                                             creation of the video.
    %   vid_axes_idx             (uint8, scalar): Index of an axes graphics object in the
    %                                             figure window, to capture an image or a
    %                                             video only from a specific subplot frame.
    %                                             If the value is set to 0, then the entire
    %                                             figure window (all axes) will be captured
    %                                             for the image or video (default value: 0).
    %   vid_filename              (char, vector): The filename of the video with the ``.avi``
    %                                             extension and the full path where the file
    %                                             will be written.
    %
    %                                             **Note:** Currently only the *uncompressed
    %                                             AVI* file format can be used for creating
    %                                             video files, because only this file format
    %                                             will be supported by Matlab on all platforms.
    %   vid_fps                 (double, scalar): Frame rate of the video playback in frames per
    %                                             second (:math:`\si{[fps]}`), specified by a
    %                                             positive number.
    %   pl_stack                  (cell, matrix): Index stack (list) for volume body objects that
    %                                             are defined as payloads for specific manipulators
    %                                             (hands) of the robot. Each payload object of the
    %                                             stack will be processed successively and is
    %                                             assigned to a manipulator of the robot, specified
    %                                             by one of these values:
    %
    %                                                - ``'lh'``: Use the *left hand* as manipulator.
    %                                                - ``'rh'``: Use the *right hand* as manipulator.
    %                                                - ``'bh'``: Use *both hands* to manipulate the object.
    %
    %                                             **Note:** The payload stack is a 2-dimensional
    %                                             array. In the first column are listed the
    %                                             reference indices to the volume body objects of
    %                                             the environment and in the second column is each
    %                                             object linked to a specific manipulator (hand)
    %                                             for processing.
    %   pl_time_idx             (uint32, matrix): Utilization time matrix of the given payload
    %                                             objects in the stack, to determine time points
    %                                             for the simulation when each object is grabbed
    %                                             and released by the specified manipulator.
    %
    %                                             **Note:** The rows of the index matrix representing
    %                                             time index vectors of the form, :math:`[idx_{start}, idx_{end}]`,
    %                                             where :math:`idx_{start}` denotes the time index
    %                                             when the object is grabbed and :math:`idx_{end}`
    %                                             is the time index when the object is released by
    %                                             the manipulator.
    %   zoom_axes               (double, matrix): Zoom index matrix to specify which axes of the
    %                                             figure window should be zoomed by a specific
    %                                             zoom factor.
    %
    %                                             **Note:** Each row of the index matrix is
    %                                             of the form :math:`[idx_{ax}, \textrm{zfac}]`,
    %                                             where :math:`idx_{ax}` denotes the reference
    %                                             index of an axes graphics object in the figure
    %                                             and :math:`\textrm{zfac}` is the corresponding
    %                                             zoom factor. If :math:`\textrm{zfac} > 1`,
    %                                             the scene appears larger (*zoom in*); if
    %                                             :math:`\textrm{zfac} \geq 0` and
    %                                             :math:`\textrm{zfac} < 1`, the scene appears
    %                                             smaller (*zoom out*). If :math:`\textrm{zfac} = 1`,
    %                                             then the scene keeps its original size. The
    %                                             row number of the matrix can grow at most
    %                                             *nAxes* long.
    %   shift_axes                (cell, matrix): Shift index array to specify which axes of
    %                                             the figure window should be shifted in a
    %                                             specific direction within the axis limits
    %                                             that are scaled to a given zoom factor.
    %                                             This is very useful if only a special area
    %                                             of the scene is of interest.
    %
    %                                             **Note:** The index array is a 2-element
    %                                             row-array of the form :math:`\{idx_{ax}, v_{sh}\}`.
    %                                             The first element :math:`idx_{ax}` of the
    %                                             array represents a column-vector with
    %                                             reference indices of the axes graphics
    %                                             objects in the figure window. The second
    %                                             element :math:`v_{sh}` describes a (n x 3)
    %                                             matrix of Cartesian vectors of the form
    %                                             :math:`[x, y, z]`. Each row-vector of the
    %                                             matrix specifies the *shift direction* of
    %                                             the scene of the current axes. The row
    %                                             numbers of both array elements must be
    %                                             equal and can grow at most *nAxes* long.
    properties(Abstract, Dependent)
        % public properties for fast get/set methods:
        custom_view@double vector
        axes_vwpts@cell    matrix
    end

    properties(Abstract, Constant)
        DF_WND_POS@double      vector
        DF_WND_SIZE@double     vector
        DF_AXES_NBR@uint8      scalar
        DF_AXES_POS@double     matrix
        DF_AXES_COLORS@double  matrix
        DF_AXES_VWPTS@cell     matrix
        DF_AXES_VIEWS@cell     vector
        DF_AXIS_LIMITS@double  vector
        DF_GROUND_SHAPE@double matrix
        DF_GROUND_COLOR@double vector
    end

    properties(Abstract)
        robot_body@WBM.wbmSimBody
        environment@WBM.wbmSimEnvironment
        trajectories@WBM.wbmLinkTrajectory vector
        target_pts@WBM.wbmTargetPoint      vector

        hWndFigure@matlab.ui.Figure
        wnd_title@char
        wnd_pos@double      vector
        wnd_size@double     vector
        show_wnd@logical    scalar

        nAxes@uint8         scalar
        hAxes@double        vector
        axes_pos@double     matrix
        axes_colors@double  matrix
        axes_views@cell     vector
        axis_limits@double  vector

        vwpts_annot@cell    vector
        gfx_objects@cell    vector

        show_light@logical  scalar
        light_pos@double    vector

        show_titles@logical scalar
        tit_font_sz@double  scalar
        tit_font_color

        show_legend@logical scalar
        lgd_font_sz@double  scalar
        lgd_font_color
        lgd_bkgrd_color
        lgd_edge_color
        lgd_location@char
        lgd_orient@char

        mkvideo@logical     scalar
        vid_axes_idx@uint8  scalar
        vid_filename@char
        vid_fps@double      scalar
    end

    properties(Abstract, SetAccess = protected, GetAccess = public)
        pl_stack@cell      matrix
        pl_time_idx@uint32 matrix

        zoom_axes@double   matrix
        shift_axes@cell    matrix
    end

    methods(Sealed)
        function setPayloadStack(obj, vb_idx, manip)
            % Sets the stack of payload objects (solid volume body objects) that
            % will be assigned to specific manipulators (hands) of the robot for
            % successive processing.
            %
            % Args:
            %   obj: The simulation configuration object.
            %   vb_idx (integer, vector): (n x 1) vector of *reference indices*
            %                             of solid volume body objects that are
            %                             defined as payloads.
            %   manip  (cellstr, vector): String array to assign each payload
            %                             object with a specific *manipulator*
            %                             (hand) of the robot, specified by one
            %                             of these values:
            %
            %                                - ``'lh'``: Use the *left hand* as manipulator.
            %                                - ``'rh'``: Use the *right hand* as manipulator.
            %                                - ``'bh'``: Use *both hands* to manipulate the object.
            len = length(vb_idx);

            if ischar(manip)
                manip = {manip};
            elseif ( isvector(vb_idx) && (len > 1) && iscellstr(manip) )
                % make sure that both lists are column vectors ...
                vb_idx = vb_idx(:);
                manip  = manip(:);
            else
                error('wbmSimConfig::setPayloadStack: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if (len ~= size(manip,1))
                error('wbmSimConfig::setPayloadStack: %s', WBM.wbmErrorMsg.DIM_MISMATCH);
            end

            % create the payload stack with object index and the used manipulator (hand):
            obj.pl_stack = cell(len, 2);
            for i = 1:len
                obj.pl_stack{i,1} = vb_idx(i,1);
            end

            for i = 1:len
                m = manip{i,1};
                switch m
                    case {'lh', 'rh', 'bh'}
                        % left hand, right hand or both hands:
                        obj.pl_stack{i,2} = m;
                    otherwise
                        error('wbmSimConfig::setPayloadStack: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
                end
            end
            % initialize the utilization time indices of the given payload objects:
            obj.pl_time_idx = uint32(zeros(len,2)); % [start_idx, end_idx]
        end

        function setPayloadUtilTime(obj, obj_idx, start_idx, end_idx)
            % Sets the utilization time of each payload object in the stack, to
            % determine the time indices for the simulation when an object is
            % grabbed and released by the specified manipulator (hand).
            %
            % Args:
            %   obj: The simulation configuration object.
            %   obj_idx   (integer): Index position of the payload object in the
            %                        stack.
            %   start_idx (integer): Time index position of the integration output
            %                        matrix :math:`\mathcal{X}`, where the robot
            %                        has grabbed the object.
            %   end_idx   (integer): Time index position of the integration output
            %                        matrix :math:`\mathcal{X}`, where the robot
            %                        has released the object.
            len = size(obj.pl_stack,1);
            if ( (obj_idx < 1) || (obj_idx > len) )
                error('wbmSimConfig::setPayloadUtilTime: %s', WBM.wbmErrorMsg.VAL_OUT_OF_BOUNDS);
            end
            if (~start_idx || ~end_idx)
                error('wbmSimConfig::setPayloadUtilTime: %s', WBM.wbmErrorMsg.VALUE_LTE_ZERO);
            end
            obj.pl_time_idx(obj_idx,1) = start_idx;
            obj.pl_time_idx(obj_idx,2) = end_idx;
        end

    end
end
