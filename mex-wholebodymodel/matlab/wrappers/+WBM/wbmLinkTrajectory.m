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

classdef wbmLinkTrajectory < WBM.wbmGObj
    % :class:`!wbmPayloadLink` is a *data type* (class) that specifies the
    % trajectory curve of a specific link of the simulated robot.
    %
    % Attributes:
    %   urdf_link_name    (char, vector): URDF-name of the link frame to be
    %                                     tracked of the robot model (default
    %                                     name: *'none'*).
    %   jnt_annot_pos     (cell, vector): Cell array of the form :math:`\{\textrm{lnk_group_name}, idx\}`,
    %                                     to specify the *index position* of the
    %                                     annotation string for the parent joint
    %                                     of the given link (default: *empty*).
    %
    %                                     **Note:** In :mod:`~WBM.utilities`, the
    %                                     joint annotation function :func:`~utilities.getJointAnnotationICub`,
    %                                     is only defined for the iCub humanoid
    %                                     robot. For other YARP-based robots, a
    %                                     custom annotation function must be used.
    %   description       (char, vector): Short description about the trajectory
    %                                     curve of the link (default: *empty*).
    %   line_style        (char, vector): Line style of the link trajectory curve.
    %                                     The options for the line style are the
    %                                     same as specified in Matlab (default
    %                                     line style: *':'*)
    %   line_width      (double, scalar): Line width of the trajectory curve, specified as
    %                                     a positive value in points (default width: 0.5).
    %   line_color (double/char, vector): Color of the trajectory curve, specified by a
    %                                     RGB-triplet or a color name (default color:
    %                                     *'green'*).
    %   ept_marker        (char, vector): Marker symbol for the end point of the
    %                                     trajectory curve. The symbols for the
    %                                     marker are the same as specified in
    %                                     Matlab (default symbol: *'o'*).
    %   ept_marker_sz   (double, scalar): Marker size of the trajectory end point,
    %                                     specified as a positive value in points
    %                                     (default size: 8).
    %   ept_line_wid    (double, scalar): Line width of the trajectory end point,
    %                                     specified as a positive value in points
    %                                     (default width: 0.5).
    %   ept_color  (double/char, vector): Color of the trajectory end point, specified
    %                                     by a RGB-triplet or a color name (default
    %                                     color: *'green'*).
    %   lnk_pos         (double, matrix): (*n* x 3) data matrix with Cartesian positions
    %                                     to specify in the simulation the trajectory
    %                                     vertices of the given link frame of the robot.
    %                                     The size *n* of the matrix represents the
    %                                     *number of time steps* of the simulation.
    properties
        urdf_link_name@char   = 'none';
        jnt_annot_pos@cell    vector = {}; % {lnk_group_name, idx}
        description@char      = '';

        line_style@char       = ':'; % dotted line
        line_width@double     scalar = 0.5;
        line_color            = 'green';

        ept_marker@char       = 'o'; % circle
        ept_marker_sz@double  scalar = 8;
        ept_line_wid@double   scalar = 0.5;
        ept_color             = 'green';

        lnk_pos@double matrix = [];
    end

    methods
        function obj = wbmLinkTrajectory(lnk_name, lnk_pos)
            % Constructor.
            %
            % Arguments:
            %   lnk_name  (char, vector): String matching URDF-name of the link
            %                             frame to be tracked of the robot model.
            %   lnk_pos (double, matrix): (*n* x 3) data matrix with Cartesian positions
            %                             in each row, to specify the coordinates of the
            %                             trajectory vertices of the given link frame of
            %                             the robot. The size *n* of the matrix represents
            %                             the *number of time steps* of the simulation.
            % Returns:
            %   obj: An instance of the :class:`!wbmLinkTrajectory` data type.
            switch nargin
                case 0
                    return
                case 2
                    if (size(lnk_pos,2) ~= 3)
                        error('wbmLinkTrajectory::wbmLinkTrajectory: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
                    end
                    obj.lnk_pos = lnk_pos;
            end
            % if nargin >= 1:
            WBM.utilities.chkfun.checkLinkName(lnk_name, 'wbmLinkTrajectory::wbmLinkTrajectory');
            obj.urdf_link_name = lnk_name;
        end

        function hgo = getGObj(obj)
            % Creates and draws the trajectory curve of the specified link of
            % the simulated robot.
            %
            % Returns:
            %   hgo (gobjects, vector): Column-array of graphics object handles
            %                           of the created trajectory curve.
            dpts = obj.lnk_pos;
            hgo  = gobjects(2,1);

            if isempty(dpts)
                warning('wbmLinkTrajectory::getGObj: The trajectory data points are not defined.');
                return
            end

            hgo(1,1) = plot3(dpts(:,1), dpts(:,2), dpts(:,3), 'LineStyle', obj.line_style, ...
                             'LineWidth', obj.line_width, 'Color', obj.line_color);
            hold on;
            % mark the endpoint ...
            hgo(2,1) = plot3(dpts(end,1), dpts(end,2), dpts(end,3), 'LineStyle', 'none', 'LineWidth', obj.ept_line_wid, ...
                             'Marker', obj.ept_marker, 'MarkerSize', obj.ept_marker_sz, 'MarkerEdgeColor', obj.ept_color);
        end

        function hgo = updGObj(obj, hgo)
            % Updates the the vertex coordinates of the link trajectory curve.
            %
            % Arguments:
            %   hgo (gobjects, vector): Column-array of graphics object handles
            %                           of the link trajectory curve.
            % Returns:
            %   hgo (gobjects, vector): Column-array with the updated vertex
            %                           coordinates of the graphics object handles.
            WBM.utilities.chkfun.checkCVecDim(hgo, 2, 'wbmLinkTrajectory::getGObj');
            dpts = obj.lnk_pos;
            if isempty(dpts)
                error('wbmLinkTrajectory::updGObj: %s', WBM.wbmErrorMsg.EMPTY_DATA_PTS);
            end
            htl = hgo(1,1);
            hep = hgo(2,1);

            % update trajectory line:
            htl.XData = dpts(:,1);
            htl.YData = dpts(:,2);
            htl.ZData = dpts(:,3);
            % update endpoint:
            hep.XData = dpts(end,1);
            hep.YData = dpts(end,2);
            hep.ZData = dpts(end,3);

            hgo(1,1) = htl;
            hgo(2,1) = hep;
        end

        function obj = set.urdf_link_name(obj, lnk_name)
            % Sets the URDF-name of the link frame to be tracked of the
            % robot model.
            %
            % Arguments:
            %   lnk_name (char, vector): String matching URDF-name of the link
            %                            frame.
            % Returns:
            %   obj: Link trajectory object with the new link name.
            WBM.utilities.chkfun.checkLinkName(lnk_name, 'wbmLinkTrajectory::set.urdf_link_name');
            obj.urdf_link_name = lnk_name;
        end

        function obj = set.jnt_annot_pos(obj, annot_pos)
            % Sets the index position for the joint annotation string of the
            % specified link frame.
            %
            % Arguments:
            %   annot_pos (cell, vector): Array with the specified *annotation
            %                             index position* of the form
            %                             :math:`\{\textrm{lnk_group_name}, idx\}`.
            % Returns:
            %   obj: Link trajectory object with the new index position.
            WBM.utilities.chkfun.checkRVecDim(annot_pos, 2, 'wbmLinkTrajectory::set.jnt_annot_pos');
            if ( ~iscell(annot_pos) || ~ischar(annot_pos{1,1}) || ~isreal(annot_pos{1,2}) )
                error('wbmLinkTrajectory::set.jnt_annot_pos: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            annot_pos{1,2} = uint8(annot_pos{1,2});
            obj.jnt_annot_pos = annot_pos; % {lnk_group_name, idx}
        end

    end
end
