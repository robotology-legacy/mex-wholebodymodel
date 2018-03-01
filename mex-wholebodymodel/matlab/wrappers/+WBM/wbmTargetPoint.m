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

classdef wbmTargetPoint < WBM.wbmGObj
    % :class:`!wbmTargetPoint` is a *data type* (class) that defines a target
    % point in the simulation environment which should be reached by a link
    % of the simulated robot.
    %
    % Attributes:
    %   pos        (double, vector): (3 x 1) Cartesian position of the target point
    %                                in the world frame (wf). Default position:
    %                                :math:`[0, 0, 0]^T`.
    %   description  (char, vector): Short description string (annotation) about
    %                                the target point (default: *empty*).
    %   line_width (double, scalar): Line width of the marker, specified as a
    %                                positive value in points (default width: 0.5).
    %   marker       (char, vector): Marker symbol for the data point. The symbols
    %                                for the marker are the same as specified in
    %                                Matlab (default symbol: *'o'*).
    %   mkr_size   (double, scalar): Marker size, specified as a positive value
    %                                in points (default size: 8).
    %   mkr_color (double/char, vector): Marker outline color, specified by a
    %                                    RGB-triplet or a color name (default
    %                                    color: *'yellow'*).
    % Note:
    %   The specified target points will be shown in the robot simulation, if in
    %   the given *simulation configuration structure*, an array of initialized
    %   *target point objects* will be set with a size of at least 1.
    properties
        pos@double        = zeros(3,1);
        description@char  = '';

        line_width@double scalar = 0.5;
        marker@char       = 'o'; % circle
        mkr_size@double   scalar = 8;
        mkr_color         = 'yellow';
    end

    methods
        function obj = wbmTargetPoint(p)
            % Constructor.
            %
            % Arguments:
            %   p (double, vector): (3 x 1) Cartesian position for the target
            %                       point in world frame (wf).
            % Returns:
            %   obj: An instance of the target point data type.
            switch nargin
                case 0
                    return
                case 1
                    WBM.utilities.chkfun.checkCVecDim(p, 3, 'wbmTargetPoint::wbmTargetPoint');
                    obj.pos = p;
                otherwise
                    error('wbmTargetPoint::wbmTargetPoint: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
        end

        function hgo = getGObj(obj)
            % Plots a 3D data point at the specified target position and returns
            % the graphics object handle of the created data point.
            %
            % Returns:
            %   hgo: Handle of the created graphics object (chart line object).

            % mark the target point to be reached:
            p = obj.pos;
            hold on;
            hgo = plot3(p(1,1), p(2,1), p(3,1), 'LineStyle', 'none', 'LineWidth', obj.line_width, ...
                        'Marker', obj.marker, 'MarkerSize', obj.mkr_size, 'MarkerEdgeColor', obj.mkr_color);
        end

        function hgo = updGObj(obj, hgo)
            % Updates the position of the 3D data point.
            %
            % Arguments:
            %   hgo: Graphics object handle of the 3D data point.
            %
            % Returns:
            %   hgo: Graphics object handle (chart line object) with
            %        the new position.

            % update the target point:
            hgo.XData = obj.pos(1,1);
            hgo.YData = obj.pos(2,1);
            hgo.ZData = obj.pos(3,1);
        end

        function obj = set.pos(obj, p)
            % Sets a new position for the target point.
            %
            % Arguments:
            %   p (double, vector): New (3 x 1) Cartesian position for the
            %                       target point in world frame (wf).
            % Returns:
            %   obj: Target point object with the new position.
            WBM.utilities.chkfun.checkCVecDim(p, 3, 'wbmTargetPoint::set.pos');
            obj.pos = p;
        end

    end
end
