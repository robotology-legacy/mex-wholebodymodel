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
% GNU Lesser General Public License for more details.
%
% A copy of the GNU Lesser General Public License can be found along
% with the WBML. If not, see <http://www.gnu.org/licenses/>.

classdef wbmRobotDrawProp < handle
    % :class:`!wbmRobotDrawProp` is a *data type* (class) to control the draw
    % properties for the body of the simulated robot.
    %
    % Attributes:
    %   joints (struct): Structure to control the appearance of the joint nodes,
    %                    specified by following fields:
    %                      - ``line_width`` (double, scalar): Line width of the marker, specified as
    %                                                         a positive value in points.
    %                      - ``marker``       (char, vector): Marker symbol for the joint nodes. The
    %                                                         symbols for the marker are the same as
    %                                                         specified in Matlab.
    %                      - ``marker_sz``  (double, scalar): Marker size, specified as a positive
    %                                                         value in points.
    %                      - ``color`` (double/char, vector): Color of the joint nodes, specified by
    %                                                         a RGB-triplet or a color name.
    %   links  (struct): Structure to control the appearance of the links (edges)
    %                    of the robot's skeleton:
    %                      - ``line_width`` (double, scalar): Line width of the links, specified as a
    %                                                         positive value in points.
    %                      - ``color`` (double/char, vector): Link color, specified by a RGB-triplet or
    %                                                         a color name.
    %   com    (struct): Structure to control the appearance of the center of
    %                    mass node:
    %                      - ``marker``       (char, vector): Marker symbol for the node of the center of
    %                                                         mass. The symbols for the marker are the
    %                                                         same as specified in Matlab.
    %                      - ``marker_sz``  (double, scalar): Marker size, specified as a positive value
    %                                                         in points.
    %                      - ``color`` (double/char, vector): Node color of the center of mass, specified
    %                                                         by a RGB-triplet or a color name.
    %   shape  (struct): Structure to control the appearance of the body shape
    %                    of the robot:
    %                      - ``line_width``      (double, scalar): Line width of the polygons (edges) for
    %                                                              the body parts of the robot, specified
    %                                                              as a positive value in points.
    %                      - ``edge_color`` (double/char, vector): Edge color for the body parts of the
    %                                                              robot, specified by a RGB-triplet or
    %                                                              a color name.
    %                      - ``face_color`` (double/char, vector): Face color for the body parts of the
    %                                                              robot, specified by a RGB-triplet or
    %                                                              a color name.
    %                      - ``face_alpha``      (double, scalar): Face transparency of the body parts,
    %                                                              specified by a scalar in range
    %                                                              :math:`[0,1]`.
    properties
        joints = struct( 'line_width', 0, ...
                         'marker',     '', ...
                         'marker_sz',  0, ...
                         'color',      [] );

        links  = struct( 'line_width', 0, ...
                         'color',      [] );

        com    = struct( 'marker',     '', ...
                         'marker_sz',  0, ...
                         'color',      [] );

        shape  = struct( 'line_width', 0, ...
                         'edge_color', [], ...
                         'face_color', [], ...
                         'face_alpha', 0 );
    end
end
