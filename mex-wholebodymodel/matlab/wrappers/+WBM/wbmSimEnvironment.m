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

classdef wbmSimEnvironment < handle
    % :class:`!wbmSimEnvironment` is a *data type* (class) that defines the
    % environment for the robot simulation. It defines the background, the
    % floor and the geometric volume bodies for the simulation environment.
    %
    % Attributes:
    %   bkgrd_color_opt (char, vector): Sets the color scheme for the axis background,
    %                                   axis lines and labels, and the figure background
    %                                   to one of the color options: *'white'*, *'black'*,
    %                                   or *'none'*.
    %   grnd_shape    (double, matrix): Defines the shape of the ground-floor plane,
    %                                   as a polygon. The vertical X, Y and Z triplets,
    %                                   combined into a matrix, specify the vertices of
    %                                   the polygon.
    %   grnd_color      (double/char, vector): Color of the ground, specified by a
    %                                          RGB-triplet or a color name.
    %   grnd_edge_color (double/char, vector): Edge color of the ground, specified
    %                                          by a RGB-triplet or a color name.
    %   orig_pt_color   (double/char, vector): Color of the origin point on the
    %                                          floor (RGB-triplet or color name).
    %   orig_pt_size         (double, scalar): Size of the origin point, specified
    %                                          as a positive value in points.
    %   vb_objects (:class:`~WBM.vbObject`, vector): Column-vector of *volume body objects*
    %                                                that representing an environment
    %                                                scenario for the simulation.
    properties
        bkgrd_color_opt@char
        grnd_shape@double       matrix
        grnd_color
        grnd_edge_color
        orig_pt_color
        orig_pt_size@double     scalar

        vb_objects@WBM.vbObject vector = WBM.vbObject.empty;
    end
end
