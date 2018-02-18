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

classdef (Abstract) wbmGObj
    % :class:`!wbmGObj` is an *abstract data type* to define specific graphics
    % objects for the robot simulation.
    %
    % Attributes:
    %   description  (char, vector): Short description string (annotation) about
    %                                the graphics object.
    %   line_width (double, scalar): Line width of the graphics object, specified
    %                                as a positive value in points.
    % Methods:
    %   getGObj(obj): *abstract* -- Creates the graphics object and returns a
    %                               handle to it.
    %
    %                               **Note:** The graphics object can consist of
    %                               several sub-graphics objects.
    %   updGObj(obj, hgo): *abstract* -- Updates the data parameters of the
    %                                    graphics object.
    properties(Abstract)
        description@char
        line_width@double scalar
    end

    methods(Abstract)
        hgo = getGObj(obj)
        hgo = updGObj(obj, hgo)
    end
end