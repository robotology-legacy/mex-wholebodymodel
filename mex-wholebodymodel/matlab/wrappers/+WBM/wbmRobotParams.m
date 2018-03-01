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

classdef wbmRobotParams < handle
    % :class:`!wbmRobotParams` is a *data type* (class) to export the model and
    % configuration parameters of a given floating base robot between interfaces.
    %
    % Attributes:
    %   model   (:class:`~WBM.wbmRobotModel`): Model object with the model parameters
    %                                          of the given floating base robot.
    %   config (:class:`~WBM.wbmRobotConfig`): Configuration object with the configuration
    %                                          settings of the given floating base robot.
    %   wf2fixlnk (logical, scalar): Boolean flag to indicate if the world frame
    %                                (wf) is set to a fixed reference link frame.
    properties
        model@WBM.wbmRobotModel
        config@WBM.wbmRobotConfig
        wf2fixlnk@logical scalar
    end

    methods(Sealed)
        function newObj = copy(obj)
            % Copy function to create deep object copies.
            %
            % Returns:
            %   newObj: A full copy of the robot parameters.
            newObj = WBM.utilities.copyObj(obj);
        end

    end
end
