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

classdef wbmHumanoidConfig < WBM.wbmRobotConfig
    % :class:`!wbmHumanoidConfig` is an extended *data type* (class) of the class
    % :class:`~WBM.wbmRobotConfig` and represents the general configuration
    % structure of a *humanoid robot*.
    %
    % Attributes:
    %   jpos_head       (double, vector): Initial joint positions for the head.
    %   jpos_torso      (double, vector): Initial joint positions for the torso.
    %   jpos_left_arm   (double, vector): Initial joint positions for the left arm.
    %   jpos_left_hand  (double, vector): Initial joint positions for the left hand.
    %   jpos_left_leg   (double, vector): Initial joint positions for the left leg.
    %   jpos_left_foot  (double, vector): Initial joint positions for the left foot.
    %   jpos_right_arm  (double, vector): Initial joint positions for the right arm.
    %   jpos_right_hand (double, vector): Initial joint positions for the right hand.
    %   jpos_right_leg  (double, vector): Initial joint positions for the right leg.
    %   jpos_right_foot (double, vector): Initial joint positions for the right foot.
    %
    % Note:
    %   The given *joint positions vectors* for the initial body pose of the robot must
    %   be specified in the data type as *column-vectors* and the corresponding *angle
    %   positions* of the joints are defined in degrees (:math:`[\textrm{deg}]`).
    %
    % See Also:
    %   :class:`~WBM.wbmRobotConfig`.
    properties
        jpos_head@double       vector
        jpos_torso@double      vector
        jpos_left_arm@double   vector
        jpos_left_hand@double  vector
        jpos_left_leg@double   vector
        jpos_left_foot@double  vector
        jpos_right_arm@double  vector
        jpos_right_hand@double vector
        jpos_right_leg@double  vector
        jpos_right_foot@double vector
    end
end
