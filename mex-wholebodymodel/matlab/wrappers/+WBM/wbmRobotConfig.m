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

classdef wbmRobotConfig < handle
    % :class:`!wbmRobotConfig` is a *data type* (class) that specifies the
    % configuration parameters of a given floating-base robot.
    %
    % Attributes:
    %   nCstrs             (uint8, scalar): Number of contact constraints of those links,
    %                                       where the robot model is in contact with the
    %                                       environment (ground or object).
    %   ccstr_link_names (cellstr, vector): (1 x *nCstrs*) string array of *contact
    %                                       constraint link names* to specify which
    %                                       link of the robot is in contact with
    %                                       the ground or an object.
    %   nPlds              (uint8, scalar): Number of payloads that are assigned
    %                                       to specific links of the robot (default: 0).
    %   payload_links (:class:`~WBM.wbmPayloadLink`, vector): (1 x *nPlds*) array of *payload link objects*,
    %                                                         where each payload is assigned to a specific
    %                                                         link of the robot (default: *empty*).
    %
    %                                                         **Note:** The index 1 and 2 of the array are
    %                                                         always reserved for the *left* and the *right
    %                                                         hand* of a given humanoid robot. The default
    %                                                         index, that will be used by the :class:`WBM`
    %                                                         class, is always 1. All further indices of the
    %                                                         array can be used for other links, on which
    %                                                         additionally special payloads are mounted (e.g.
    %                                                         a battery pack or a knapsack at the torso,
    %                                                         special tools, etc.).
    %   nTools             (uint8, scalar): Number of tools that are assigned to
    %                                       specific links of the robot. The maximum
    %                                       number of tools that a humanoid robot can
    %                                       use simultaneously, is by default 2 (see
    %                                       :attr:`WBM.MAX_NUM_TOOLS`). The default
    %                                       number is 0.
    %   tool_links (:class:`~WBM.wbmToolLink`, vector): (*n* x 1) array of *tool link objects*, where
    %                                                   each tool is assigned to a specific link of
    %                                                   the robot. The length *n* of the array can
    %                                                   not exceed the limit :attr:`WBM.MAX_NUM_TOOLS`.
    %                                                   By default, the initial array is *empty*.
    %
    %                                                   **Note:** The *default tool link* for the robot
    %                                                   is always the first element of the object list.
    %   init_state_params (:class:`~WBM.wbmStateParams`): Data object to define the initial state
    %                                                     parameters of a given floating-base robot.
    %   stvLen            (uint16, scalar): Length of the state parameter vector, i.e. the
    %                                       length of all state parameters concatenated in
    %                                       one vector. The length can be either
    %                                       :math:`2 * n_{dof} + 13` (default),
    %                                       :math:`2 * n_{dof} + 6` (without
    %                                       :attr:`~WBM.wbmStateParams.x_b` and
    %                                       :attr:`~WBM.wbmStateParams.qt_b`) or 0 if
    %                                       :attr:`init_state_params` is empty [#f1]_.
    %   body       (:class:`~WBM.wbmBody`): Data object to define and configure the
    %                                       body components of the given robot.
    properties
        nCstrs@uint8          scalar = 0;
        ccstr_link_names@cell            vector
        nPlds@uint8           scalar = 0;
        payload_links@WBM.wbmPayloadLink vector = WBM.wbmPayloadLink.empty;
        nTools@uint8          scalar = 0;
        tool_links@WBM.wbmToolLink       vector = WBM.wbmToolLink.empty;
        init_state_params@WBM.wbmStateParams    = WBM.wbmStateParams;
        stvLen@uint16         scalar = 0;
        body@WBM.wbmBody
    end
end
