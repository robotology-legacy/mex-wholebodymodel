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

classdef wbmPayloadLink
    % :class:`!wbmPayloadLink` is a *data type* (class) to define a payload that
    % is assigned to a specific link of a given floating base robot.
    %
    % Attributes:
    %   urdf_link_name (char, vector): URDF-name of the specified *reference link frame*
    %                                  or *contact link frame*. The link frame can be an
    %                                  end-effector frame of a hand or a special frame
    %                                  on which a payload object is mounted (knapsack,
    %                                  battery pack, etc.). If the object is undefined,
    %                                  the default link name is *'none'*.
    %   lnk_p_cm     (double, vector): (3 x 1) Cartesian position vector relative from
    %                                  the center of mass (cm) of the payload object
    %                                  to the link frame (lnk). Default position:
    %                                  :math:`[0, 0, 0]^T`.
    %   t_idx         (uint8, scalar): Index number of a specified *tool link object*
    %                                  to define that the grabbed payload object is
    %                                  also a tool. If the index value is 0, then the
    %                                  payload object is not linked with a tool
    %                                  (default value: 0).
    %   vb_idx        (uint8, scalar): Index number of a *volume body object* that will
    %                                  be linked to the given link frame.
    %                                  If the index value is 0, then the payload is not
    %                                  assigned to any given geometric volume bodies of
    %                                  the simulation environment (default: 0).
    %   vb_grabbed  (logical, scalar): Boolean flag to indicate that the payload object
    %                                  is *grabbed* by the given reference link.
    %   vb_released (logical, scalar): Boolean flag to indicate that the payload object
    %                                  is *released* by the given reference link.
    %   m_rb         (double, scalar): Mass of the rigid body (solid volume body) in
    %                                  :math:`[\si{kg}]`. The value is 0 by default,
    %                                  if the mass of the object is undefined.
    %   I_cm         (double, matrix): (3 x 3) *inertia tensor* of a rigid body or geometric
    %                                  object with the origin of the coordinate system at
    %                                  the center of mass (CoM) of the object body. The
    %                                  matrix is *empty*, if the inertia is undefined.
    properties
        urdf_link_name@char        = 'none';
        lnk_p_cm@double     vector = zeros(3,1);

        t_idx@uint8         scalar = 0;

        vb_idx@uint8        scalar = 0;
        vb_grabbed@logical  scalar = false;
        vb_released@logical scalar = false;

        m_rb@double         scalar = 0;
        I_cm@double         matrix = [];
    end
end
