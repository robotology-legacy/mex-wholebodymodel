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

classdef wbmToolLink
    % :class:`!wbmToolLink` is a *data type* (class) to define a tool with its
    % tool-tip frame (tt) at a specified link of an end-effector (ee).
    %
    % Attributes:
    %   urdf_link_name (char, vector): URDF-name of the end-effector link frame
    %                                  (default name: ``'none'``).
    %   ee_vqT_tt    (double, vector): :math:`(7 \times 1)` VQ-transformation frame
    %                                  of the tool, relative from the tool-tip *tt*
    %                                  to the link frame of the end-effector *ee*.
    %                                  Default vector: :math:`[0, 0, 0, 1, 0, 0, 0]^T`.
    %   pl_idx        (uint8, scalar): Index number of a specified *payload link
    %                                  object* for the case that a grabbed
    %                                  payload object is also a tool.
    %                                  If the index is set to 0, then the tool
    %                                  is not linked with a given payload object
    %                                  (default value: 0).
    % See Also:
    %   :class:`~WBM.wbmPayloadLink`.
     properties
        urdf_link_name@char     = 'none';
        ee_vqT_tt@double vector = [0; 0; 0; 1; 0; 0; 0];

        pl_idx@uint8     scalar = 0;
    end
end
