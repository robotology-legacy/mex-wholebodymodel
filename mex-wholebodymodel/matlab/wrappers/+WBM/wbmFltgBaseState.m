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

classdef wbmFltgBaseState < handle
    % :class:`!wbmFltgBaseState` is a *data type* (class) to represent the
    % actual state parameters of the robot's floating base *b* related to
    % the world frame *wf*.
    %
    % Attributes:
    %   wf_R_b (double, matrix): :math:`(3 \times 3)` orientation matrix of the
    %                            base (in axis-angle representation).
    %   wf_p_b (double, vector): :math:`(3 \times 1)` Cartesian position vector
    %                            of the base.
    %   v_b    (double, vector): :math:`(6 \times 1)` generalized base velocity
    %                            vector (Cartesian and rotational velocity of
    %                            the base).
    % Note:
    %   The data type is also useful for interfaces to get quickly the current
    %   state of the floating base.
    %
    % See Also:
    %   :class:`Interfaces.iCubWBM` and :meth:`WBMBase.getFloatingBaseState`.
    properties
        wf_R_b@double matrix
        wf_p_b@double vector
        v_b@double    vector
    end
end
