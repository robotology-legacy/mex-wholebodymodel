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

classdef wbmStateParams
    % :class:`!wbmStateParams` is a *data type* (class) to represent the state
    % parameters of the current state of a given floating base robot.
    %
    % Attributes:
    %   x_b     (double, matrix): Cartesian position of the base in Euclidean
    %                             space :math:`\mathbb{R}^3`.
    %   qt_b    (double, matrix): Orientation of the base in quaternions (global
    %                             parametrization of :math:`\mathbf{SO}^3`).
    %   q_j     (double, matrix): Joint positions (angles) of dimension
    %                             :math:`\mathbb{R}^{n_{dof}}` in :math:`[\si{\radian}]`.
    %   dx_b    (double, matrix): Cartesian velocity of the base in Euclidean
    %                             space :math:`\mathbb{R}^3`.
    %   omega_b (double, matrix): Angular velocity describing the orientation of
    %                             the base in :math:`\mathbf{SO}^3`.
    %   dq_j    (double, matrix): Joint velocities of dimension :math:`\mathbb{R}^{n_{dof}}`
    %                             in :math:`[\si{\radian/s}]`.
    % Note:
    %   In dependency of the situation, the state variables can represent either
    %   a *single state* with only *column-vectors* or, a *time sequence of
    %   states* with only *matrices*, where each row-vector represents a state.
    properties
        % positions & orientation:
        x_b@double     matrix
        qt_b@double    matrix
        q_j@double     matrix
        % velocities:
        dx_b@double    matrix
        omega_b@double matrix
        dq_j@double    matrix
    end
end