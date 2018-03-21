function M = wbm_massMatrix(varargin)
    % WBM_MASSMATRIX computes the generalized mass matrix of the floating-base robot w.r.t. the
    % given joint configuration q_j.
    %
    % Input Arguments:
    %
    %   Normal mode:
    %       wf_R_b -- (3 x 3) rotation matrix from the base frame 'b' to the world frame 'wf'.
    %       wf_p_b -- (3 x 1) position vector from the base frame 'b' to the world frame 'wf'.
    %       q_j    -- (n_dof x 1) joint angle vector in [rad].
    %
    %   Optimized mode:  No arguments.
    %
    % Output Arguments:
    %   M -- (n_dof+6 x n_dof+6) generalized mass matrix of the robot.

    % Copyright (C) 2014-2017 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    %
    % This function is part of the Whole-Body Model Library for Matlab (WBML).
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
    switch nargin
        case 0
            M = mexWholeBodyModel('mass-matrix');
        case 3
            M = mexWholeBodyModel('mass-matrix', reshape(varargin{1,1}, 9, 1), varargin{1,2}, varargin{1,3});
        otherwise
            wbm_narginError('wbm_massMatrix');
    end
end
