function c_qv = wbm_coriolisBiasForces(varargin)
    % WBM_CORIOLISBIASFORCES computes the bias Coriolis and centrifugal forces C(q_j,dq_j) in the
    % dynamics of rigid-body systems.
    %
    % Note:
    %   This function is derived from the generalized bias force function and depends
    %   on the same state parameter triplet [q_j, dq_j, v_b] as the parent-function.
    %
    % Input Arguments:
    %
    %   Normal mode:
    %       wf_R_b -- (3 x 3) rotation matrix from the base frame 'b' to the world frame 'wf'.
    %       wf_p_b -- (3 x 1) position vector from the base frame 'b' to the world frame 'wf'.
    %       q_j    -- (n_dof x 1) joint angle vector in [rad].
    %       dq_j   -- (n_dof x 1) joint angle velocity vector in [rad/s].
    %       v_b    -- (6 x 1) generalized base velocity vector.
    %
    %   Optimized mode:  No arguments.
    %
    % Output Arguments:
    %   c_qv -- (n_dof+6 x 1) bias Coriolis and centrifugal force vector of the robot.
    %
    % See also:
    %   WBM_GENERALIZEDBIASFORCES

    % Copyright (C) 2017 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
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
            c_qv = mexWholeBodyModel('coriolis-forces');
        case 5
            c_qv = mexWholeBodyModel('coriolis-forces', reshape(varargin{1,1}, 9, 1), varargin{1,2}, varargin{1,3}, ...
                                                        varargin{1,4}, varargin{1,5});
        otherwise
            wbm_narginError('wbm_coriolisBiasForces');
    end
end
