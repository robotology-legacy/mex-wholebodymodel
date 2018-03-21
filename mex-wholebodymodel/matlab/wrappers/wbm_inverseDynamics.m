function tau_j = wbm_inverseDynamics(varargin)
    % WBM_CENTROIDALMOMENTUM computes the inverse dynamics of a rigid-body system. It depends on
    % the state parameter triplet [q_j, dq_j, v_b], the joint angle accelerations ddq_j and the
    % generalized base accelerations dv_b.
    %
    % Input Arguments:
    %
    %   Normal mode:
    %       wf_R_b -- (3 x 3) rotation matrix from the base frame 'b' to the world frame 'wf'.
    %       wf_p_b -- (3 x 1) position vector from the base frame 'b' to the world frame 'wf'.
    %       q_j    -- (n_dof x 1) joint angle vector in [rad].
    %       dq_j   -- (n_dof x 1) joint angle velocity vector in [rad/s].
    %       v_b    -- (6 x 1) generalized base velocity vector.
    %       ddq_j  -- (n_dof x 1) joint angle acceleration vector in [rad/s^2].
    %       dv_b   -- (6 x 1) generalized base acceleration vector.
    %
    %   Optimized mode:  No arguments.
    %
    % Output Arguments:
    %   tau_j -- (n_dof+6 x 1) generalized force vector at the joints and the base of the robot.

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
        case 2
            tau_j = mexWholeBodyModel('inverse-dynamics', varargin{1,1}, varargin{1,2});
        case 7
            tau_j = mexWholeBodyModel('inverse-dynamics', reshape(varargin{1,1}, 9, 1), varargin{1,2}, varargin{1,3}, ...
                                                          varargin{1,4}, varargin{1,5}, varargin{1,6}, varargin{1,7});
        otherwise
            wbm_narginError('wbm_inverseDynamics');
    end
end
