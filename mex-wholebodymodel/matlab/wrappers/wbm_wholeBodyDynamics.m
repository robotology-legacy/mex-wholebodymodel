function [M, c_qv, h_c] = wbm_wholeBodyDynamics(varargin)
    % WBM_WHOLEBODYDYNAMICS obtains the main components for the whole-body dynamics of
    % a robot system, in particular it computes w.r.t. to the state parameter triplet
    % [q_j, dq_j, v_b],
    %
    %   * the generalized mass matrix,
    %   * the generalized bias forces  and
    %   * the centroidal momentum
    %
    % of the given floating-base robot.
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
    %   M    -- (n_dof+6 x n_dof+6) generalized mass matrix of the robot.
    %   c_qv -- (n_dof+6 x 1) generalized bias force vector.
    %   h_c  -- (6 x 1) centroidal moment vector of the robot.

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
            M    = mexWholeBodyModel('mass-matrix');
            c_qv = mexWholeBodyModel('generalized-forces');
            h_c  = mexWholeBodyModel('centroidal-momentum');
        case 5
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            M    = mexWholeBodyModel('mass-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3});
            c_qv = mexWholeBodyModel('generalized-forces', wf_R_b_arr,varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5});
            h_c  = mexWholeBodyModel('centroidal-momentum', wf_R_b_arr, varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5});
        otherwise
            wbm_narginError('wbm_wholeBodyDynamics');
    end

end
