function [vqT_b, q_j, v_b, dq_j] = wbm_getState()
    % WBM_GETSTATE obtains the currently stored state of the robot system , in particular
    %
    %   * the base VQ-transformation,
    %   * the joint angles and velocities  and
    %   * the generalized base velocity.
    %
    % Output Arguments:
    %   vqT_b -- (7 x 1) VQ-transformation [*] from the base frame 'b' to the world frame 'wf'.
    %   q_j   -- (n_dof x 1) joint angle vector in [rad].
    %   v_b   -- (6 x 1) generalized base velocity vector.
    %   dq_j  -- (n_dof x 1) joint angle velocity vector in [rad/s].
    %
    %   [*]: The first 3 elements of the vector are representing the position and
    %        the last 4 elements are defining the orientation in quaternions.
    %
    % See also:
    %   WBM_UPDATESTATE

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

    [vqT_b, q_j, v_b, dq_j] = mexWholeBodyModel('get-state');
end
