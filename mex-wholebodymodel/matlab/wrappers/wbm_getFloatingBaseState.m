function [wf_R_b, wf_p_b, v_b] = wbm_getFloatingBaseState()
    % WBM_GETFLOATINGBASESTATE returns the current state, i.e. the position, orientation and the
    % velocity of the robot's floating base 'b'.
    %
    % Note:
    %   The values of the floating base state are derived from the currently stored state
    %   of the robot system.
    %
    % Output Arguments:
    %   wf_R_b -- (3 x 3) floating base rotation matrix from the base frame 'b' to the world frame 'wf'.
    %   wf_p_b -- (3 x 1) floating base position vector from the base frame 'b' to the world frame 'wf'.
    %   v_b    -- (6 x 1) generalized base velocity vector.
    %
    % See also:
    %   WBM_GETSTATE

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

    [wf_R_b, wf_p_b, v_b] = mexWholeBodyModel('get-base-state');
end
