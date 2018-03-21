function h_c = wbm_centroidalMomentum(varargin)
    % WBM_CENTROIDALMOMENTUM computes the centroidal momentum h_c of a robot system.
    % It is a function of the state variables q_j, dq_j and the generalized base
    % velocity v_b.
    %
    % Note:
    %   The centroidal momentum of a humanoid robot is the sum of all link moments
    %   projected to the center of mass (CoM) of the robot.
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
    %   h_c -- (6 x 1) centroidal moment vector of the robot.

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
            h_c = mexWholeBodyModel('centroidal-momentum');
        case 5
            h_c = mexWholeBodyModel('centroidal-momentum', reshape(varargin{1,1}, 9, 1), varargin{1,2}, varargin{1,3}, ...
                                                           varargin{1,4}, varargin{1,5});
        otherwise
            wbm_narginError('wbm_centroidalMomentum');
    end
end
