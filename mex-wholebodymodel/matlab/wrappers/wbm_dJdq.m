function djdq_lnk = wbm_dJdq(varargin)
    % WBM_DJDQ computes the product (bias acceleration) of the time derivative of the Jacobian and
    % the joint velocities w.r.t. the current state and the specified link (frame) of the robot.
    %
    % Note:
    %   Useful for Operational Space Control and for calculating the external contact constraints.
    %
    % Input Arguments:
    %
    %   Normal mode:
    %       wf_R_b         -- (3 x 3) rotation matrix from the base frame 'b' to the world frame 'wf'.
    %       wf_p_b         -- (3 x 1) position vector from the base frame 'b' to the world frame 'wf'.
    %       q_j            -- (n_dof x 1) joint angle vector in [rad].
    %       dq_j           -- (n_dof x 1) joint angle velocity vector in [rad/s].
    %       v_b            -- (6 x 1) generalized base velocity vector.
    %       urdf_link_name -- String matching URDF name of the link.
    %
    %   Optimized mode:
    %       urdf_link_name -- String matching URDF name of the link.
    %
    % Output Arguments:
    %   djdq_lnk -- (6 x 1) bias acceleration vector relative from the link frame 'lnk' to the
    %               world frame 'wf' in operational space.

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
        case 1
            djdq_lnk = mexWholeBodyModel('dJdq', varargin{1,1});
        case 6
            djdq_lnk = mexWholeBodyModel('dJdq', reshape(varargin{1,1}, 9, 1), varargin{1,2}, varargin{1,3}, ...
                                                 varargin{1,4}, varargin{1,5}, varargin{1,6});
        otherwise
            wbm_narginError('wbm_dJdq');
    end
end
