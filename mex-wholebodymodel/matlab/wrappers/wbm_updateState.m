function wbm_updateState(varargin)
    % WBM_UPDATESTATE updates the state of the robot model, i.e. the joint angles,
    % joint velocities and the generalized base velocity of the floating base.
    %
    % Input Arguments:
    %   q_j  -- (n_dof x 1) joint angle vector in [rad].
    %   dq_j -- (n_dof x 1) joint angle velocity vector in [rad/s].
    %   v_b  -- (6 x 1) generalized base velocity vector.
    %
    % See also:
    %    WBM_GETSTATE

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
        case 3
            mexWholeBodyModel('update-state', varargin{1,1}, varargin{1,2}, varargin{1,3});
        otherwise
            wbm_narginError('wbm_updateState');
    end
end
