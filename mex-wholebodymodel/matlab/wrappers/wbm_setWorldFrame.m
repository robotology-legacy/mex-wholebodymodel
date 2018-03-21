function wbm_setWorldFrame(varargin)
    % WBM_SETWORLDFRAME sets the world frame (wf) at a given position and orientation, starting
    % from a specified fixed link frame (reference frame). The specified fixed link (reference
    % link) can also be a contact constraint link.
    %
    % Note:
    %   The internally specified base-to-world transformation matrix wf_H_b of the mexWholeBodyModel
    %   supports (w.r.t. a given joint configuration q_j) the optimized computations of all kinematic
    %   and dynamic functions of the given C++ component classes. Moreover, it supports also the
    %   computation of the base VQ-transformation vqT_b of the current model's state.
    %
    % Input Arguments:
    %   wf_R_b -- (3 x 3) rotation matrix from the base frame 'b' to the world frame 'wf'.
    %   wf_p_b -- (3 x 1) position vector from the base frame 'b' to the world frame 'wf'.
    %   g_wf   -- (3 x 1) gravity vector in the world frame 'wf'.
    %
    % See also:
    %   WBM_GETWORLDFRAMEFROMFIXLNK

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
            mexWholeBodyModel('set-world-frame', reshape(varargin{1,1}, 9, 1), varargin{1,2}, varargin{1,3});
        otherwise
            wbm_narginError('wbm_setWorldFrame');
    end
end
