function [wf_p_b, wf_R_b] = wbm_getWorldFrameFromFixLnk(varargin)
    % WBM_GETWORLDFRAMEFROMFIXLNK computes the position and the orientation of the floating base
    % w.r.t. a world frame (wf) that is intentionally set at a specified fixed link frame (base
    % reference frame).
    %
    % The base reference frame, also called floating-base frame, is attached to the fixed reference
    % link of the robot. Since the most humanoid robots and other legged robots are not physically
    % attached to the world, the floating-base framework provides a more general representation for
    % the robot control. The returned position and orientation of the floating base is obtained from
    % the forward kinematics w.r.t. the specified fixed link frame.
    %
    % Note:
    %   The default fixed link (floating-base link or base reference link) may be different for each
    %   YARP-based robot and the selection of the floating-base link depends also from the given
    %   situation. For example the default fixed link of the iCub humanoid robot is "l_sole".
    %   Furthermore, the specified fixed link can also be a contact constraint link.
    %
    % Input Arguments:
    %
    %   Normal mode:
    %       urdf_fixed_link -- String matching URDF name of the fixed link.
    %       q_j             -- (n_dof x 1) joint angle vector in [rad].
    %
    %   Optimized mode:
    %       urdf_fixed_link -- String matching URDF name of the fixed link.
    %
    % Output Arguments:
    %       wf_p_b -- (3 x 1) position vector from the base frame 'b' to the world frame 'wf'.
    %       wf_R_b -- (3 x 3) rotation matrix (orientation) from the base frame 'b' to the
    %                 world frame 'wf'.
    %
    % See also:
    %   WBM_SETWORLDFRAME

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
            [wf_p_b, wf_R_b] = computeNewWorld2Base(varargin{1,1});
        case 2
            [wf_p_b, wf_R_b] = computeNewWorld2Base(varargin{1,1}, varargin{1,2});
        otherwise
            wbm_narginError('wbm_getWorldFrameFromFixLnk');
    end
end


function [nw_p_b, nw_R_b] = computeNewWorld2Base(urdf_fixed_link, q_j)
    [ow_vqT_b,~,~,~] = wbm_getState(); % vector-quaternion transformation (from base to old world)

    % get the homogeneous transformation matrix H (from base to old world):
    [ow_p_b, ow_R_b] = WBM.utilities.tfms.frame2posRotm(ow_vqT_b);
    ow_H_b = WBM.utilities.tfms.posRotm2tform(ow_p_b, ow_R_b);

    % get the VQ-transformation (from reference link to old world):
    if (nargin == 1)
        ow_vqT_rlnk = wbm_forwardKinematics(urdf_fixed_link);
    else
        ow_vqT_rlnk = wbm_forwardKinematics(ow_R_b, ow_p_b, q_j, urdf_fixed_link);
    end

    % compute the new homogeneous transformation matrix H (from base to new world):
    ow_H_rlnk = WBM.utilities.tfms.frame2tform(ow_vqT_rlnk);
    nw_H_b = ow_H_rlnk \ ow_H_b; % = ow_H_rlnk^(-1) * ow_H_b

    % get the new position and rotation (from base to new world):
    [nw_p_b, nw_R_b] = WBM.utilities.tfms.tform2posRotm(nw_H_b);
end
