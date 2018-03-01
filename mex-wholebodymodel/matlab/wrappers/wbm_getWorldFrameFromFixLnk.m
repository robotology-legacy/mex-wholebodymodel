function [wf_p_b, wf_R_b] = wbm_getWorldFrameFromFixLnk(varargin)
    % WBM_GETWORLDFRAMEFROMFIXLNK returns the position and the orientation of the floating base w.r.t.
    % a world frame (WF) that is intentionally set and fixed at a specified link frame. The specified
    % fixed link (reference link) can also be a contact constraint link.
    %
    % The returned floating base position and orientation is obtained from the forward kinematics
    % w.r.t. the specified fixed link frame (reference frame).
    %
    % Note: The default fixed link (floating base link) may be different for each YARP-based robot,
    % and the selection of the floating base link also depends from the given situation. For example
    % the default fixed link of the iCub humanoid robot is "l_sole".
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:
    %           urdf_fixed_link -- string matching URDF name of the fixed link frame
    %
    %       Normal mode:
    %           urdf_fixed_link -- string matching URDF name of the fixed link frame
    %                       q_j -- (nDoF x 1) joint angle vector in radian
    %
    %   OUTPUT ARGUMENTS:
    %       wf_p_b -- (3 x 1) position vector (from base to world frame)
    %       wf_R_b -- (3 x 3) rotation matrix (from base to world frame)
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
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
