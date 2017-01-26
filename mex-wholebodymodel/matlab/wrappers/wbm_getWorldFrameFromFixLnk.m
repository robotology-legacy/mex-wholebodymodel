function [wf_p_b, wf_R_b] = wbm_getWorldFrameFromFixLnk(varargin)
    % WBM_GETWORLDFRAMEFROMFIXLNK returns the position and the orientation of the floating base w.r.t.
    % a world frame (WF) that is intentionally set and fixed at a specified (contact) link frame.
    %
    % The returned floating base position and orientation is obtained from the forward kinematics
    % w.r.t. the specified fixed link frame.
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
            [wf_p_b, wf_R_b] = computeNewWorld2Base(varargin{1});
        case 2
            [wf_p_b, wf_R_b] = computeNewWorld2Base(varargin{1}, varargin{2});
        otherwise
            error('wbm_getWorldFrameFromFixLnk: %s\n', wbm_errorMsg());
    end
end


function [nw_p_b, nw_R_b] = computeNewWorld2Base(urdf_fixed_link, q_j)
    [ow_vqT_b,~,~,~] = wbm_getState(); % vector-quaternion transformation (from base to old world)

    % get the homogeneous transformation matrix H (from base to old world):
    [ow_p_b, ow_R_b] = WBM.utilities.frame2posRotm(ow_vqT_b);
    ow_H_b = WBM.utilities.posRotm2tform(ow_p_b, ow_R_b);

    % get the VQ-Transformation to the old world of the reference (contact) link:
    if (nargin == 1)
        ow_vqT_rlnk = wbm_forwardKinematics(urdf_fixed_link);
    else
        ow_vqT_rlnk = wbm_forwardKinematics(ow_R_b, ow_p_b, q_j, urdf_fixed_link);
    end

    % compute the new homogeneous transformation matrix H (from base to new world):
    ow_H_rlnk = WBM.utilities.frame2tform(ow_vqT_rlnk);
    nw_H_b = ow_H_rlnk \ ow_H_b;

    % get the new position and rotation (from base to new world):
    [nw_p_b, nw_R_b] = WBM.utilities.tform2posRotm(nw_H_b);
end
