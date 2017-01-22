function [wf_R_b, wf_p_b] = wbm_getWorldFrameFromFixedLink(varargin)
    % wbm_getWorldFrameFromFixedLink returns the position and the orientation of the floating base
    % w.r.t. a world frame (WF) that is intentionally set and fixed at a specified (contact) link frame.
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
    %       wf_R_b -- (3 x 3) rotation matrix (from base to world frame)
    %       wf_p_b -- (3 x 1) position vector (from base to world frame)
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 1
            [wf_R_b, wf_p_b] = computeNewWorld2Base(varargin{1});
        case 2
            [wf_R_b, wf_p_b] = computeNewWorld2Base(varargin{1}, varargin{2});
        otherwise
            error('wbm_getWorldFrameFromFixedLink: %s\n', wbm_errorMsg());
    end
end


function [wf_R_b, wf_p_b] = computeNewWorld2Base(varargin)
    [ow_vqT_b,~,~,~] = wbm_getState(); % vector-quaternion transformation (from base to old world)
    [ow_p_b, ow_R_b] = frame2posrot(ow_vqT_b);

    % homogeneous transformation matrix H (from base to old world):
    ow_H_b = eye(4,4);
    ow_H_b(1:3,1:3) = ow_R_b;
    ow_H_b(1:3,4)   = ow_p_b;

    % get the VQ-Transformation to the old world of the reference (contact) link:
    if (nargin == 1)
        ow_vqT_rlnk = wbm_forwardKinematics(varargin{1});
    else
        ow_vqT_rlnk = wbm_forwardKinematics(ow_R_b, ow_p_b, varargin{2}, varargin{1});
    end

    % compute the new homogeneous transformation matrix H (from base to new world):
    [ow_p_rlnk, ow_R_rlnk] = frame2posrot(ow_vqT_rlnk);
    ow_H_rlnk = eye(4,4);
    ow_H_rlnk(1:3,1:3) = ow_R_rlnk;
    ow_H_rlnk(1:3,4)   = ow_p_rlnk;

    nw_H_b = ow_H_rlnk \ ow_H_b;

    % get the new position and rotation (from base to world frame):
    wf_p_b = nw_H_b(1:3,4);
    wf_R_b = nw_H_b(1:3,1:3);
end
