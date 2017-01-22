function wf_H_lnk = wbm_transformationMatrix(varargin)
    % wbm_transformationMatrix computes the homogeneous transformation matrix H of a specified
    % link (frame) of the robot w.r.t. the current joint configuration q_j.
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:
    %           urdf_link_name -- string matching URDF name of the link (frame)
    %
    %       Normal mode:
    %                   wf_R_b -- (3 x 3) rotation matrix from base to world frame
    %                   wf_p_b -- (3 x 1) position vector from base to world frame
    %                      q_j -- (nDoF x 1) joint angle vector in radian
    %           urdf_link_name -- string matching URDF name of the link (frame)
    %
    %   OUTPUT ARGUMENTS:
    %       wf_H_lnk -- (4 x 4) homogenous transformation matrix (from link (frame) to world frame).
    %
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 1
            wf_H_lnk = mexWholeBodyModel('transformation-matrix', varargin{1});
        case 4
            wf_H_lnk = mexWholeBodyModel('transformation-matrix', reshape(varargin{1}, 9, 1), varargin{2}, varargin{3}, varargin{4});
        otherwise
            error('wbm_transformationMatrix: %s\n', wbm_errorMsg());
    end
end
