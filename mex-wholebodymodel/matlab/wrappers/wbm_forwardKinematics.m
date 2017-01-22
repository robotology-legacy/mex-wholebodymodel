function vqT_lnk = wbm_forwardKinematics(varargin)
    % wbm_forwardKinematics computes the forward kinematic transformation vector to a specified
    % link (frame) w.r.t. the current joint configuration q_j.
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
    %       vqT_lnk -- (7 x 1) vector-quaternion transformation (from link (frame) to world frame). (*)
    %
    %   (*) The first 3 elements of the vector representing the position and the
    %       remaining 4 elements are the orientation in quaternions.
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 1
            vqT_lnk = mexWholeBodyModel('forward-kinematics', varargin{1});
        case 4
            vqT_lnk = mexWholeBodyModel('forward-kinematics', reshape(varargin{1}, 9, 1), varargin{2}, varargin{3}, varargin{4});
        otherwise
            error('wbm_forwardKinematics: %s\n', wbm_errorMsg());
    end
end
