function wf_J_lnk = wbm_jacobian(varargin)
    % wbm_jacobian computes the Jacobian of a specified link (frame) of the robot w.r.t. the current
    % joint configuration q_j.
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
    %       wf_J_lnk -- (6 x (nDoF+6)) Jacobian matrix from the specified link (frame) to the world frame.
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017

    switch nargin
        case 1
            wf_J_lnk = mexWholeBodyModel('jacobian', varargin{1});
        case 4
            wf_J_lnk = mexWholeBodyModel('jacobian', reshape(varargin{1}, [], 1), varargin{2}, varargin{3}, varargin{4});
        otherwise
            error('wbm_jacobian: %s\n', wbm_errorMsg());
    end
end
