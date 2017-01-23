function djdq_lnk = wbm_dJdq(varargin)
    % WBM_DJDQ computes the product of the time derivative of the Jacobian and the joint velocities,
    % w.r.t. to the current state and the specified link (frame) of the robot.
    %
    % Useful for Operational Space Control and for calculating the external contact constraints.
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:
    %           urdf_link_name -- string matching URDF name of the link (frame)
    %
    %       Normal mode:
    %                   wf_R_b -- (3 x 3) rotation matrix from base to world frame
    %                   wf_p_b -- (3 x 1) position vector from base to world frame
    %                      q_j -- (nDoF x 1) joint angle vector in radian
    %                     dq_j -- (nDoF x 1) joint angle velocity vector (rad/s)
    %                      v_b -- (6 x 1) floating base velocity vector
    %           urdf_link_name -- string matching URDF name of the link (frame)
    %
    %   OUTPUT ARGUMENTS:
    %       djdq_lnk -- (6 x 1) bias acceleration vector (from link (frame) to world frame) in
    %                   operational space.
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 1
            djdq_lnk = mexWholeBodyModel('dJdq', varargin{1});
        case 6
            djdq_lnk = mexWholeBodyModel('dJdq', reshape(varargin{1}, 9, 1), varargin{2}, varargin{3}, varargin{4}, varargin{5}, varargin{6});
        otherwise
            error('wbm_dJdq: %s\n', wbm_errorMsg());
    end
end
