function g_q = wbm_gravityBiasForces(varargin)
    % WBM_GRAVITYBIASFORCES computes the gravity bias forces G(q_j) in the dynamics of rigid-body
    % systems.
    %
    % This function is derived from the generalized bias force function and depends only on the
    % given joint configuration q_j.
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:  no arguments
    %
    %       Normal mode:
    %           wf_R_b -- (3 x 3) rotation matrix from base to world frame
    %           wf_p_b -- (3 x 1) position vector from base to world frame
    %           q_j    -- (nDoF x 1) joint angle vector in radian
    %
    %   OUTPUT ARGUMENTS:
    %       g_q -- ((nDoF+6) x 1) gravity bias force vector.
    %
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 0
            g_q = mexWholeBodyModel('gravity-forces');
        case 3
            g_q = mexWholeBodyModel('gravity-forces', reshape(varargin{1}, 9, 1), varargin{2}, varargin{3});
        otherwise
            error('wbm_gravityBiasForces: %s\n', wbm_errorMsg());
    end
end
