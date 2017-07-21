function c_qv = wbm_coriolisBiasForces(varargin)
    % WBM_CORIOLISBIASFORCES computes the bias Coriolis and centrifugal forces C(q_j,dq_j) in the
    % dynamics of rigid-body systems.
    %
    % This function is derived from the generalized bias force function and has the same
    % dependencies (q_j, dq_j, v_b) as the parent-function.
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:  no arguments
    %
    %       Normal mode:
    %           wf_R_b -- (3 x 3) rotation matrix from base to world frame
    %           wf_p_b -- (3 x 1) position vector from base to world frame
    %           q_j    -- (nDoF x 1) joint angle vector in radian
    %           dq_j   -- (nDoF x 1) joint angle velocity vector (rad/s)
    %           v_b    -- (6 x 1) generalized base velocity vector
    %
    %   OUTPUT ARGUMENTS:
    %       c_qv -- ((nDoF+6) x 1) bias Coriolis and centrifugal force vector.
    %
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 0
            c_qv = mexWholeBodyModel('coriolis-forces');
        case 5
            c_qv = mexWholeBodyModel('coriolis-forces', reshape(varargin{1,1}, 9, 1), varargin{1,2}, varargin{1,3}, ...
                                                        varargin{1,4}, varargin{1,5});
        otherwise
            wbm_narginError('wbm_coriolisBiasForces');
    end
end
