function tau_j = wbm_inverseDynamics(varargin)
    % WBM_CENTROIDALMOMENTUM computes the inverse dynamics of a rigid-body system.
    %
    % It depends on the state variables q_j, dq_j and v_b, and also on the joint angle accelerations
    % dqq_j and the generalized base accelerations dv_b.
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
    %           ddq_j  -- (nDoF x 1) joint angle acceleration vector (rad/s^2)
    %           dv_b   -- (6 x 1) generalized base acceleration vector
    %
    %   OUTPUT ARGUMENTS:
    %       tau_j -- ((nDoF+6) x 1) generalized force vector at the joints and base.
    %
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 2
            tau_j = mexWholeBodyModel('inverse-dynamics', varargin{1,1}, varargin{1,2});
        case 7
            tau_j = mexWholeBodyModel('inverse-dynamics', reshape(varargin{1,1}, 9, 1), varargin{1,2}, varargin{1,3}, ...
                                                          varargin{1,4}, varargin{1,5}, varargin{1,6}, varargin{1,7});
        otherwise
            wbm_narginError('wbm_inverseDynamics');
    end
end
