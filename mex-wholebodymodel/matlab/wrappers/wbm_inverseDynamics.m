function tau_j = wbm_inverseDynamics(varargin)
    % wbm_centroidalMomentum computes the inverse dynamics of a rigid-body system.
    %
    % It depends on the state variables q_j, dq_j and v_b, and also on the joint angle accelerations
    % dqq_j and the floating base accelerations dv_b.
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:  no arguments
    %
    %       Normal mode:
    %           wf_R_b -- (3 x 3) rotation matrix from base to world frame
    %           wf_p_b -- (3 x 1) position vector from base to world frame
    %           q_j    -- (nDoF x 1) joint angle vector in radian
    %           dq_j   -- (nDoF x 1) joint angle velocity vector (rad/s)
    %           v_b    -- (6 x 1) floating base velocity vector
    %           ddq_j  -- (nDoF x 1) joint angle acceleration vector (rad/s^2)
    %           dv_b   -- (6 x 1) floating base acceleration vector
    %
    %   OUTPUT ARGUMENTS:
    %       tau_j -- ((nDoF+6) x 1) generalized force vector at the joints and base.
    %
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 2
            tau_j = mexWholeBodyModel('inverse-dynamics', varargin{1}, varargin{2});
        case 7
            tau_j = mexWholeBodyModel('inverse-dynamics', reshape(varargin{1}, 9, 1), varargin{2}, varargin{3}, ...
                                                          varargin{4}, varargin{5}, varargin{6}, varargin{7});
        otherwise
            error('wbm_inverseDynamics: %s\n', wbm_errorMsg());
    end
end
