function c_qv = wbm_generalizedBiasForces(varargin)
    % WBM_GENERALIZEDBIASFORCES computes the generalized bias forces C(q_j,dq_j) in the dynamics of
    % rigid-body systems.
    % It accounts the Coriolis, centrifugal and gravity forces, which are depending on the joint
    % angles q_j, the joint velocities dq_j and the floating base velocity v_b.
    %
    % For further details see:
    %   [1] Rigid Body Dynamics Algorithms, Roy Featherstone, Springer, 2008, p. 40.
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
    %
    %   OUTPUT ARGUMENTS:
    %       c_qv -- ((nDoF+6) x 1) generalized bias force vector.
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 0
            c_qv = mexWholeBodyModel('generalized-forces');
        case 5
            c_qv = mexWholeBodyModel('generalized-forces', reshape(varargin{1,1}, 9, 1), varargin{1,2}, varargin{1,3}, ...
                                                           varargin{1,4}, varargin{1,5});
        otherwise
            wbm_narginError('wbm_generalizedBiasForces');
    end
end
