function h_c = wbm_centroidalMomentum(varargin)
    % WBM_CENTROIDALMOMENTUM computes the centroidal momentum h_c of a robot system.
    % It is a function of the state variables q_j, dq_j and the generalized base velocity v_b.
    %
    % The centroidal momentum of a humanoid robot is the sum of all link moments projected
    % to the Center of Mass (CoM) of the robot.
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
    %       h_c -- (6 x 1) centroidal moment vector.
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 0
            h_c = mexWholeBodyModel('centroidal-momentum');
        case 5
            h_c = mexWholeBodyModel('centroidal-momentum', reshape(varargin{1,1}, 9, 1), varargin{1,2}, varargin{1,3}, ...
                                                           varargin{1,4}, varargin{1,5});
        otherwise
            wbm_narginError('wbm_centroidalMomentum');
    end
end
