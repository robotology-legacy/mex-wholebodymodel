function [q_j, vqT_b, dq_j, v_b] = wbm_getState(varargin)
    % wbm_getState obtains the currently stored state of the system (joint angles and velocities,
    % floating base velocity and base VQ-Transformation).
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:  no arguments
    %       Normal mode:     no arguments
    %
    %   OUTPUT ARGUMENTS:
    %       q_j   -- (nDoF x 1) joint angle vector in radian
    %       vqT_b -- (7 x 1) vector-quaternion transformation from base to world frame (*)
    %       dq_j  -- (nDoF x 1) joint angle velocities
    %       v_b   -- (6 x 1) floating base velocity
    %
    %   (*) The first 3 elements of the vector representing the position and the
    %       remaining 4 elements are the orientation in quaternions.
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017

    switch nargin
        case 0
            [q_j, vqT_b, dq_j, v_b] = mexWholeBodyModel('get-state');
        otherwise
            error('wbm_getState: %s\n', wbm_errorMsg());
    end
end
