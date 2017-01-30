function [vqT_b, q_j, v_b, dq_j] = wbm_getState()
    % WBM_GETSTATE obtains the currently stored state of the system (joint angles and velocities,
    % floating base velocity and base VQ-Transformation).
    %
    %   INPUT ARGUMENTS:  none
    %
    %   OUTPUT ARGUMENTS:
    %       vqT_b -- (7 x 1) vector-quaternion transformation from base to world frame (*)
    %       q_j   -- (nDoF x 1) joint angle vector in radian
    %       v_b   -- (6 x 1) floating base velocity vector
    %       dq_j  -- (nDoF x 1) joint angle velocity vector (rad/s)
    %
    %   (*) The first 3 elements of the vector representing the position and the
    %       remaining 4 elements are the orientation in quaternions.
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    [vqT_b, q_j, v_b, dq_j] = mexWholeBodyModel('get-state');
end
