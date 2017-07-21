function [wf_R_b, wf_p_b, v_b] = wbm_getFloatingBaseState()
    % WBM_GETFLOATINGBASESTATE returns the current state of the robot's floating base (orientation,
    % position and velocity).
    %
    % The values of the floating base state are derived from the currently stored state of the robot
    % system.
    %
    %   INPUT ARGUMENTS:  none
    %
    %   OUTPUT ARGUMENTS:
    %       wf_R_b -- (3 x 3) floating base rotation matrix (base to world frame)
    %       wf_p_b -- (3 x 1) floating base position vector (base to world frame)
    %       v_b    -- (6 x 1) generalized base velocity vector
    %
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    [wf_R_b, wf_p_b, v_b] = mexWholeBodyModel('get-base-state');
end
