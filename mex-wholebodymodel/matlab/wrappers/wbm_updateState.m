function wbm_updateState(varargin)
    % wbm_updateState updates the state of the robot model, i.e. the joint angles and velocities,
    % and the floating base velocity.
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:  none (does not exist)
    %
    %       Normal mode:
    %           q_j    -- (nDoF x 1) joint angle vector in radian
    %           dq_j   -- (nDoF x 1) joint angle velocity vector (rad/s)
    %           v_b    -- (6 x 1) floating base velocity vector
    %
    %   OUTPUT ARGUMENTS:  none
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 3
            mexWholeBodyModel('update-state', varargin{1}, varargin{2}, varargin{3});
        otherwise
            error('wbm_updateState: %s\n', wbm_errorMsg());
    end
end
