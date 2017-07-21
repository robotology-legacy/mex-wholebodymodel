function wbm_updateState(varargin)
    % WBM_UPDATESTATE updates the state of the robot model, i.e. the joint angles and velocities,
    % and the generalized base velocity of the floating base.
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:  none (does not exist)
    %
    %       Normal mode:
    %           q_j    -- (nDoF x 1) joint angle vector in radian
    %           dq_j   -- (nDoF x 1) joint angle velocity vector (rad/s)
    %           v_b    -- (6 x 1) generalized base velocity vector
    %
    %   OUTPUT ARGUMENTS:  none
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 3
            mexWholeBodyModel('update-state', varargin{1,1}, varargin{1,2}, varargin{1,3});
        otherwise
            wbm_narginError('wbm_updateState');
    end
end
