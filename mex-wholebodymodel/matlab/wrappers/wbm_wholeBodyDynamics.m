function [M, c_qv, h_c] = wbm_wholeBodyDynamics(varargin)
    % WBM_WHOLEBODYDYNAMICS computes the mass matrix, the generalized bias forces and the centroidal
    % momentum of the floating base robot w.r.t. the joint angles and velocities q_j, dq_j and the
    % generalized base velocity v_b.
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
    %       M    -- ((nDoF+6) x (nDoF+6)) floating base mass matrix.
    %       c_qv -- ((nDoF+6) x 1) generalized bias force vector.
    %       h_c  -- (6 x 1) centroidal moment vector.
    %
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 0
            M    = mexWholeBodyModel('mass-matrix');
            c_qv = mexWholeBodyModel('generalized-forces');
            h_c  = mexWholeBodyModel('centroidal-momentum');
        case 5
            wf_R_b_arr = reshape(varargin{1,1}, 9, 1);
            M    = mexWholeBodyModel('mass-matrix', wf_R_b_arr, varargin{1,2}, varargin{1,3});
            c_qv = mexWholeBodyModel('generalized-forces', wf_R_b_arr,varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5});
            h_c  = mexWholeBodyModel('centroidal-momentum', wf_R_b_arr, varargin{1,2}, varargin{1,3}, varargin{1,4}, varargin{1,5});
        otherwise
            wbm_narginError('wbm_wholeBodyDynamics');
    end

end
