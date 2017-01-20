function M = wbm_massMatrix(varargin)
    % wbm_massMatrix computes the mass matrix of the floating-base robot w.r.t. the given joint
    % configuration q_j.
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:  no arguments
    %
    %       Normal mode:
    %           wf_R_b -- (3 x 3) rotation matrix from base to world frame
    %           wf_p_b -- (3 x 1) position vector from base to world frame
    %           q_j    -- (nDoF x 1) joint angle vector in radian
    %
    %   OUTPUT ARGUMENTS:
    %       M -- ((nDoF+6) x (nDoF+6)) floating base mass matrix.
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017

    switch nargin
        case 0
            M = mexWholeBodyModel('mass-matrix');
        case 3
            M = mexWholeBodyModel('mass-matrix', reshape(varargin{1}, [], 1), varargin{2}, varargin{3});
        otherwise
            error('wbm_massMatrix: %s\n', wbm_errorMsg());
    end
end
