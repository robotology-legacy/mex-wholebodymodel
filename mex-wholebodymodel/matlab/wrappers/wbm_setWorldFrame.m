function wbm_setWorldFrame(varargin)
    % WBM_SETWORLDFRAME sets the world frame (WF), which is fixed at a specified (contact) link frame,
    % to a given transformation (position & orientation).
    %
    % With the help of the WF-transformation, the transformation vqT_b from the base to the WF can be
    % computed afterwards w.r.t. a given joint configuration q_j. The base-transformation, or
    % base VQ-Transformation vqT_b supports the optimized computations of all kinematic/dynamic
    % functions of the component classes.
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:  none (does not exist)
    %
    %       Normal mode:
    %           wf_R_b -- (3 x 3) rotation matrix from base to world frame
    %           wf_p_b -- (3 x 1) position vector from base to world frame
    %           g_wf   -- (3 x 1) gravity vector in the world frame
    %
    %   OUTPUT ARGUMENTS:  none
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 3
            mexWholeBodyModel('set-world-frame', reshape(varargin{1}, 9, 1), varargin{2}, varargin{3});
        otherwise
            error('wbm_setWorldFrame: %s\n', wbm_errorMsg());
    end
end

