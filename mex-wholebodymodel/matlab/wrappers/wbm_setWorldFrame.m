function wbm_setWorldFrame(varargin)
    % WBM_SETWORLDFRAME sets the world frame (WF) at a given position and orientation (transformation)
    % form a specified fixed link frame (reference frame). The specified fixed link (reference link)
    % can also be a contact constraint link.
    %
    % The set base-to-world transformation wf_H_b supports, w.r.t. a given joint configuration q_j,
    % the optimized computations of all kinematic and dynamic functions of the component classes in C++.
    % It supports also the computation of the base VQ-transformation vqT_b of the current model's state.
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
            mexWholeBodyModel('set-world-frame', reshape(varargin{1,1}, 9, 1), varargin{1,2}, varargin{1,3});
        otherwise
            wbm_narginError('wbm_setWorldFrame');
    end
end
