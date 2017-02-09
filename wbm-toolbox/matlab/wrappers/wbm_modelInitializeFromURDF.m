function wbm_modelInitializeFromURDF(varargin)
    % WBM_MODELINITIALIZEFROMURDF initializes the whole body model with a given URDF-file of a
    % YARP-based robot.
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:  none (does not exist)
    %
    %       Normal mode:
    %           urdf_file_name -- string with the path to the URDF-file to read.
    %
    %   OUTPUT ARGUMENTS:  none
    %
    % Author: Silvio Traversaro (silvio.traversaro@iit.it); Genova, Nov 2015
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    switch nargin
        case 1
            mexWholeBodyModel('model-initialize-urdf', varargin{1});
        otherwise
            error('wbm_modelInitializeFromURDF: %s\n', wbm_errorMsg());
    end
end
