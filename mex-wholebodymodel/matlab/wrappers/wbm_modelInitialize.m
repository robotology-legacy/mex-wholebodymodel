function wbm_modelInitialize(varargin)
    % wbm_modelInitialize initializes the whole body model of the robot.
    %
    %   INPUT ARGUMENTS:
    %       Optimized mode:  no arguments
    %
    %       Normal mode:
    %           robot_name -- string matching name of the robot model. (*)
    %
    %       (*) The URDF-file of the robot model must exist in the directory of the
    %           yarpWholeBodyInterface library.
    %
    %   OUTPUT ARGUMENTS:  none
    %
    % Author: Naveen Kuppuswamy (naveen.kuppuswamy@iit.it); Genova, Dec 2014
    % Modified by: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017

    switch nargin
        case 0
            mexWholeBodyModel('model-initialize');
        case 1
            mexWholeBodyModel('model-initialize', varargin{1});
        otherwise
            error('wbm_modelInitialize: %s\n', wbm_errorMsg());
    end
end
