function [] = wbm_modelInitialise(varargin)
    % WMB_MODELINITIALISE initialises the whole body model
    %
    %   Arguments :
    %       Optimised Mode : No arguments
    %       Normal Mode :  robot-name as a string (URDF must exist in the whole body
    %       interface folder)
    %   Returns :   None
    %
    % Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
    % Genova, Dec 2014

    switch(nargin)
        case 0
            mexWholeBodyModel('model-initialize');
        case 1
            mexWholeBodyModel('model-initialize', varargin{1});
        otherwise
            disp('modelInitialise : Incorrect number of arguments, check docs');
    end
end
