classdef wbmBasicRobotConfig
    properties
       ndof@int16
       nCstrs@int16
       cstrLinkNames
       dampCoeff@double
       qj_init = zeros(ndof,1);
    end
end