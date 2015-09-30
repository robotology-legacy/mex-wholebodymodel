classdef wbmBasicRobotConfig
    properties
       ndof@int16
       nCstrs@int16
       cstrLinkNames = {};
       dampCoeff@double
       wb_initState@wbmState
    end
end