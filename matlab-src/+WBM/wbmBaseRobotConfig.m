classdef wbmBaseRobotConfig
    properties
       ndof@uint16   scalar
       nCstrs@uint16 scalar
       cstrLinkNames@cell = {};
       dampCoeff@double scalar
       initStateParams  = WBM.wbmStateParams;
       stvLen@uint16    scalar
    end
end