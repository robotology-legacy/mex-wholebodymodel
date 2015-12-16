classdef wbmBaseRobotConfig
    properties
       ndof@uint16        scalar
       nCstrs@uint16      scalar
       cstrLinkNames@cell vector = {};
       dampCoeff@double   scalar
       initStateParams@WBM.wbmStateParams = WBM.wbmStateParams;
       stvLen@uint16      scalar
    end
end
