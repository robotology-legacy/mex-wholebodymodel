classdef iCubConfig
    properties
       ndof@int16
       nCstrs@int16
       cstrLinkNames
       dampCoeff@double
       torso
       leftArm
       leftLeg
       rightArm
       rightLeg
    end
end