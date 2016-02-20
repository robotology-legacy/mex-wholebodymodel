classdef wbmHumanoidConfig < WBM.wbmBaseRobotConfig
    properties
        jpos_head@double      vector
        jpos_torso@double     vector
        jpos_leftArm@double   vector 
        jpos_leftHand@double  vector
        jpos_leftLeg@double   vector
        jpos_leftFeet@double  vector
        jpos_rightArm@double  vector
        jpos_rightHand@double vector
        jpos_rightLeg@double  vector
        jpos_rightFeet@double vector
    end
end